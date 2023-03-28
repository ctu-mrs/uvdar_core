#include "alternativeHT.h"

using namespace uvdar;

alternativeHT::alternativeHT(double decay_factor, int poly_order, int stored_seq_len_factor, double conf_probability_percent){
    poly_order_ = poly_order;
    extended_search_ = std::make_unique<ExtendedSearch>(decay_factor, poly_order);
    stored_seq_len_factor_ = stored_seq_len_factor;
    conf_probab_percent_ = conf_probability_percent;
}

void alternativeHT::setDebugFlags(bool debug, bool visual_debug){
    debug_ = debug;
    visual_debug_ = visual_debug;
}

void alternativeHT::setSequences(std::vector<std::vector<bool>> i_sequences){
  
    original_sequences_ = i_sequences;
    matcher_ = std::make_unique<SignalMatcher>(original_sequences_);
}

void alternativeHT::updateFramerate(double input) {
  if (input > 1.0)
    framerate_ = input;
}

void alternativeHT::processBuffer(const mrs_msgs::ImagePointsWithFloatStampedConstPtr pts_msg) {
    
    std::vector<PointState> current_frame;
    for ( auto point_time_stamp : pts_msg->points) {
        PointState p;
        p.point = cv::Point(point_time_stamp.x, point_time_stamp.y);
        p.led_state = true;
        p.insert_time = pts_msg->stamp;
        current_frame.push_back(p);
    }

    findClosestPixelAndInsert(current_frame);
    cleanPotentialBuffer();

    // for(auto seq : gen_sequences_){
    //     if(seq->size() > 20){
    //         auto diff = seq->end()[-1].point - seq->end()[-18].point;
    //         if(seq->end()[-1].extended_search){
    //             auto diff_predict = seq->end()[-1].point - seq->end()[-1].predicted;
    //             if( (diff.x < 0 && diff_predict.x < 0) || (diff.y < 0 && diff_predict.y < 0) ){
    //                 std::cout << "Seq ";
    //                 int diff_i = seq->size() - 18;
    //                 for(int i = diff_i; i < seq->size(); ++i){
    //                     std::cout << "x:" << (*seq)[i].point.x << " ";
    //                 }
    //                 std::cout << " PREDICTED X: " << seq->end()[-1].predicted.x << " ";

    //                 for(int i = diff_i; i < seq->size(); ++i){
    //                     std::cout << " y:" << (*seq)[i].point.y << " ";
    //                 }
    //                 std::cout << " PREDICTED Y: " << seq->end()[-1].predicted.y << "\n";

    //             }
    //         }
    //     }
    // }
}

void alternativeHT::findClosestPixelAndInsert(std::vector<PointState> & current_frame) {   
    
    // reference to sequences used for processing in the moving average functions
    
    std::vector<seqPointer> p_gen_seq;
    {
        // TODO: double check this 
        std::scoped_lock lock(mutex_gen_sequences_);
        p_gen_seq = gen_sequences_;
    }
    
    std::vector<PointState> no_nn;
    for(auto & curr_point : current_frame){
        std::scoped_lock lock(mutex_gen_sequences_);
        bool nn = false;
        for(auto seq = p_gen_seq.begin(); seq != p_gen_seq.end(); ++seq){
            PointState last_inserted = (*seq)->end()[-1];
            cv::Point2d diff = computeXYDiff(curr_point.point, last_inserted.point);
            if(diff.x <= max_pixel_shift_x_ && diff.y <= max_pixel_shift_y_){
                nn = true;
                insertPointToSequence(**seq, curr_point);    
                p_gen_seq.erase(seq);
                break;
            }else{
                nn = false;
            }
        }
        if(nn == false){
            no_nn.push_back(curr_point);
        }
    }
    expandedSearch(no_nn, p_gen_seq);
}

cv::Point2d alternativeHT::computeXYDiff(const cv::Point2d first, const cv::Point2d second){
    cv::Point2d difference; 
    difference.x = std::abs(first.x - second.x);
    difference.y = std::abs(first.y - second.y);
    return difference;
}

void alternativeHT::insertPointToSequence(std::vector<PointState> & sequence, const PointState signal){
    sequence.push_back(signal);            
    if(sequence.size() > (original_sequences_[0].size()*stored_seq_len_factor_)){
        sequence.erase(sequence.begin());
    }
}

void alternativeHT::expandedSearch(std::vector<PointState>& no_nn_current_frame, std::vector<seqPointer>& sequences_no_insert){
    
    std::scoped_lock lock(mutex_gen_sequences_);

    // std::cout << "Sequences No Insert" << sequences_no_insert.size() << " Seq " << generatedSequences_.size() << "\n";
    if((int)no_nn_current_frame.size() != 0){
        double insert_time = no_nn_current_frame[0].insert_time.toSec() + prediction_margin_;
        // std::vector<SeqWithTrajectory> sequences;
        for(int k = 0; k < (int)sequences_no_insert.size(); ++k){
            
            if(!checkSequenceValidityWithNewInsert(sequences_no_insert[k])){
                continue;
            }
            

            std::vector<double> x,y;
            std::vector<double> time;

            // prepare for polynomial regression TODO: Maybe calculate the variance over the points for outlier rejection
            for(const auto point : *sequences_no_insert[k]){
                if(point.led_state){
                    x.push_back(point.point.x);
                    y.push_back(point.point.y);
                    time.push_back(point.insert_time.toSec());
                }
            }

            PointState& last_point = sequences_no_insert[k]->end()[-1]; 
            bool draw_x, draw_y;
            PredictionStatistics x_predictions = selectStatisticsValues(x, time, insert_time, max_pixel_shift_x_, draw_x);
            PredictionStatistics y_predictions = selectStatisticsValues(y, time, insert_time, max_pixel_shift_y_, draw_y);
            last_point.confidence_interval.x = x_predictions.confidence_interval;
            last_point.predicted.x = x_predictions.predicted_coordinate;
            last_point.x_coeff = x_predictions.coeff;

            last_point.confidence_interval.y = y_predictions.confidence_interval;
            last_point.predicted.y= y_predictions.predicted_coordinate;
            last_point.y_coeff = y_predictions.coeff;
            // if(draw_x && draw_y)
            last_point.extended_search = true;
            
            // construct BB 
            cv::Point2d left_top =     last_point.predicted - last_point.confidence_interval;   
            cv::Point2d right_bottom = last_point.predicted + last_point.confidence_interval;
            left_top.x = std::floor(left_top.x);
            left_top.y = std::floor(left_top.y);
            right_bottom.x = std::ceil(right_bottom.x);
            right_bottom.y = std::ceil(right_bottom.y);

            for(int i = 0; i < (int)no_nn_current_frame.size(); i++){
                if(extended_search_->isInsideBB(no_nn_current_frame[k].point, left_top, right_bottom)){
                    no_nn_current_frame[i].extended_search = true;
                    no_nn_current_frame[i].x_coeff = last_point.x_coeff;
                    no_nn_current_frame[i].y_coeff = last_point.y_coeff;
                    no_nn_current_frame[i].confidence_interval = last_point.confidence_interval;
                    no_nn_current_frame[i].predicted = last_point.predicted;
                    insertPointToSequence(*sequences_no_insert[k], no_nn_current_frame[i]);
                    std::vector<double> _x, _y, _time;
                    for(const auto point : *sequences_no_insert[k]){
                        if(point.led_state){
                            _x.push_back(point.point.x);
                            _y.push_back(point.point.y);
                            _time.push_back(point.insert_time.toSec());
                        }
                    }
                    PredictionStatistics x_predictions = selectStatisticsValues(_x, _time, insert_time, max_pixel_shift_x_, draw_x);
                    PredictionStatistics y_predictions = selectStatisticsValues(_y, _time, insert_time, max_pixel_shift_y_, draw_y);
                    PointState& last_point = sequences_no_insert[k]->end()[-1]; 
                    last_point.confidence_interval.x = x_predictions.confidence_interval;
                    last_point.predicted.x = x_predictions.predicted_coordinate;
                    last_point.x_coeff = x_predictions.coeff;
                    last_point.confidence_interval.y = y_predictions.confidence_interval;
                    last_point.predicted.y= y_predictions.predicted_coordinate;
                    last_point.y_coeff = y_predictions.coeff;
                    if(draw_x && draw_y) last_point.extended_search = true;
                    no_nn_current_frame.erase(no_nn_current_frame.begin()+i);
                    sequences_no_insert.erase(sequences_no_insert.begin()+k); 
                    break;
                }   
            }

            // trial and error approach
            // bool all_blinkers_equal = true; 
            // if(all_blinkers_equal){
            //     if(sequences_no_insert[k]->size() >= 2 && !sequences_no_insert[k]->end()[-1].led_state && !sequences_no_insert[k]->end()[-2].led_state){
            //         cv::Point2d min(20,20);
            //         PointState selected;
            //         int iterator_place = -1;
            //         for(int i = 0; i < (int)no_nn_current_frame.size(); ++i){        
            //             auto diff = computeXYDiff(no_nn_current_frame[i].point, sequences_no_insert[k]->end()[-1].point);
            //             if(diff.x < min.x && diff.y < min.y){
            //                 min = diff; 
            //                 selected = no_nn_current_frame[i];
            //                 iterator_place = i;
            //             }
            //         }
            //         if(iterator_place != -1){
            //             insertPointToSequence(*sequences_no_insert[k], selected);
            //             no_nn_current_frame.erase(no_nn_current_frame.begin()+iterator_place);
            //             sequences_no_insert.erase(sequences_no_insert.begin()+k);
            //             continue;
            //         }
            //     }
            // }
        }
    }

    // for the points, still no NN found start new sequence
    for(auto point : no_nn_current_frame){
        ++global_count;
        // std::cout << "GC " << global_count << "\n";
        std::vector<PointState> vect;
        vect.push_back(point);
        gen_sequences_.push_back(std::make_shared<std::vector<PointState>>(vect));
    }

    // for sequences where still no insert happend
    for(auto seq : sequences_no_insert){
        insertVPforSequencesWithNoInsert(seq);
    }
}

bool alternativeHT::checkSequenceValidityWithNewInsert(const seqPointer & seq){

    if(seq->size() > 1){
        // if the last two led states were on - no "on" point expected -> no NN search necessary
       if(seq->end()[-1].led_state == true && seq->end()[-2].led_state == true){
            return false;
        }
    }

    if(seq->size() > 2){
        // if last three led states are off, the sequence is already illegal -> no NN search necessary
        if(seq->end()[-1].led_state == false && seq->end()[-2].led_state == false && seq->end()[-3].led_state == false){
            return false;
        }
    }
    return true; 
}

void alternativeHT::insertVPforSequencesWithNoInsert(seqPointer & seq){
    PointState pVirtual;
    pVirtual = seq->end()[-1];
    pVirtual.insert_time = ros::Time::now();
    pVirtual.led_state = false;
    // pVirtual.extended_search = false;
    insertPointToSequence(*seq, pVirtual);
}

PredictionStatistics alternativeHT::selectStatisticsValues(const std::vector<double>& values, const std::vector<double>& time, const double& insert_time, const int& max_pix_shift, bool & draw_poly){

    auto weight_vect = extended_search_->calculateWeightVector(time);
    auto weighted_mean = extended_search_->calcWeightedMean(values, weight_vect); 
    auto std = extended_search_->calcWSTD(values, weight_vect, weighted_mean); 

    PredictionStatistics statistics;
    statistics.mean = weighted_mean;
           
    bool confidence_interval = false, poly_reg_computed = false;

    if(std > max_pix_shift && values.size() >= 10){

        statistics = extended_search_->polyReg(values, time, weight_vect);
        auto coeff = statistics.coeff;
        bool all_coeff_zero = std::all_of(coeff.begin(), coeff.end(), [](double coeff){return coeff == 0.0;});
        int val = 0;
        for(auto l : coeff){
            if(l == 0.0) val++;
        }
        if(!all_coeff_zero){
            for(int i = 0; i < (int)coeff.size(); ++i){
                statistics.predicted_coordinate += coeff[i]*pow(insert_time, i);
            }
            statistics.predicted_coordinate = std::round(statistics.predicted_coordinate);  
            poly_reg_computed = true;
        }else{
            poly_reg_computed = false;
        }
        if((int)values.size() > statistics.used_poly_order + 1){
            statistics.confidence_interval = extended_search_->confidenceInterval(statistics, values, weight_vect, conf_probab_percent_);
            confidence_interval = true;
        }else{
            confidence_interval = false;
        }
    }
    
    if(!poly_reg_computed){
        statistics.predicted_coordinate = std::round(weighted_mean); 
        statistics.coeff = std::vector<double>(1, 0.0);
    }
    if(!confidence_interval) statistics.confidence_interval = (std < max_pix_shift) ? max_pix_shift*2 : std*2;

    statistics.confidence_interval = (statistics.confidence_interval < 2*max_pix_shift) ? max_pix_shift*2 : statistics.confidence_interval;  
    if(poly_reg_computed == false && confidence_interval == false){
        draw_poly = false;
    }else draw_poly = true;

    return statistics;

}

void alternativeHT::cleanPotentialBuffer(){

    std::scoped_lock lock(mutex_gen_sequences_);
    double timing_tolerance = 5/60;
    if(framerate_ != 0){
        timing_tolerance = (1/framerate_) * 4; 
    }

    // std::cout << "Time margin " << timeMargin; 

    auto it = gen_sequences_.begin();
    while(it != gen_sequences_.end()){ 
        if ((*it)->empty()){
            ++it;
            continue;
        }

        if( (*it)->size() > 2){
            if(!(*it)->end()[-1].led_state && !(*it)->end()[-2].led_state && !(*it)->end()[-3].led_state){
                it = gen_sequences_.erase(it);
                continue;
            }
        }

        // works but eventually not usefull in that current implementation
        double insert_time = (*it)->end()[-1].insert_time.toSec(); 
        double diff_last_insert = std::abs(insert_time - ros::Time::now().toSec());
        if (diff_last_insert > timing_tolerance){ 
            it = gen_sequences_.erase(it);
            std::cout << "Time\n";
            continue;
        }
        ++it;

    }
}

std::vector<std::pair<std::vector<PointState>, int>> alternativeHT::getResults(){

    std::scoped_lock lock(mutex_gen_sequences_);
    std::vector<std::pair<std::vector<PointState>, int>> retrieved_signals;

    for (auto sequence : gen_sequences_){
        std::vector<bool> led_states;
        std::vector<PointState> return_seq; 
        for(auto p : *sequence){
            return_seq.push_back(p);
        }

        std::vector<PointState> selected;
        if ((int)return_seq.size() > (int)original_sequences_[0].size()){
            int diff = (int)return_seq.size() - (int)original_sequences_[0].size();
            for(int i = diff; i < (int)return_seq.size(); ++i){
                selected.push_back(return_seq[i]);
            }
        }else{

            selected = return_seq;
        }
        for (auto point : selected){
            led_states.push_back(point.led_state);
        }

        int id = findSequenceMatch(led_states);
        retrieved_signals.push_back(std::make_pair(return_seq, id));
    }
    return retrieved_signals;
}

int alternativeHT::findSequenceMatch(std::vector<bool> sequence){

    for ( int i = 2; i < (int)sequence.size(); i++) {
        if (sequence[i] == false && sequence[i-1] == false && sequence[i-2] == false){
            return -2;
        }
    }
    int id = matcher_->matchSignalWithCrossCorr(sequence);

    return id;
}

alternativeHT::~alternativeHT() {
    // delete generatedSequences_;
}