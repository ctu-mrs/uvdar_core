#include "alternativeHT.h"

using namespace uvdar;

alternativeHT::alternativeHT(const loadedParamsForAHT& params){
    max_pixel_shift_.x = params.max_pixel_shift.x;
    max_pixel_shift_.y = params.max_pixel_shift.y;
    communication_mode_  = params.communication_mode;
    stored_seq_len_factor_ = params.stored_seq_len_factor;
    poly_order_ = params.poly_order;
    extended_search_ = std::make_unique<ExtendedSearch>(params.decay_factor, poly_order_);
    conf_probab_percent_ = params.conf_probab_percent;
    seq_overlap_probab_percent_ = params.seq_overlap_probab_percent;
    threshold_values_len_for_poly_reg_ = params.threshold_values_len_for_poly_reg;
    frame_tolerance_ = params.frame_tolerance;
    
}

void alternativeHT::setDebugFlags(bool debug, bool visual_debug){
    debug_ = debug;
    visual_debug_ = visual_debug;
}

void alternativeHT::setSequences(std::vector<std::vector<bool>> i_sequences){
  
    original_sequences_ = i_sequences;
    matcher_ = std::make_unique<SignalMatcher>(original_sequences_, seq_overlap_probab_percent_);
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
            if(diff.x <= max_pixel_shift_.x && diff.y <= max_pixel_shift_.y){
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

    if((int)no_nn_current_frame.size() != 0){
        double insert_time = no_nn_current_frame[0].insert_time.toSec() + prediction_margin_;
        // std::vector<SeqWithTrajectory> sequences;
        for(int k = 0; k < (int)sequences_no_insert.size(); ++k){
            
            if(!checkSequenceValidityWithNewInsert(sequences_no_insert[k])){
                continue;
            }

            std::vector<double> x,y,time;

            // prepare for polynomial regression TODO: Maybe calculate the variance over the points for outlier rejection
            for(const auto point : *sequences_no_insert[k]){
                if(point.led_state){
                    x.push_back(point.point.x);
                    y.push_back(point.point.y);
                    time.push_back(point.insert_time.toSec());
                }
            }

            if(time.size() == 0 || x.size() != y.size() || x.size() != time.size() || sequences_no_insert[k]->size() == 0)continue;

            PointState& last_point = sequences_no_insert[k]->end()[-1]; 
            bool draw_x, draw_y;
            PredictionStatistics x_predictions = selectStatisticsValues(x, time, insert_time, max_pixel_shift_.x, draw_x);
            PredictionStatistics y_predictions = selectStatisticsValues(y, time, insert_time, max_pixel_shift_.y, draw_y);
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

            for(auto it_frame = no_nn_current_frame.begin(); it_frame != no_nn_current_frame.end();){
            // for(int i = 0; i < (int)no_nn_current_frame.size(); i++){
                // if(extended_search_->isInsideBB(no_nn_current_frame[i].point, left_top, right_bottom)){
                if(extended_search_->isInsideBB(it_frame->point, left_top, right_bottom)){
                    // no_nn_current_frame[i].extended_search = true;
                    // no_nn_current_frame[i].x_coeff = last_point.x_coeff;
                    // no_nn_current_frame[i].y_coeff = last_point.y_coeff;
                    // no_nn_current_frame[i].confidence_interval = last_point.confidence_interval;
                    // no_nn_current_frame[i].predicted = last_point.predicted;
                    it_frame->extended_search = true;
                    it_frame->x_coeff = last_point.x_coeff;
                    it_frame->y_coeff = last_point.y_coeff;
                    it_frame->confidence_interval = last_point.confidence_interval;
                    it_frame->predicted = last_point.predicted;
                    // insertPointToSequence(*sequences_no_insert[k], no_nn_current_frame[i]);
                    insertPointToSequence(*sequences_no_insert[k], *it_frame);
                    it_frame = no_nn_current_frame.erase(it_frame);
                    // no_nn_current_frame.erase(no_nn_current_frame.begin()+i);
                    sequences_no_insert.erase(sequences_no_insert.begin()+k); 
                    break;
                }else{   
                    ++it_frame;
                }
            }
        }
    }

    // for the points, still no NN found start new sequence
    for(auto point : no_nn_current_frame){
        std::vector<PointState> vect;
        vect.push_back(point);
        gen_sequences_.push_back(std::make_shared<std::vector<PointState>>(vect));
        std::cout << "start new seq\n";
    }

    // for sequences where still no insert happend
    for(auto seq : sequences_no_insert){
        insertVPforSequencesWithNoInsert(seq);
    }
}

bool alternativeHT::checkSequenceValidityWithNewInsert(const seqPointer & seq){
    
    if(communication_mode_) return true;

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
    insertPointToSequence(*seq, pVirtual);
}

PredictionStatistics alternativeHT::selectStatisticsValues(const std::vector<double>& values, const std::vector<double>& time, const double& insert_time, const int& max_pix_shift, bool & draw_poly){

    auto weight_vect = extended_search_->calculateWeightVector(time);
    auto weighted_mean = extended_search_->calcWeightedMean(values, weight_vect); 
    auto std = extended_search_->calcWSTD(values, weight_vect, weighted_mean); 

    PredictionStatistics statistics;
    statistics.mean = weighted_mean;
           
    bool confidence_interval = false, poly_reg_computed = false;

    if(std > max_pix_shift && (int)values.size() >= threshold_values_len_for_poly_reg_){

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
    double timing_tolerance = frame_tolerance_/60;
    if(framerate_ != 0){
        timing_tolerance = (1/framerate_) * frame_tolerance_; 
    }

    for(int i = 0; i < (int)gen_sequences_.size(); ++i){
        if(gen_sequences_[i]->empty()){
            continue;
        }

        if(!communication_mode_ && gen_sequences_[i]->size() > 2){
            if(!gen_sequences_[i]->end()[-1].led_state && !gen_sequences_[i]->end()[-2].led_state && !gen_sequences_[i]->end()[-3].led_state){
                gen_sequences_.erase(gen_sequences_.begin() +i);
                continue;
            }
        }

        // works but eventually not useful in that current implementation
        double insert_time = gen_sequences_[i]->end()[-1].insert_time.toSec(); 
        double diff_last_insert = std::abs(insert_time - ros::Time::now().toSec());
        if (diff_last_insert > timing_tolerance){ 
            gen_sequences_.erase(gen_sequences_.begin()+i);
            continue;
        }
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

        int id = matcher_->matchSignalWithCrossCorr(led_states);
        retrieved_signals.push_back(std::make_pair(return_seq, id));
    }
    return retrieved_signals;
}

alternativeHT::~alternativeHT() {
    // delete generatedSequences_;
}