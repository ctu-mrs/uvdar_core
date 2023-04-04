#include "alternativeHT.h"

using namespace uvdar;

alternativeHT::alternativeHT(const loadedParamsForAHT& i_params){

    *loaded_params_ = i_params;
    extended_search_ = std::make_unique<ExtendedSearch>(loaded_params_->decay_factor, loaded_params_->poly_order);
}

void alternativeHT::setDebugFlags(bool i_debug){
    debug_ = i_debug;
}

void alternativeHT::updateFramerate(double input) {
  if (input > 1.0)
    framerate_ = input;
}

bool alternativeHT::setSequences(std::vector<std::vector<bool>> i_sequences){
  
    original_sequences_ = i_sequences;
    matcher_ = std::make_unique<SignalMatcher>(original_sequences_, loaded_params_->allowed_BER_per_seq);
    if(original_sequences_.size() == 0)
        return false;

    if((int)original_sequences_[0].size() < loaded_params_->max_zeros_consecutive){
        ROS_ERROR("[Alternative_HT]: The wanted number of consecutive zeros is higher than the sequence length! Returning..");
        return false;
    }
    return true;
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
            if(diff.x <= loaded_params_->max_px_shift.x && diff.y <= loaded_params_->max_px_shift.y){
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

            for(const auto point : *sequences_no_insert[k]){
                if(point.led_state){
                    x.push_back(point.point.x);
                    y.push_back(point.point.y);
                    time.push_back(point.insert_time.toSec());
                }
            }

            if(sequences_no_insert[k]->size() == 0)continue;

            PointState& last_point = sequences_no_insert[k]->end()[-1]; 
            bool draw_x, draw_y;
            PredictionStatistics x_predictions = selectStatisticsValues(x, time, insert_time, loaded_params_->max_px_shift.x, draw_x);
            PredictionStatistics y_predictions = selectStatisticsValues(y, time, insert_time, loaded_params_->max_px_shift.y, draw_y);
            last_point.confidence_interval.x = x_predictions.confidence_interval;
            last_point.predicted.x = x_predictions.predicted_coordinate;
            last_point.x_coeff = x_predictions.coeff;

            last_point.confidence_interval.y = y_predictions.confidence_interval;
            last_point.predicted.y= y_predictions.predicted_coordinate;
            last_point.y_coeff = y_predictions.coeff;
            if(draw_x && draw_y)
            last_point.extended_search = true;
            
            // construct BB 
            cv::Point2d left_top =     last_point.predicted - last_point.confidence_interval;   
            cv::Point2d right_bottom = last_point.predicted + last_point.confidence_interval;
            left_top.x = std::floor(left_top.x);
            left_top.y = std::floor(left_top.y);
            right_bottom.x = std::ceil(right_bottom.x);
            right_bottom.y = std::ceil(right_bottom.y);

            if(debug_){
                std::cout << "[Aht]: Predicted Point: x = " << last_point.predicted.x << " y = " << last_point.predicted.y << " Prediction Interval: x = " << last_point.confidence_interval.x << " y = " << last_point.confidence_interval.y << " seq_size" << x.size();
                std::cout << "\n";
            }

            for(auto it_frame = no_nn_current_frame.begin(); it_frame != no_nn_current_frame.end();){
            // for(int i = 0; i < (int)no_nn_current_frame.size(); i++){
                // if(extended_search_->isInsideBB(no_nn_current_frame[i].point, left_top, right_bottom)){
                if(extended_search_->isInsideBB(it_frame->point, left_top, right_bottom)){
                    it_frame->extended_search = true;
                    it_frame->x_coeff = last_point.x_coeff;
                    it_frame->y_coeff = last_point.y_coeff;
                    it_frame->confidence_interval = last_point.confidence_interval;
                    it_frame->predicted = last_point.predicted;
                    insertPointToSequence(*sequences_no_insert[k], *it_frame);
                    it_frame = no_nn_current_frame.erase(it_frame);
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
    }

    // for sequences where still no insert happend
    for(auto seq : sequences_no_insert){
        insertVPforSequencesWithNoInsert(seq);
    }
}

bool alternativeHT::checkSequenceValidityWithNewInsert(const seqPointer & seq){
    
    if((int)seq->size() >  loaded_params_->max_ones_consecutive){
        int cnt = 0;
        for(int i = 0; i < loaded_params_->max_ones_consecutive; ++i){
            if(seq->end()[-(i+1)].led_state == true) ++cnt; 
        }
        if(cnt >=  loaded_params_->max_ones_consecutive) return false; 
    }

    if((int)seq->size() > loaded_params_->max_zeros_consecutive){
        int cnt =0;
        for(int i = 0; i < loaded_params_->max_zeros_consecutive; ++i){
             if(!(seq->end()[-(i+1)].led_state)) ++cnt;
        }
        if(cnt >= loaded_params_->max_ones_consecutive) return false;
    }
    return true; 
}


void alternativeHT::insertPointToSequence(std::vector<PointState> & sequence, const PointState signal){
    sequence.push_back(signal);            
    if(sequence.size() > (original_sequences_[0].size()* loaded_params_->stored_seq_len_factor)){
        sequence.erase(sequence.begin());
    }
}

void alternativeHT::insertVPforSequencesWithNoInsert(seqPointer & seq){
    PointState pVirtual;
    pVirtual = seq->end()[-1];
    pVirtual.insert_time = ros::Time::now();
    pVirtual.led_state = false;
    insertPointToSequence(*seq, pVirtual);
}

PredictionStatistics alternativeHT::selectStatisticsValues(const std::vector<double>& values, const std::vector<double>& time, const double& insert_time, const int& max_pix_shift, bool & draw_poly){

    auto weight_vect = extended_search_->calcNormalizedWeightVect(time);
    double w_mean_dependent = extended_search_->calcWeightedMean(values, weight_vect); 
    double w_mean_independent = extended_search_->calcWeightedMean(time, weight_vect); 
    auto std = extended_search_->calcWSTD(values, weight_vect, w_mean_dependent); 

    PredictionStatistics statistics;
    statistics.mean_dependent = w_mean_dependent;
    statistics.mean_independent = w_mean_independent; 
    statistics.time_pred = insert_time;
           
    bool conf_interval_bool = false, poly_reg_computed = false;

    // if(std > 1 && (int)values.size() >= threshold_values_len_for_poly_reg_){
    // if(std > 1 && (int)values.size() >= loaded_params_->threshold_values_len_for_poly_reg){

        statistics = extended_search_->polyReg(values, time, weight_vect);
        auto coeff = statistics.coeff;
        bool all_coeff_zero = std::all_of(coeff.begin(), coeff.end(), [](double coeff){return coeff == 0.0;});
        int val = 0;
        if(true){
            for(int i = 0; i < (int)coeff.size(); ++i){
                statistics.predicted_coordinate += coeff[i]*pow(insert_time, i);
            }
            statistics.predicted_coordinate = std::round(statistics.predicted_coordinate);  
            poly_reg_computed = true;
        }else{
            poly_reg_computed = false;
        }
        statistics.confidence_interval = extended_search_->confidenceInterval(statistics, time, values, weight_vect, loaded_params_->conf_probab_percent);
        conf_interval_bool = (statistics.confidence_interval == -1.0) ? false : true; 
    // }
    
    if(!poly_reg_computed){
        statistics.predicted_coordinate = std::round(w_mean_dependent); 
    }
    if(!conf_interval_bool) statistics.confidence_interval = (std < max_pix_shift) ? max_pix_shift : std*2; //TODO: TGINK ABOUT THIS

    // statistics.confidence_interval = (statistics.confidence_interval < max_pix_shift) ? max_pix_shift*2 : statistics.confidence_interval;  
    if(poly_reg_computed == false){
        draw_poly = false;
    }else draw_poly = true;

    return statistics;

}

void alternativeHT::cleanPotentialBuffer(){

    std::scoped_lock lock(mutex_gen_sequences_);
    double timing_tolerance = loaded_params_->frame_tolerance/60;
    if(framerate_ != 0){
        timing_tolerance = (1/framerate_) * loaded_params_->frame_tolerance; 
    }

    for(int i = 0; i < (int)gen_sequences_.size(); ++i){
        if(gen_sequences_[i]->empty()){
            continue;
        }

        int number_zeros_till_seq_deleted = (loaded_params_->max_zeros_consecutive + loaded_params_->allowed_BER_per_seq); // TODO:!!
        if((int)gen_sequences_[i]->size() > number_zeros_till_seq_deleted){
            int cnt = 0;
            for(int k = 0; k < number_zeros_till_seq_deleted; ++k){
                if(!(gen_sequences_[i]->end()[-(k+1)].led_state))++cnt;
            }
            if(cnt >= number_zeros_till_seq_deleted){
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
    if(debug_) std::cout << "[AHT]: The retrieved signals:{\n";
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
        if(debug_){
            std::cout << "[ ";
            for(auto state : led_states){
                if(state) std::cout << "1,";
                else std::cout << "0,";
            }
            std::cout << "]\n";
        }

        int id = matcher_->matchSignalWithCrossCorr(led_states);
        if(id >= 0 ){ 
            // gc++;
            // std::cout << "gc " << gc << "\n";
        } 
        retrieved_signals.push_back(std::make_pair(return_seq, id));
    }
    std::cout << "}\n";
    return retrieved_signals;
}

alternativeHT::~alternativeHT() {
}