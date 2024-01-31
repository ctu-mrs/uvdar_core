#include "omta.h"

using namespace uvdar;

OMTA::OMTA(const loadedParamsForOMTA& i_params){

    *loaded_params_ = i_params;
    extended_search_ = std::make_unique<ExtendedSearch>(loaded_params_->decay_factor);
}

void OMTA::setDebugFlags(bool i_debug){
    debug_ = i_debug;
}

void OMTA::updateFramerate(double input) {
  if (input > 1.0)
    framerate_ = input;
}

bool OMTA::setSequences(std::vector<std::vector<bool>> i_sequences){
  
    original_sequences_ = i_sequences;
    matcher_ = std::make_unique<SignalMatcher>(original_sequences_, loaded_params_->allowed_BER_per_seq);
    if(original_sequences_.size() == 0)
        return false;

    if((int)original_sequences_[0].size() < loaded_params_->max_zeros_consecutive){
        ROS_ERROR("[OMTA]: The wanted number of consecutive zeros is higher than the sequence length! Sequence cannot be set. Returning..");
        return false;
    }
    return true;
}

void OMTA::processBuffer(const uvdar_core::ImagePointsWithFloatStampedConstPtr pts_msg) {

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

void OMTA::findClosestPixelAndInsert(std::vector<PointState> & current_frame) {   
    
    std::vector<seqPointer> p_gen_seq;
    {
        std::scoped_lock lock(mutex_gen_sequences_);
        p_gen_seq = gen_sequences_;
    }    
    std::vector<PointState> no_nn;
    for(auto & curr_point : current_frame){
        bool nn = false;
        {
            std::scoped_lock lock(mutex_gen_sequences_);
            for(auto seq = p_gen_seq.begin(); seq != p_gen_seq.end(); ++seq){
                PointState last_inserted = (*seq)->end()[-1];
                cv::Point2d bb_left_top = last_inserted.point - cv::Point2d(loaded_params_->max_px_shift);
                cv::Point2d bb_right_bottom = last_inserted.point + cv::Point2d(loaded_params_->max_px_shift);
                if(extended_search_->isInsideBB( curr_point.point, bb_left_top, bb_right_bottom)){
                    nn = true;
                    insertPointToSequence(**seq, curr_point);    
                    p_gen_seq.erase(seq);
                    break;
                }else{
                    nn = false;
                }
            }
        }
        if(nn == false){
            no_nn.push_back(curr_point);
        }
    }
    expandedSearch(no_nn, p_gen_seq);
}

void OMTA::expandedSearch(std::vector<PointState>& no_nn_current_frame, std::vector<seqPointer>& sequences_no_insert){
    std::scoped_lock lock(mutex_gen_sequences_);

    if((int)no_nn_current_frame.size() != 0){
        double insert_time = no_nn_current_frame[0].insert_time.toSec() + prediction_margin_;
        // std::vector<SeqWithTrajectory> sequences;
        // for(int k = 0; k < (int)sequences_no_insert.size(); ++k){
        for(auto it_seq = sequences_no_insert.begin();  it_seq != sequences_no_insert.end();){

            if((*it_seq)->size() == 0)continue;
            
            if(!checkSequenceValidityWithNewInsert(*it_seq)){
                continue;
            }


            std::vector<double> x,y,time;
            for(const auto point : **it_seq){
                if(point.led_state){
                    x.push_back(point.point.x);
                    y.push_back(point.point.y);
                    time.push_back(point.insert_time.toSec());
                }
            }

            PointState& last_point = (*it_seq)->end()[-1]; 
            PredictionStatistics x_predictions = selectStatisticsValues(x, time, insert_time);
            PredictionStatistics y_predictions = selectStatisticsValues(y, time, insert_time);
            if( !x_predictions.poly_reg_computed || !y_predictions.poly_reg_computed){
                ++it_seq;
                continue;
            }
            last_point.x_statistics = x_predictions;
            last_point.y_statistics = y_predictions;
            double x_predicted = last_point.x_statistics.predicted_coordinate;
            double y_predicted = last_point.y_statistics.predicted_coordinate;

            last_point.x_statistics.confidence_interval = ( last_point.x_statistics.confidence_interval > (loaded_params_->max_px_shift.x * 2) ) ? (loaded_params_->max_px_shift.x * 2) : last_point.x_statistics.confidence_interval;
            last_point.y_statistics.confidence_interval = ( last_point.y_statistics.confidence_interval > (loaded_params_->max_px_shift.y * 2) ) ? (loaded_params_->max_px_shift.y * 2) : last_point.y_statistics.confidence_interval;

            last_point.x_statistics.confidence_interval = ( last_point.x_statistics.confidence_interval < (loaded_params_->max_px_shift.x)) ? (loaded_params_->max_px_shift.x) : last_point.x_statistics.confidence_interval;
            last_point.y_statistics.confidence_interval = ( last_point.y_statistics.confidence_interval < (loaded_params_->max_px_shift.x)) ? (loaded_params_->max_px_shift.x) : last_point.y_statistics.confidence_interval;

            double x_conf = last_point.x_statistics.confidence_interval;
            double y_conf = last_point.y_statistics.confidence_interval; 
             
            cv::Point2d bb_left_top = cv::Point2d( (x_predicted - x_conf), (y_predicted - y_conf) );
            cv::Point2d bb_right_bottom = cv::Point2d( (x_predicted + x_conf), (y_predicted + y_conf) );
            
            if(debug_){
                std::cout << "[OMTA]: Predicted Point: x = " << x_predicted << " y = " << y_predicted << " Prediction Interval: x = " << x_conf << " y = " << y_conf << " seq_size " << x.size();
                std::cout << "\n";
            }

            bool deleted = false;
            for(auto it_frame = no_nn_current_frame.begin(); it_frame != no_nn_current_frame.end();){
                if(extended_search_->isInsideBB(it_frame->point, bb_left_top, bb_right_bottom)){
                    it_frame->x_statistics = last_point.x_statistics;
                    it_frame->y_statistics = last_point.y_statistics;
                    insertPointToSequence(*(*it_seq), *it_frame);
                    it_frame = no_nn_current_frame.erase(it_frame);
                    it_seq = sequences_no_insert.erase(it_seq); 
                    deleted = true;
                    break;
                }else{   
                    ++it_frame;
                }
            }
            if(!deleted){
                ++it_seq;
            }
        }
    }


    // for sequences with no newly inserted point, add virtual point
    for(auto seq : sequences_no_insert){
        insertVPforSequencesWithNoInsert(seq);
    }


    // delete the sequences that are over the max_buffer_length. Elements are deleted from the back of sequence vector
    if(loaded_params_->max_buffer_length < (int)gen_sequences_.size()){
        ROS_ERROR("[OMTA]: The maximal excepted buffer length of %d is reached! %d points will be discarded. Please consider to set the parameter \"_max_buffer_length_\" higher, if the memory has the capacity.", loaded_params_->max_buffer_length, (int)no_nn_current_frame.size());

        int diff = (int)gen_sequences_.size() - loaded_params_->max_buffer_length;
        for(int i = 0; i < diff; ++i){
            gen_sequences_.erase(gen_sequences_.end() - 1); 
        }
        return;
    }

    // for the points, still no NN found -> start new sequence
    for(auto point : no_nn_current_frame){
        std::vector<PointState> vect;
        vect.reserve(loaded_params_->stored_seq_len_factor*original_sequences_[0].size());
        vect.emplace_back(point);
        gen_sequences_.emplace_back(std::make_shared<std::vector<PointState>>(vect));
    }
    

}

bool OMTA::checkSequenceValidityWithNewInsert(const seqPointer & seq){
    
    if((int)seq->size() >  loaded_params_->max_ones_consecutive){
        int cnt = 0;
        for(int i = 0; i < loaded_params_->max_ones_consecutive; ++i){
            if(seq->end()[-(i+1)].led_state == true) ++cnt; 
        }
        if(cnt >  loaded_params_->max_ones_consecutive) return false; 
    }

    if((int)seq->size() > loaded_params_->max_zeros_consecutive){
        int cnt =0;
        for(int i = 0; i < loaded_params_->max_zeros_consecutive; ++i){
             if(!(seq->end()[-(i+1)].led_state)) ++cnt;
        }
        if(cnt > loaded_params_->max_zeros_consecutive) return false;
    }
    return true; 
}

void OMTA::insertPointToSequence(std::vector<PointState> & sequence, const PointState signal){
    sequence.push_back(signal);            
    if(sequence.size() > (original_sequences_[0].size()* loaded_params_->stored_seq_len_factor)){
        sequence.erase(sequence.begin());
    }
}

void OMTA::insertVPforSequencesWithNoInsert(seqPointer & seq){
    PointState pVirtual;
    pVirtual = seq->end()[-1];
    pVirtual.insert_time = ros::Time::now();
    pVirtual.led_state = false;
    insertPointToSequence(*seq, pVirtual);
}

PredictionStatistics OMTA::selectStatisticsValues(const std::vector<double>& values, const std::vector<double>& time, const double& insert_time){

    auto weight_vect = extended_search_->calcNormalizedWeightVect(time);
    
    PredictionStatistics statistics;
    statistics.mean_independent = extended_search_->calcWeightedMean(time, weight_vect); 
    statistics.time_pred = insert_time;
    statistics.poly_reg_computed = false;
    int poly_order = loaded_params_->poly_order;

    if((int)values.size() < poly_order && values.size() > 0){
        poly_order = values.size();
    }

    if((int)values.size() > 1){

        auto [coeff, predicted_vals_past] = extended_search_->polyReg(values, time, weight_vect, poly_order);
        statistics.coeff = coeff;
        statistics.predicted_vals_past = predicted_vals_past;

        bool all_coeff_zero = std::all_of(coeff.begin(), coeff.end(), [](double coeff){return coeff == 0.0;});
        if(!all_coeff_zero){ 
            for(int i = 0; i < (int)coeff.size(); ++i){
                statistics.predicted_coordinate += coeff[i]*pow(insert_time, i);
            }
        }

        statistics.confidence_interval = extended_search_->confidenceInterval(statistics, time, values, weight_vect, loaded_params_->conf_probab_percent);
        statistics.poly_reg_computed = true;
    }
        
    statistics.extended_search = true;
    return statistics;
}

void OMTA::cleanPotentialBuffer(){

    std::scoped_lock lock(mutex_gen_sequences_);
    for( auto it_seq = gen_sequences_.begin(); it_seq != gen_sequences_.end();){
        bool deleted = false;
        int number_zeros_till_seq_deleted = (loaded_params_->max_zeros_consecutive + loaded_params_->allowed_BER_per_seq);
        if((int)((*it_seq)->size()) > number_zeros_till_seq_deleted){
            int cnt = 0;
            for(auto it_seq_element : *(*it_seq)){
                if(!it_seq_element.led_state){
                    cnt++;
                    if(cnt > number_zeros_till_seq_deleted)
                        break;
                }else{
                    cnt = 0;
                }
            }
            if(cnt > number_zeros_till_seq_deleted){
                deleted = true;
                it_seq = gen_sequences_.erase(it_seq);
                continue;
            }
        }
        if(!deleted)++it_seq;
    }
}

std::vector<std::pair<seqPointer, int>> OMTA::getResults(){

    std::scoped_lock lock(mutex_gen_sequences_);
    std::vector<std::pair<seqPointer, int>> retrieved_signals;
    if(debug_) std::cout << "[OMTA]: The retrieved signals:{\n";
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
        auto sequence_copy = sequence; 
        retrieved_signals.push_back(std::make_pair(sequence_copy, id));
    }
    if(debug_)std::cout << "}\n";
    
    return retrieved_signals;
}

OMTA::~OMTA() {
}
