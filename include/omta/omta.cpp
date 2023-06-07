#include "omta.h"

using namespace uvdar;

OMTA::OMTA(const loadedParamsForOMTA& i_params){

    *loaded_params_ = i_params;
    extended_search_ = std::make_unique<ExtendedSearch>(loaded_params_->decay_factor, loaded_params_->poly_order);
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

void OMTA::processBuffer(const mrs_msgs::ImagePointsWithFloatStampedConstPtr pts_msg) {

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
            PredictionStatistics x_predictions = selectStatisticsValues(x, time, insert_time, loaded_params_->max_px_shift.x);
            PredictionStatistics y_predictions = selectStatisticsValues(y, time, insert_time, loaded_params_->max_px_shift.y);
            last_point.x_statistics = x_predictions;
            last_point.y_statistics = y_predictions;
            double x_predicted = last_point.x_statistics.predicted_coordinate;
            double y_predicted = last_point.y_statistics.predicted_coordinate;

            // todo: make upper limit settable, e.g. when only UAV is expected, value can set higher, than for swarming applications 
            last_point.x_statistics.confidence_interval = ( last_point.x_statistics.confidence_interval > 20.0 ) ? 20.0 : last_point.x_statistics.confidence_interval;
            last_point.y_statistics.confidence_interval = ( last_point.y_statistics.confidence_interval > 20.0 ) ? 20.0 : last_point.y_statistics.confidence_interval;

            double x_conf = last_point.x_statistics.confidence_interval;
            double y_conf = last_point.y_statistics.confidence_interval;     
             
            cv::Point2d bb_left_top = cv::Point2d( (x_predicted - x_conf), (y_predicted - y_conf) );
            cv::Point2d bb_right_bottom = cv::Point2d( (x_predicted + x_conf), (y_predicted + y_conf) );
            
            if(debug_){
                std::cout << "[OMTA]: Predicted Point: x = " << x_predicted << " y = " << y_predicted << " Prediction Interval: x = " << x_conf << " y = " << y_conf << " seq_size" << x.size();
                std::cout << "\n";
            }

            for(auto it_frame = no_nn_current_frame.begin(); it_frame != no_nn_current_frame.end();){
                if(extended_search_->isInsideBB(it_frame->point, bb_left_top, bb_right_bottom)){
                    it_frame->x_statistics = last_point.x_statistics;
                    it_frame->y_statistics = last_point.y_statistics;
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

    if(loaded_params_->max_buffer_length < (int)gen_sequences_.size()){
        ROS_ERROR("[OMTA]: The maximal excepted buffer length of %d is reached! %d points will be discarded. Please consider to set the parameter _max_buffer_length_ higher, if this effect is not wanted", loaded_params_->max_buffer_length, (int)no_nn_current_frame.size());
    }

    // for the points, still no NN found start new sequence
    for(auto point : no_nn_current_frame){

        std::vector<PointState> vect;
        vect.reserve(loaded_params_->stored_seq_len_factor*original_sequences_[0].size());
        vect.emplace_back(point);
        gen_sequences_.emplace_back(std::make_shared<std::vector<PointState>>(vect));
    }

    // for sequences where still no insert happend
    for(auto seq : sequences_no_insert){
        insertVPforSequencesWithNoInsert(seq);
    }
}

bool OMTA::checkSequenceValidityWithNewInsert(const seqPointer & seq){
    
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

PredictionStatistics OMTA::selectStatisticsValues(const std::vector<double>& values, const std::vector<double>& time, const double& insert_time, const int& max_pix_shift){

    auto weight_vect = extended_search_->calcNormalizedWeightVect(time);
    
    PredictionStatistics statistics;
    statistics.mean_dependent = extended_search_->calcWeightedMean(values, weight_vect); 
    statistics.mean_independent = extended_search_->calcWeightedMean(time, weight_vect); 
    statistics.time_pred = insert_time;
    statistics.poly_reg_computed = false;

    auto std = extended_search_->calcWSTD(values, weight_vect, statistics.mean_dependent); 

    bool conf_interval_bool = false;

    // std::cout << "here" << values.size() << " " << loaded_params_->poly_order << " std " << std << "\n";
    if((int)values.size() >= loaded_params_->poly_order  && std > loaded_params_->std_threshold_poly_reg){
    // if((int)values.size() >= loaded_params_->poly_order ){
        
        auto [coeff, predicted_vals_past] = extended_search_->polyReg(values, time, weight_vect);
        statistics.coeff = coeff;
        statistics.predicted_vals_past = predicted_vals_past;

        bool all_coeff_zero = std::all_of(coeff.begin(), coeff.end(), [](double coeff){return coeff == 0.0;});
        if(!all_coeff_zero){ 
            for(int i = 0; i < (int)coeff.size(); ++i){
                statistics.predicted_coordinate += coeff[i]*pow(insert_time, i);
            }
            statistics.poly_reg_computed = true;
        }

        statistics.confidence_interval = extended_search_->confidenceInterval(statistics, time, values, weight_vect, loaded_params_->conf_probab_percent);
        conf_interval_bool = (statistics.confidence_interval == -1.0) ? false : true; 
    }
    
    if(!statistics.poly_reg_computed){
        statistics.predicted_coordinate = statistics.mean_dependent;
    }
    
    /*  if the confidence interval is not computed and the std is smaller than the 
        expected max_pix_shift set to max_px_shift. Otherwise set the confidence_interval 
        to two standard deviations, which is equivalent to a 95% confidence interval around the mean 
    */
    statistics.confidence_interval = ( !conf_interval_bool && (std < max_pix_shift)) ? max_pix_shift : std*2;

    statistics.extended_search = true;
    return statistics;

}

void OMTA::cleanPotentialBuffer(){

    double timing_tolerance = loaded_params_->frame_tolerance/60;
    if(framerate_ != 0){
        timing_tolerance = (1/framerate_) * loaded_params_->frame_tolerance; 
    }

    std::scoped_lock lock(mutex_gen_sequences_);
    for(int i = 0; i < (int)gen_sequences_.size(); ++i){
        if(gen_sequences_[i]->empty()){
            continue;
        }


        int number_zeros_till_seq_deleted = (loaded_params_->max_zeros_consecutive + loaded_params_->allowed_BER_per_seq);
        if((int)gen_sequences_[i]->size() > number_zeros_till_seq_deleted){
            int cnt = 0;
            for (auto reverse_it = gen_sequences_[i]->rbegin(); 
                reverse_it != gen_sequences_[i]->rend(); ++reverse_it ) {  
                if(!(reverse_it->led_state)){
                    ++cnt;
                    if(cnt > number_zeros_till_seq_deleted){
                        gen_sequences_.erase(gen_sequences_.begin() + i);
                        break;
                    }
                }else{
                    cnt = 0;
                }
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
        if(id >= 0 ){ 
            // gc++;
            // std::cout << "gc " << gc << "\n";
        } 
        auto sequence_copy = sequence; 
        retrieved_signals.push_back(std::make_pair(sequence_copy, id));
    }
    if(debug_)std::cout << "}\n";
    
    return retrieved_signals;
}

OMTA::~OMTA() {
}