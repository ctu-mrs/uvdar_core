#include "alternativeHT.h"

using namespace uvdar;

alternativeHT::alternativeHT(double decayFactor, int poly_order){
    poly_order_ = poly_order;
    extended_search_ = std::make_unique<ExtendedSearch>(decayFactor, poly_order);

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
    
    std::vector<PointState> currentFrame;
    for ( auto pointWithTimeStamp : pts_msg->points) {
        PointState p;
        p.point = cv::Point(pointWithTimeStamp.x, pointWithTimeStamp.y);
        p.led_state = true;
        p.insert_time = pts_msg->stamp;
        currentFrame.push_back(p);
    }




// std::cout << "Berfore\n"; 

//     for (auto l : gen_sequences_){
//         for(auto k : l ){
//             if(k->led_state) {
//                 std::cout << "1,";
//             }else{
//                 std::cout << "0,";
//             }
//         }
//         std::cout << "\n";
//     } 

    findClosestPixelAndInsert(currentFrame);

    // currently crashing
    cleanPotentialBuffer();  // TODO: NOT WORKING!!!!!

    // std::cout << "after\n";

    // for (auto l : gen_sequences_){
    //     for(auto k : l ){
    //         if(k->led_state) {
    //             std::cout << "1,";
    //         }else{
    //             std::cout << "0,";
    //         }
    //     }
    //     std::cout << "\n";
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
        if(sequence.size() > (original_sequences_[0].size()*size_for_saved_seqs_)){
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
                // std::cout << "rejected\n";
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

            auto weight_vect = extended_search_->calculateWeightVector(time);
            auto x_weighted_mean = extended_search_->calcWeightedMean(x, weight_vect); 
            auto y_weighted_mean = extended_search_->calcWeightedMean(y, weight_vect); 
            auto x_std = extended_search_->calcWSTD(x, weight_vect, x_weighted_mean);
            auto y_std = extended_search_->calcWSTD(y, weight_vect, y_weighted_mean);

            
            bool x_all_coeff_zero = false;
            bool y_all_coeff_zero = false; 
            std::pair<std::vector<double>, Eigen::VectorXd> x_stats, y_stats;
            PointState& last_point = sequences_no_insert[k]->end()[-1]; 
            bool x_poly_reg_computed = false, y_poly_reg_computed = false;

            if(x_std > 2.0){
                x_stats = extended_search_->polyReg(x, time, weight_vect);
                x_all_coeff_zero = std::all_of(x_stats.first.begin(), x_stats.first.end(), [](double coeff){return coeff == 0;});
                if(!x_all_coeff_zero) x_poly_reg_computed = true;
                last_point.x_coeff = x_stats.first;
                auto x_prediction_vals_past = x_stats.second;
                // last_point.ellipse.x = (x_stats.second < 4) ? 4 : x_stats.second; 
                for(int i = 0; i < (int)last_point.x_coeff.size(); ++i){
                    last_point.predicted.x += last_point.x_coeff[i]*pow(insert_time, i);
                }
                auto x_conf_interval = extended_search_->standardErrorPrediction(x_prediction_vals_past, x, weight_vect, x_weighted_mean, last_point.predicted.x);
                last_point.ellipse.x = x_conf_interval;
                std::cout << "X CONF " << last_point.ellipse.x << "\n";

            }


            if(y_std > 2.0){
                y_stats = extended_search_->polyReg(y, time, weight_vect);
                y_all_coeff_zero = std::all_of(y_stats.first.begin(), y_stats.first.end(), [](double coeff){return coeff == 0;});
                if(!y_all_coeff_zero) y_poly_reg_computed = true;
                last_point.y_coeff = y_stats.first;
                auto y_prediction_vals_past = y_stats.second;
                // last_point.ellipse.y = y_stats.second;
                for(int i = 0; i < (int)last_point.y_coeff.size(); ++i){
                    last_point.predicted.y += last_point.y_coeff[i]*pow(insert_time, i);
                }
                
                auto y_conf_interval = extended_search_->standardErrorPrediction(y_prediction_vals_past, y, weight_vect, y_weighted_mean, last_point.predicted.y);
                last_point.ellipse.y = y_conf_interval;
                std::cout << "Y CONF " << last_point.ellipse.y << "\n";
            }
            
            if(x_poly_reg_computed){
                last_point.predicted.x = x_weighted_mean; // TODO: Bullshit???
                last_point.ellipse.x = (x_std < 1.0) ? max_pixel_shift_x_*2 : x_std*2;
                last_point.ellipse.x = (last_point.ellipse.x < 4) ? 4 : last_point.ellipse.x; 
                last_point.x_coeff = std::vector<double>(1, 0.0);
            }
            
            if(y_poly_reg_computed){
                last_point.predicted.y = y_weighted_mean; //TODO: Bullshit ?!
                last_point.ellipse.y = (y_std < 1.0) ? max_pixel_shift_y_*2 : y_std*2;
                last_point.ellipse.y = (last_point.ellipse.y < 4) ? 4 : last_point.ellipse.y; 
                last_point.y_coeff = std::vector<double>(1, 0.0);
            }
            last_point.computed_extended_search = true;

            for(int i = 0; i < (int)no_nn_current_frame.size(); ++i){
                if(extended_search_->checkIfInsideEllipse(last_point.point, last_point.ellipse, no_nn_current_frame[i].point)){
                    no_nn_current_frame[i].computed_extended_search = true;
                    no_nn_current_frame[i].x_coeff = last_point.x_coeff;
                    no_nn_current_frame[i].y_coeff = last_point.y_coeff;
                    no_nn_current_frame[i].ellipse = last_point.ellipse;
                    no_nn_current_frame[i].predicted = last_point.predicted;
                    insertPointToSequence(*sequences_no_insert[k], no_nn_current_frame[i]);
                    no_nn_current_frame.erase(no_nn_current_frame.begin()+i);
                    sequences_no_insert.erase(sequences_no_insert.begin()+k);
                    continue;
                }
            }
        }
    }
    // std::cout << "+++++++++++++++++++++++++++++++++++++++\n"; 

    // insert VP to sequnces were no point was inserted
    for(auto seq : sequences_no_insert){
        insertVPforSequencesWithNoInsert(seq);
    }

    // if still points are not inserted start new sequence
    for(auto point : no_nn_current_frame){
        // std::cout <<  no_nn_current_frame.size() << " - START NEW SEQ " << point.point.x << " " << point.point.y << "\n"  ;
        std::vector<PointState> vect;
        vect.push_back(point);
        gen_sequences_.push_back(std::make_shared<std::vector<PointState>>(vect));
    }
    // std::cout << "==================\n";
}

bool alternativeHT::checkSequenceValidityWithNewInsert(const seqPointer & seq){

    if(seq->size() > 1){
        // if the last two led states were on - no "on" point expected -> no NN search necessary
       if(seq->end()[-1].led_state == true && seq->end()[-2].led_state == true){
            return false;
        }
    }
    return true; 
}

void alternativeHT::insertVPforSequencesWithNoInsert(seqPointer & seq){
    
    PointState pVirtual;
    // select eventually predicted point and not last inserted one 
    pVirtual = seq->end()[-1];
    pVirtual.insert_time = ros::Time::now();
    pVirtual.led_state = false;
    insertPointToSequence(*seq, pVirtual);
}

void alternativeHT::cleanPotentialBuffer(){

    std::scoped_lock lock(mutex_gen_sequences_);
    double timeMargin = 5/60;
    if(framerate_ != 0){
        timeMargin = (1/framerate_) * 60; 
    }

    // std::cout << "Time margin " << timeMargin; 

    for (auto it = gen_sequences_.begin(); it != gen_sequences_.end();){ 
        if ((*it)->empty()){
            continue;
        }

        if( (*it)->size() > 2){
            if(!(*it)->end()[-1].led_state && !(*it)->end()[-2].led_state && !(*it)->end()[-3].led_state){
                // for(auto l : *it){
                //     if(l.led_state) std::cout << "1,";
                //     else std::cout << "0,";
                // }std::cout << std::endl;
                gen_sequences_.erase(it);
                continue;
            }
        }
        ++it;

        // TODO: INSERT!!!
        // double insert_time = it->end()[-1].insert_time.toSec(); 
        // double timeDiffLastInsert = std::abs(insert_time - ros::Time::now().toSec());
        // if (timeDiffLastInsert > timeMargin){ //&& i != generatedSequences_.end()){
        //     it = generatedSequences_.erase(it);
        //     std::cout << "TIME\n";
        //     continue;
        // }

    }
}

std::vector<std::pair<std::vector<PointState>, int>> alternativeHT::getResults(){

    std::scoped_lock lock(mutex_gen_sequences_);
    std::vector<std::pair<std::vector<PointState>, int>> retrievedSignals;

    for (auto sequence : gen_sequences_){
        std::vector<bool> led_states;
        std::vector<PointState> return_seq; 
        for(auto p : *sequence){
            return_seq.push_back(p);
        }

        std::vector<PointState> selected;
        if ((int)return_seq.size() > (int)original_sequences_[0].size()){
            int diff = (int)return_seq.size() - (int)original_sequences_[0].size();
            // std::cout << "the diff" << diff << "\n";

            for(int i = diff; i < (int)return_seq.size(); ++i){
                selected.push_back(return_seq[i]);
            }
        }else{

            selected = return_seq;
        }
        // std::cout << "size " << selected.size() << "\n";
        for (auto point : selected){
            led_states.push_back(point.led_state);
        }

        int id = findSequenceMatch(led_states);



        // retrievedSignals.push_back(std::make_pair(sequence.end()[-1], id));
        retrievedSignals.push_back(std::make_pair(return_seq, id));
        // if(sequence.end()[-1].computed_extended_search)std::cout << "Predicted " << sequence.end()[-1].predicted.x << " " << sequence.end()[-1].predicted.y << std::endl; 
       
    }

    return retrievedSignals;
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