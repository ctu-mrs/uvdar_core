#include "alternativeHT.h"

#include <bits/stdc++.h>

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
  
    originalSequences_ = i_sequences;
    matcher_ = std::make_unique<SignalMatcher>(originalSequences_);
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
        p.ledState = true;
        p.insertTime = pts_msg->stamp;
        currentFrame.push_back(p);
    }

    findClosestPixelAndInsert(currentFrame);

    cleanPotentialBuffer();  // TODO: NOT WORKING!!!!!
  
}

void alternativeHT::findClosestPixelAndInsert(std::vector<PointState> & current_frame) {   
    
    // reference to sequences used for processing in the moving average functions
    std::vector<seqPointer> p_gen_seq;
    {
       std::scoped_lock lock(mutex_generatedSequences_);
        for(auto & seq : generatedSequences_){
            p_gen_seq.push_back(&seq);
        }
    }

    std::vector<PointState> no_nn;
    for(auto & currPoint : current_frame){
        std::scoped_lock lock(mutex_generatedSequences_);
        bool nn = false;
        for(auto seq = p_gen_seq.begin(); seq != p_gen_seq.end(); ++seq){
            PointState lastInserted = (*seq)->end()[-1];
            cv::Point2d diff = computeXYDiff(currPoint.point, lastInserted.point);
            if(diff.x <= max_pixel_shift_x_ && diff.y <= max_pixel_shift_y_){
                nn = true;
                insertPointToSequence(*(*seq), currPoint);    
                p_gen_seq.erase(seq);
                break;
            }else{
                nn = false;
            }
        }
        if(nn == false){
            no_nn.push_back(currPoint);
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
        if(sequence.size() > (originalSequences_[0].size()*size_for_savedSequences_)){
            sequence.erase(sequence.begin());
        }
}


void alternativeHT::expandedSearch(std::vector<PointState> & no_nn_current_frame, std::vector<seqPointer>& sequences_no_insert){
    
    std::scoped_lock lock(mutex_generatedSequences_);

    // std::cout << "Before\n";
    // for(auto k : generatedSequences_){
    //     auto last       = k.end()[-1].ledState;
    //     auto lastsecond= k.end()[-2].ledState;
    //     auto lastThree= k.end()[-3].ledState;
    //     if(lastThree)std::cout << "1,";
    //     else std::cout << "0,";
    //     if(lastsecond)std::cout << "1,";
    //     else std::cout << "0,";
    //     if(last)std::cout << "1,";
    //     else std::cout << "0,";;
    //     std::cout << "\n";
    // }
    
    // std::cout << "Sequences No Insert" << sequences_no_insert.size() << " Seq " << generatedSequences_.size() << "\n";
    if((int)no_nn_current_frame.size() != 0){
        double insertTime = no_nn_current_frame[0].insertTime.toSec() + predictionMargin_;
        // std::vector<SeqWithTrajectory> sequences;
        for(int k = 0; k < (int)sequences_no_insert.size(); ++k){
            
            if(!checkSequenceValidityWithNewInsert(sequences_no_insert[k])){
                std::cout << "rejected\n";
                continue;
            }
            

            std::vector<double> x,y;
            std::vector<ros::Time> time;

            // prepare for polynomial regression TODO: Maybe calculate the variance over the points for outlier rejection
            for(const auto point : *sequences_no_insert[k]){
                if(point.ledState){
                    x.push_back(point.point.x);
                    y.push_back(point.point.y);
                    time.push_back(point.insertTime);
                }
            }

            // if((int)x.size() < 5 || x.size() != y.size()){
            //     std::cout << "TO SMALL"; 
            //     continue;
            // }
            // std::cout << "Size " << x.size()  << "\n "; 

            double sum_x = std::accumulate(x.begin(), x.end(), 0.0);
            double sum_y = std::accumulate(y.begin(), y.end(), 0.0);

            double mean_x = sum_x/(int)x.size();
            double mean_y = sum_y/(int)y.size();

            double ss_x = 0;
            double ss_y = 0; 
            for(int i = 0; i < (int)x.size(); ++i){
                ss_x += pow(x[i] - mean_x, 2);
                ss_y += pow(y[i] - mean_y, 2);
            }
            // eventually n-1
            double std_x = sqrt(ss_x/(int)x.size());
            double std_y = sqrt(ss_y/(int)y.size());

            bool x_all_coeff_zero = false;
            bool y_all_coeff_zero = false; 
            std::pair<std::vector<double>, double> x_stats, y_stats;
            auto last_point = &sequences_no_insert[k]->end()[-1]; 
            bool poly_reg_y = false;
            bool poly_reg_x = false;
            if(std_x > 2.0){
                poly_reg_x = true;
                x_stats = extended_search_->polyReg(x, time);
                x_all_coeff_zero = std::all_of(x_stats.first.begin(), x_stats.first.end(), [](double coeff){return coeff == 0;});
                last_point->x_coeff = x_stats.first;
                last_point->ellipse.x = (x_stats.second < 4) ? 4 : x_stats.second; 

                for(int i = 0; i < last_point->x_coeff.size(); ++i){
                    last_point->predicted.x += last_point->x_coeff[i]*pow(insertTime, i);
                }
            }else{
                // std::cout << "MEAN X\n";
                last_point->predicted.x = mean_x;
                last_point->ellipse.x = (std_x < 1.0) ? max_pixel_shift_x_*2 : std_x*2;
                last_point->ellipse.x = (last_point->ellipse.x < 4) ? 4 : last_point->ellipse.x; 
                last_point->x_coeff = std::vector<double>(1, 0.0);
            }

            if(std_y > 2.0){
                poly_reg_y = true;
                y_stats = extended_search_->polyReg(y, time);
                y_all_coeff_zero = std::all_of(y_stats.first.begin(), y_stats.first.end(), [](double coeff){return coeff == 0;});
                last_point->y_coeff = y_stats.first;
                last_point->ellipse.y = y_stats.second;
                for(int i = 0; i < last_point->y_coeff.size(); ++i){
                    last_point->predicted.y += last_point->y_coeff[i]*pow(insertTime, i);
                }
                last_point->ellipse.y = (y_stats.second < 4) ? 4 : y_stats.second; 
            }else{
                // std::cout << "MEAN Y\n";
                last_point->predicted.y = mean_y;
                last_point->ellipse.y = (std_y < 1.0) ? max_pixel_shift_y_*2 : std_y*2;
                last_point->ellipse.y = (last_point->ellipse.y < 4) ? 4 : last_point->ellipse.y; 
                last_point->y_coeff = std::vector<double>(1, 0.0);
            }
            std::cout << "PREDICTION "<<  last_point->predicted.x << " " << last_point->predicted.y << " ELLI " <<  last_point->ellipse.x << " " << last_point->ellipse.y << "\n"; 

            // std::cout << "X P \n";
            // for(auto k : last_point->x_coeff){
            //     std::cout << k << ", ";
            // }std::cout << "\n";
            // std::cout << "Y P " << std::endl;
            // for(auto k : last_point->y_coeff){
            //     std::cout << k << ", ";
            // }std::cout << "\n";
            last_point->computedExtendedSearch = true;

            // else std::cout << "NO bool set\n";

            for(int i = 0; i < (int)no_nn_current_frame.size(); ++i){
                std::cout << no_nn_current_frame[i].point.x << " " << no_nn_current_frame[i].point.y <<", ";
                if(extended_search_->checkIfInsideEllipse(*last_point, no_nn_current_frame[i].point)){
                    no_nn_current_frame[i].computedExtendedSearch = true;
                    no_nn_current_frame[i].x_coeff = last_point->x_coeff;
                    no_nn_current_frame[i].y_coeff = last_point->y_coeff;
                    no_nn_current_frame[i].ellipse = last_point->ellipse;
                    no_nn_current_frame[i].predicted = last_point->predicted;
                    if(!no_nn_current_frame[i].ledState)ROS_ERROR("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWwwww"); 

                    insertPointToSequence(*(sequences_no_insert[k]), no_nn_current_frame[i]);
                    no_nn_current_frame.erase(no_nn_current_frame.begin()+i);
                    sequences_no_insert.erase(sequences_no_insert.begin()+k);
                    std::cout << " INSERT " << no_nn_current_frame.size() << " SEQ SIZE " << sequences_no_insert.size();  
                    continue;
                }
            }
            std::cout <<"\n";
        }
    }
    // std::cout << "+++++++++++++++++++++++++++++++++++++++\n"; 

    // insert VP to sequnces were no point was inserted
    for(auto seq : sequences_no_insert){
        insertVPforSequencesWithNoInsert(seq);
    }

    // if still points are not inserted start new sequence
    for(auto point : no_nn_current_frame){
        std::cout <<  no_nn_current_frame.size() << " - START NEW SEQ " << point.point.x << " " << point.point.y << "\n"  ;
        std::vector<PointState> vect;
        vect.push_back(point);
        generatedSequences_.push_back(vect);
    }
    std::cout << "==================\n";
}

bool alternativeHT::checkSequenceValidityWithNewInsert(const seqPointer & seq){

    if(seq->size() > 1){
        // if the last two led states were on - no "on" point expected -> no NN search necessary
       if(seq->end()[-1].ledState == true && seq->end()[-2].ledState == true){
            return false;
        }
    }
    return true; 
}

void alternativeHT::insertVPforSequencesWithNoInsert(seqPointer & seq){
    
    PointState pVirtual;
    // select eventually predicted point and not last inserted one 
    pVirtual = seq->end()[-1];
    pVirtual.insertTime = ros::Time::now();
    pVirtual.ledState = false;
    insertPointToSequence(*seq, pVirtual);
}

void alternativeHT::cleanPotentialBuffer(){

    std::scoped_lock lock(mutex_generatedSequences_);
    double timeMargin = 5/60;
    if(framerate_ != 0){
        timeMargin = (1/framerate_) * 60; 
    }
    // std::cout << "Time margin " << timeMargin; 

    for (auto it = generatedSequences_.begin(); it != generatedSequences_.end(); ++it){ 
        if (it->empty()){
            continue;
        }

        if( it->size() >= 2){
            if(!it->end()[-1].ledState && !it->end()[-2].ledState && !it->end()[-3].ledState){
                for(auto l : *it){
                    if(l.ledState) std::cout << "1,";
                    else std::cout << "0,";
                }std::cout << std::endl;
                it = generatedSequences_.erase(it);
                continue;
            }
        }
        // double insertTime = it->end()[-1].insertTime.toSec(); 
        // double timeDiffLastInsert = std::abs(insertTime - ros::Time::now().toSec());
        // if (timeDiffLastInsert > timeMargin){ //&& i != generatedSequences_.end()){
        //     it = generatedSequences_.erase(it);
        //     std::cout << "TIME\n";
        //     continue;
        // }

    }
}

std::vector<std::pair<std::vector<PointState>, int>> alternativeHT::getResults(){

    std::scoped_lock lock(mutex_generatedSequences_);
    std::vector<std::pair<std::vector<PointState>, int>> retrievedSignals;

    for (auto sequence : generatedSequences_){
        std::vector<bool> ledStates;

        std::vector<PointState> selected;
        if ((int)sequence.size() > (int)originalSequences_[0].size()){
            int diff = (int)sequence.size() - (int)originalSequences_[0].size();
            // std::cout << "the diff" << diff << "\n";

            for(int i = diff; i < (int)sequence.size(); ++i){
                selected.push_back(sequence[i]);
            }
        }else{
            selected = sequence;
        }
        // std::cout << "size " << selected.size() << "\n";
        for (auto point : selected){
            ledStates.push_back(point.ledState);
        }

        int id = findSequenceMatch(ledStates);

        // retrievedSignals.push_back(std::make_pair(sequence.end()[-1], id));
        retrievedSignals.push_back(std::make_pair(sequence, id));
        // if(sequence.end()[-1].computedExtendedSearch)std::cout << "Predicted " << sequence.end()[-1].predicted.x << " " << sequence.end()[-1].predicted.y << std::endl; 
       
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