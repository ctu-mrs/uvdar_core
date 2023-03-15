#include "alternativeHT.h"

#include <bits/stdc++.h>

using namespace uvdar;

alternativeHT::alternativeHT(double decayFactor, int polyOrder){

    extended_search_ = std::make_unique<ExtendedSearch>(decayFactor, polyOrder);

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

void alternativeHT::processBuffer(const mrs_msgs::ImagePointsWithFloatStampedConstPtr ptsMsg) {
    
    std::vector<PointState> currentFrame;
    for ( auto pointWithTimeStamp : ptsMsg->points) {
        PointState p;
        p.point = cv::Point(pointWithTimeStamp.x, pointWithTimeStamp.y);
        p.ledState = true;
        p.insertTime = ptsMsg->stamp;
        currentFrame.push_back(p);
    }

    findClosestPixelAndInsert(currentFrame);

    cleanPotentialBuffer();  // TODO: NOT WORKING!!!!!
  
}

void alternativeHT::findClosestPixelAndInsert(std::vector<PointState> & currentFrame) {   
    
    // reference to sequences used for processing in the moving average functions
    std::vector<seqPointer> pGenSequence;
    {
       std::scoped_lock lock(mutex_generatedSequences_);
        for(auto & seq : generatedSequences_){
            pGenSequence.push_back(&seq);
        }
    }

    std::vector<PointState> noNN;
    for(auto & currPoint : currentFrame){
        std::scoped_lock lock(mutex_generatedSequences_);
        bool nearestNeighbor = false;
        for(auto seq = pGenSequence.begin(); seq != pGenSequence.end(); ++seq){
            PointState lastInserted = (*seq)->end()[-1];
            cv::Point2d diff = computeXYDiff(currPoint.point, lastInserted.point);
            if(diff.x <= max_pixel_shift_x_ && diff.y <= max_pixel_shift_y_){
                nearestNeighbor = true;
                insertPointToSequence(*(*seq), currPoint);    
                pGenSequence.erase(seq);
                break;
            }else{
                nearestNeighbor = false;
            }
        }
        if(nearestNeighbor == false){
            noNN.push_back(currPoint);
        }
    }
    expandedSearch(noNN, pGenSequence);

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


void alternativeHT::expandedSearch(std::vector<PointState> & noNNCurrentFrame, std::vector<seqPointer>& sequencesNoInsert){
    
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
    
    // std::cout << "Sequences No Insert" << sequencesNoInsert.size() << " Seq " << generatedSequences_.size() << "\n";
    if((int)noNNCurrentFrame.size() != 0){
        double insertTime = noNNCurrentFrame[0].insertTime.toSec() + predictionMargin_;
        // std::vector<SeqWithTrajectory> sequences;
        for(int k = 0; k < (int)sequencesNoInsert.size(); ++k){
            
            if(!checkSequenceValidityWithNewInsert(sequencesNoInsert[k])){
                continue;
            }
            

            std::vector<double> x,y;
            std::vector<ros::Time> time;

            // prepare for polynomial regression TODO: Maybe calculate the variance over the points for outlier rejection
            for(const auto point : *sequencesNoInsert[k]){
                if(point.ledState){
                    x.push_back(point.point.x);
                    y.push_back(point.point.y);
                    time.push_back(point.insertTime);
                }
            }

            if((int)x.size() < 5 || x.size() != y.size()){
                continue;
            }

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
            double std_x = sqrt(ss_x/(int)x.size());
            double std_y = sqrt(ss_y/(int)y.size());

            std::cout << "STD " << std_x << " " << std_y << " Sample size " << x.size() << std::endl;
            bool movement  = true;
            if(std_x < 1.0 && std_y < 1.0 && (int)x.size() < 10 && (int)y.size() < 10 ){
                movement = false;
            }

            auto lastPoint = &sequencesNoInsert[k]->end()[-1]; 
            
            if(movement){
                if(!extended_search_->doRegression(sequencesNoInsert[k], x, y, time)){
                    continue;
                }


                if((int)lastPoint->x_coeff.size() != (int)lastPoint->y_coeff.size()){
                    continue;
                }

                for(int k = 0; k < (int)lastPoint->x_coeff.size(); ++k){
                    lastPoint->predicted.x += lastPoint->x_coeff[k]*pow(insertTime, k);
                    lastPoint->predicted.y += lastPoint->y_coeff[k]*pow(insertTime, k);
                }
            }
            // else std::cout << "NO bool set\n";
            for(int i = 0; i < (int)noNNCurrentFrame.size(); ++i){
                if(movement){
                    if(extended_search_->checkIfInsideEllipse(*lastPoint, noNNCurrentFrame[i].point)){
                        noNNCurrentFrame[i].computedExtendedSearch = true;
                        noNNCurrentFrame[i].ellipse = lastPoint->ellipse;
                        noNNCurrentFrame[i].predicted = lastPoint->predicted;
                        noNNCurrentFrame[i].x_coeff = lastPoint->x_coeff;
                        noNNCurrentFrame[i].y_coeff = lastPoint->y_coeff;
                        insertPointToSequence(*(sequencesNoInsert[k]), noNNCurrentFrame[i]);
                        noNNCurrentFrame.erase(noNNCurrentFrame.begin()+i);
                        sequencesNoInsert.erase(sequencesNoInsert.begin()+k); 
                        // std::cout << "ELLIPSE\n";
                        break;
                    }
                }else{
                    cv::Point2d diff = computeXYDiff(sequencesNoInsert[k]->end()[-1].point, noNNCurrentFrame[i].point);
                    if(diff.x <= max_pixel_shift_x_*2 && diff.y <= max_pixel_shift_y_*2){
                        ROS_ERROR("hit box");                        
                        noNNCurrentFrame[i].computedExtendedSearch = true;
                        noNNCurrentFrame[i].ellipse = lastPoint->ellipse;
                        noNNCurrentFrame[i].predicted = lastPoint->predicted;
                        noNNCurrentFrame[i].x_coeff = lastPoint->x_coeff;
                        noNNCurrentFrame[i].y_coeff = lastPoint->y_coeff;
                        insertPointToSequence(*(sequencesNoInsert[k]), noNNCurrentFrame[i]);
                        noNNCurrentFrame.erase(noNNCurrentFrame.begin()+i);
                        sequencesNoInsert.erase(sequencesNoInsert.begin()+k);
                        break;
                    }
                }
            }
        }
    }


    // insert VP to sequnces were no point was inserted
    for(auto seq : sequencesNoInsert){
        insertVPforSequencesWithNoInsert(seq);
    }

    // if still points are not inserted start new sequence
    for(auto point : noNNCurrentFrame){
        std::vector<PointState> vect;
        vect.push_back(point);
        generatedSequences_.push_back(vect);
    }
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

std::vector<std::pair<PointState, int>> alternativeHT::getResults(){

    std::scoped_lock lock(mutex_generatedSequences_);
    std::vector<std::pair<PointState, int>> retrievedSignals;

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
        retrievedSignals.push_back(std::make_pair(sequence.end()[-1], id));
        if(sequence.end()[-1].computedExtendedSearch)std::cout << "Predicted " << sequence.end()[-1].predicted.x << " " << sequence.end()[-1].predicted.y << std::endl; 
       
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