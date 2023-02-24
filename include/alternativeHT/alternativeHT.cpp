#include "alternativeHT.h"

#include <bits/stdc++.h>

using namespace uvdar;

alternativeHT::alternativeHT(){
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
    std::cout << "-------------------------\n";
    for ( auto pointWithTimeStamp : ptsMsg->points) {
        PointState p;
        p.point = cv::Point(pointWithTimeStamp.x, pointWithTimeStamp.y);
        p.ledState = true; // every existent point is "on"
        p.insertTime = ptsMsg->stamp;
        // cv::Point2d leftUpperCorner, rightDownCorner;
        // leftUpperCorner.x = p.point.x - boundingBox_x_Size_; 
        // leftUpperCorner.y = p.point.y - boundingBox_y_Size_;
        // rightDownCorner.x = p.point.x + boundingBox_x_Size_;
        // rightDownCorner.y = p.point.y + boundingBox_y_Size_;
        // if (p.bbLeftUp.x < 0) p.bbLeftUp.x = 0;
        // if (p.bbLeftUp.y < 0) p.bbLeftUp.y = 0;
        // p.bbLeftUp = leftUpperCorner;
        // p.bbRightDown = rightDownCorner;
        currentFrame.push_back(p);
    }

    findClosestPixelAndInsert(currentFrame);
    // for (const auto k : generatedSequences_){
    //     std::vector<bool> p;
    //     for (auto r : k){
    //         if(r.ledState) p.push_back(true);
    //         else p.push_back(false);
    //     }
    //     printVectorIfNotEmpty(p, "before");
    // }
    insertVPIfNoNewPointArrived(currentFrame);
// std::cout << "---------------------------------------------\n";
    // cleanPotentialBuffer();  // TODO: NOT WORKING!!!!!
    // for (const auto k : generatedSequences_){
    //     std::vector<bool> p;
    //     for (auto r : k){
    //         if(r.ledState) p.push_back(true);
    //         else p.push_back(false);
    //     }
    //     // printVectorIfNotEmpty(p, "after");
    //     // for( auto p : k){
    //     //     std::cout << p.insertTime << ", ";
    //     // }
    //     // std::cout << std::endl;
    // }
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
//TODO: here was before the "start new sequence" stuff
            // std::vector<PointState> vect;
            // vect.push_back(currPoint);
            // generatedSequences_.push_back(vect);
        }
    }

// TODO: This is done within the insertVP - However maybe more usefull here?!
    // if((int)noNN.size() == 0 && pGenSequence.size() != 0){
    //     for(auto & seq : pGenSequence){
            
    //         insertPointToSequence(*seq, signal);
    //     }
    // }

    if((int)noNN.size() != 0 ){
        expandedSearch(noNN, pGenSequence);
    }
   
}

cv::Point2d uvdar::alternativeHT::computeXYDiff(const cv::Point2d first, const cv::Point2d second){
    
    cv::Point2d difference; 
    difference.x = std::abs(first.x - second.x);
    difference.y = std::abs(first.y - second.y);
    return difference;
}

void alternativeHT::insertPointToSequence(std::vector<PointState> & sequence, const PointState & signal){
        sequence.push_back(signal);            
        if(sequence.size() > originalSequences_[0].size()){
            sequence.erase(sequence.begin());
        }
}


void uvdar::alternativeHT::expandedSearch(std::vector<PointState> & noNNCurrentFrame, std::vector<seqPointer> & sequencesNoInsert){

    movAvgCheckLastTwoLEDStates(sequencesNoInsert);

    if(noNNCurrentFrame.size() == 0){
        return;
    }

    ros::Time insertTime = noNNCurrentFrame[0].insertTime; 
    std::vector<SeqWithTrajectory> sequencesWithRegression;
    for(auto seq : sequencesNoInsert){

        SeqWithTrajectory seqTrajectory;
        seqTrajectory.seq = seq;
        if(!HelpFunctions::prepareForPolyReg(seqTrajectory, 2)){
            continue;
        }
        
        calculatePredictionTriangle(seqTrajectory, insertTime);   
        sequencesWithRegression.push_back(seqTrajectory);
    }


    // if still points are not inserted start new sequence
    if(noNNCurrentFrame.size() != 0){
        for(auto point : noNNCurrentFrame){
            std::vector<PointState> vect;
            vect.push_back(point);
            generatedSequences_.push_back(vect);
        }
    }
}

void uvdar::alternativeHT::movAvgCheckLastTwoLEDStates(std::vector<seqPointer>& sequencesNoInsert){

    for(auto it = sequencesNoInsert.begin(); it != sequencesNoInsert.end(); ++it){
        if((*it)->size() >= 2){
            // if the last two led states were on - no new inserted point is expected and the sequence can be erased from the NoInsert vector
            if((*it)->end()[-1].ledState && (*it)->end()[-2].ledState){
                it = sequencesNoInsert.erase(it);
                // TODO: Insert here new Zero bit??
                continue; 
            }
        }
    }
}

void uvdar::alternativeHT::calculatePredictionTriangle(SeqWithTrajectory & path, const ros::Time insertTime){
// TODO: not tested yet
    int polynomSize = (int)path.xCoeff.size();
    double xPredict = 0, yPredict = 0;
    if(path.yCoeff.size() != path.xCoeff.size()){
        return;
    }
    for(int i = 0; i < (int)path.xCoeff.size(); ++i){
        xPredict += path.xCoeff[i]*pow(insertTime.toSec(), i); 
        yPredict += path.yCoeff[i]*pow(insertTime.toSec(), i);
    }

    cv::Point2d predictedPoint = cv::Point2d(xPredict, yPredict);
    std::cout << "predicted point " << predictedPoint.x  << "\t" << predictedPoint.y << "\n";

    cv::Point2d lastInserted = path.seq->end()[-1].point;
    cv::Point2d diffVect = predictedPoint - lastInserted;

    std::vector<cv::Point2d> orthoVects = HelpFunctions::findOrthogonalVectorWithLength(diffVect, 2);

    // construct triangle in coordinate center
    cv::Point2d firstEdgeCenter   = diffVect + orthoVects[0];
    cv::Point2d secEdgeCenter   = diffVect + orthoVects[1]; 

    //project to the last seen point
    cv::Point2d firstEdge = lastInserted + firstEdgeCenter;
    cv::Point2d secEdge = lastInserted + secEdgeCenter; 


    path.seq->end()[-1].firstEdgeTri    = firstEdge;
    path.seq->end()[-1].secEdgeTri      = secEdge;
    

}


//TODO: Too time sensitve
 // for all sequences where no correspondence was found insert VP
void alternativeHT::insertVPIfNoNewPointArrived(std::vector<PointState> & currentFrame){
    std::scoped_lock lock(mutex_generatedSequences_);
    for(auto & sequence : generatedSequences_){
        
        if(sequence.size() >= 2){
            bool lastLEDstate = sequence.end()[-1].ledState;
            bool secondLastLEDstate = sequence.end()[-2].ledState;
            if(!lastLEDstate && !secondLastLEDstate){
                continue;
            }
        }
        
        if(currentFrame.size() != 0){
            ros::Time lastInsert = currentFrame[0].insertTime;
            if(lastInsert != sequence.end()[-1].insertTime){
                auto p = sequence.end()[-1];
                p.ledState = false; 
                p.insertTime = ros::Time::now();
                insertPointToSequence(sequence, p);
            }
        }else{
            auto p = sequence.end()[-1];
            p.ledState = false; 
            p.insertTime = ros::Time::now();;
            insertPointToSequence(sequence, p);
        }
    }
}


void alternativeHT::cleanPotentialBuffer(){

//TODO: eventually not needed - correction in Moving Avg part
    double timeThreeFrames = (1/60.0) * 6; 

    std::scoped_lock lock(mutex_generatedSequences_);
    for (auto it = generatedSequences_.begin(); it != generatedSequences_.end(); ++it){ 
        if (it->empty()){
            continue;
        }

        if(it->size() >= 3){
            bool first  = it->end()[-1].ledState;
            bool second = it->end()[-2].ledState;
            bool third  = it->end()[-3].ledState;
            // if(first && second && third && forth){
            //     ROS_ERROR("all true");
            //     it = generatedSequences_.erase(it);
            //     continue;
            // }
            if(!first && !second && !third){
                ROS_ERROR("all false");
                it = generatedSequences_.erase(it);
                continue;
            }
        }
        
        
        
        double insertTime = it->end()[-1].insertTime.toSec(); 
        double timeDiffLastInsert = std::abs(insertTime - ros::Time::now().toSec());
        std::cout << "time diff " << timeDiffLastInsert << " last insedrt " << insertTime; 
        if (timeDiffLastInsert > timeThreeFrames){ //&& i != generatedSequences_.end()){
            ROS_ERROR("TIME");
            it = generatedSequences_.erase(it);
            continue;
        }

    }
}

std::vector<std::pair<PointState, int>> alternativeHT::getResults(){

    std::scoped_lock lock(mutex_generatedSequences_);
    std::vector<std::pair<PointState, int>> retrievedSignals;

    for (auto sequence : generatedSequences_){
        std::vector<bool> ledStates;
        for (auto point : sequence){
            ledStates.push_back(point.ledState);
        }
        // std::cout << "The seq" << std::endl; 
        // for(auto k : ledStates){
        //     if(k)std::cout << "1,";
        //     else std::cout << "0,";
        // }
        // std::cout << std::endl;


        int id = findSequenceMatch(ledStates);
        retrievedSignals.push_back(std::make_pair(sequence.end()[-1], id));
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