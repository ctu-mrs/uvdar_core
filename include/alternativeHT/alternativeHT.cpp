#include "alternativeHT.h"

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

void alternativeHT::processBuffer(const mrs_msgs::ImagePointsWithFloatStampedConstPtr ptsMsg) {
    
    std::vector<PointState> currentFrame;
    for ( auto pointWithTimeStamp : ptsMsg->points) {
        PointState p;
        p.point = cv::Point(pointWithTimeStamp.x, pointWithTimeStamp.y);
        p.ledState = true; // every existent point is "on"
        p.insertTime = ptsMsg->stamp;
        cv::Point2d leftUpperCorner, rightDownCorner;
        leftUpperCorner.x = p.point.x - boundingBox_x_Size_; 
        leftUpperCorner.y = p.point.y - boundingBox_y_Size_;
        if (p.bbLeftUp.x < 0) p.bbLeftUp.x = 0;
        if (p.bbLeftUp.y < 0) p.bbLeftUp.y = 0;
        p.bbLeftUp = leftUpperCorner;
        p.bbRightDown = rightDownCorner;
        currentFrame.push_back(p);
    }

    findClosestPixelAndInsert(currentFrame);
    inserVPIfNoNewPointArrived(currentFrame);
    cleanPotentialBuffer();  // TODO: NOT WORKING!!!!!
    
std::cout<< "======================================\n";
    for (const auto k : generatedSequences_){
        std::vector<bool> p;
        for (auto r : k){
            if(r.ledState) p.push_back(true);
            else p.push_back(false);
        }
        printVectorIfNotEmpty(p, "seq");
    }
}

void alternativeHT::findClosestPixelAndInsert(std::vector<PointState> & currentFrame) {   
    
    std::vector<std::shared_ptr<PointState>> noNN;
    for(auto & currPoint : currentFrame){
        std::scoped_lock lock(mutex_generatedSequences_);
        bool nearestNeighbor = false;
        for(auto & sequence : generatedSequences_){
            PointState lastInserted = sequence.end()[-1];
            cv::Point2d diff = computeXYDiff(currPoint.point, lastInserted.point);
            if(diff.x <= max_pixel_shift_x_ && diff.y <= max_pixel_shift_y_){
                nearestNeighbor = true;
                insertPointToSequence(sequence, currPoint);    
                break;
            }else{
                nearestNeighbor = false;
            }
        }
        if(nearestNeighbor == false){
            // std::cout << "curr point " << currPoint.point.x << "," << currPoint.point.y << "\n";
            noNN.push_back(std::make_shared<PointState>(currPoint));
// // here was before ther "stasrt new sequence"
//             std::vector<PointState> vect;
//             vect.push_back(currPoint);
//             generatedSequences_.push_back(vect);
        }
    }

    if((int)noNN.size() != 0){
        checkBoundingBoxIntersection(noNN);
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

 // for all sequences where no correspondence was found insert VP
void alternativeHT::inserVPIfNoNewPointArrived(std::vector<PointState> & currentFrame){
    std::scoped_lock lock(mutex_generatedSequences_);
    for(auto & sequence : generatedSequences_){

        if(currentFrame.size() != 0){
            ros::Time lastInsert = currentFrame[0].insertTime;
            if(lastInsert != sequence.end()[-1].insertTime){
                // ROS_ERROR("Should happen sometimes");
                auto p = sequence.end()[-1];
                p.ledState = false; 
                p.insertTime = ros::Time::now();
                insertPointToSequence(sequence, p);
            }
        }else{
            ros::Time now = ros::Time::now();
            if(now != sequence.end()[-1].insertTime){
                auto p = sequence.end()[-1];
                p.ledState = false; 
                p.insertTime = now;
                insertPointToSequence(sequence, p);
            }
        }
    }
}

void uvdar::alternativeHT::checkBoundingBoxIntersection(std::vector<std::shared_ptr<PointState>> noNNCurrentFrame){

    for(const auto point : noNNCurrentFrame){
        std::vector<std::shared_ptr<std::vector<PointState>>> ptsHit; 
        // std::vector<std::shared_ptr<PointState>> ptsHit;
        std::scoped_lock lock(mutex_generatedSequences_);
        for(const auto seq : generatedSequences_){
            if(checkForValidityWithNewInsertedPoint(seq, point)){
                // NOW check if boxes hit
                    // std::cout << "seq" << seq.end()[-1].point.x << ", " << seq.end()[-1].point.y << " -- " << point->point.x << ", " << point->point.y << "\n";
                if(bbIntersec(seq.end()[-1], *point)){
                    // ROS_ERROR("HIT");
                    ptsHit.push_back(std::make_shared<std::vector<PointState>>(seq));
                }
            }
        }

        if(ptsHit.size() == 0){
            // // start new sequence
            // std::cout << "new seq\n";
            std::vector<PointState> vect;
            vect.push_back(*point);
            generatedSequences_.push_back(vect);
            continue;
        }
        for (auto p : ptsHit){
            std::cout << p->end()[-1].point.x << "," << p->end()[-1].point.y << "\n";
        }
        // std::cout << "after continue\n";
        auto selectedSequence = findClosestWithinSelectedBB(ptsHit, *point);
        // std::cout << "The selected one " << point->point.x << "," << point->point.y << " the seq " << selectedSequence->end()[-1].point.x << "," <<  selectedSequence->end()[-1].point.y <<  "\n";
        insertPointToSequence(*selectedSequence, *point);
    }

}


bool uvdar::alternativeHT::checkForValidityWithNewInsertedPoint(const std::vector<PointState> & seq, const std::shared_ptr<PointState> currPoint){

    if(seq.size() <= 1){
        return true;
    }
    // if all LEDs are "on" Manchester Coding Property is violated
    if ((seq.end()[-1].ledState && seq.end()[-2].ledState && currPoint->ledState)){
        return false;
    }

    //if all LEDs are "off" Manchester Coding Property is violated
    if(!seq.end()[-1].ledState && !seq.end()[-2].ledState && !currPoint->ledState){
        return false;
    }

    // if the last two LEDs are "on" Manchester Coding Property would be violated if current "on" point will be inserted 
    if(seq.end()[-1].ledState && seq.end()[-2].ledState && !currPoint->ledState){
        ROS_ERROR("Shouldn't happen! VIRTUAL POINT SHOULDN'T BE HERE");
        return false; 
    }

    return true; 
}


bool uvdar::alternativeHT::bbIntersec(const PointState & prev, const PointState & curr){
    
    // if(curr.bbLeftUp.x > prev.bbRightDown.x || prev.bbLeftUp.x > curr.bbRightDown.x || 
    //     curr.bbRightDown.y > prev.bbLeftUp.y || prev.bbRightDown.y > curr.bbLeftUp.y){
    //     return false;
    // }
    // return true;
    auto diff = computeXYDiff(prev.point, curr.point);
    if(diff.x <= 20 && diff.y <= 20){
        return true;
    }

    return false; 
}


//TODO: Change to ICP
std::shared_ptr<std::vector<PointState>> uvdar::alternativeHT::findClosestWithinSelectedBB(std::vector<std::shared_ptr<std::vector<PointState>>> boxMatchPoints, const PointState queryPoint){

    cv::Point2d min;
    std::shared_ptr<std::vector<PointState>> selected; 
    min.x = -1;
    min.y = -1; 
    for (const auto p : boxMatchPoints){
        auto matchedPoint = p->end()[-1];
        cv::Point2d diff = computeXYDiff(matchedPoint.point, queryPoint.point);
        if( (diff.x < min.x && diff.y < min.y) || (min.x == -1 && min.y == -1) ){
            selected = p;
            min.x = diff.x; 
            min.y = diff.y;
        }
    }
    return selected;
}

void alternativeHT::cleanPotentialBuffer(){

//TODO: eventually not needed - correction in Moving Avg part
    double timeThreeFrames = (1/60.0) * 4; 

    std::scoped_lock lock(mutex_generatedSequences_);
    for (auto it = generatedSequences_.begin(); it != generatedSequences_.end(); ++it){ 
        if (it->empty()){
            continue;
        }

        // if(it->size() >= 3){
        // bool first = true;
        // bool second = true;
        // bool third = true;
        //     first   = it->end()[-1].ledState;
        //     second  = it->end()[-2].ledState;
        //     third   =  it->end()[-3].ledState;
        //     if((first && second && third) || (!first && !second && !third)){
        //     ROS_ERROR("MOTHERFUCKER");
        //     it = generatedSequences_.erase(it);
        //     continue;
        // }
        // }
        
        
        
        double insertTime = it->end()[-1].insertTime.toSec(); 
        double timeDiffLastInsert = std::abs(insertTime - ros::Time::now().toSec());
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