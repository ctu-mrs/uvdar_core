#include "alternativeHT.h"

using namespace uvdar;

alternativeHT::alternativeHT( int i_buffer_size ){
    
    buffer_size_ = i_buffer_size;
}

void alternativeHT::initBuffer(){
    std::scoped_lock lock(mutex_buffer);
    for (int i = 0; i < buffer_size_; i++){
        std::vector<PointState> dummyVector;
        buffer.push_back(dummyVector);
    }
}

void alternativeHT::setDebugFlags(bool debug, bool visual_debug){
    debug_ = debug;
    visual_debug_ = visual_debug;
}

void alternativeHT::setSequences(std::vector<std::vector<bool>> i_sequences){
  
    originalSequences_ = i_sequences;
    matcher_ = std::make_unique<SignalMatcher>(originalSequences_);
    initBuffer();
    
}

void alternativeHT::processBuffer(const mrs_msgs::ImagePointsWithFloatStampedConstPtr ptsMsg) {

    {
        std::scoped_lock lock(mutex_buffer);
        std::vector<PointState> points;
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
            points.push_back(p);

        }

        if((int)buffer.size() < buffer_size_){
            buffer.push_back(points);
        }else{
            buffer.erase(buffer.begin());
            buffer.push_back(points);
        }

        if((int)buffer.size() < 2){
            return;
        }
    }

    findClosestAndLEDState();
    cleanPotentialBuffer(); 
    // checkIfThreeConsecutiveZeros(); // TODO: eventually buffer not preventing from overflow
}


void alternativeHT::findClosestAndLEDState() {   
    
    std::scoped_lock lock(mutex_buffer);
    
    std::vector<PointState> & currPts = buffer.end()[-1]; // newest points;
    std::vector<PointState> & prevPts = buffer.end()[-2]; // second newest points

    std::vector<std::shared_ptr<PointState>> currPtsPointer;
    std::vector<std::shared_ptr<PointState>> noNNFoundPrev; 
    
    for (auto & k : currPts){
        std::shared_ptr<PointState> p = std::make_shared<PointState>(k);
        currPtsPointer.push_back(p);
    }
    
    
    std::cout << "POINTER " <<  currPtsPointer.size()<< std::endl; 
    for(auto & prevPoint : prevPts){
        bool nearestNeighbor = false;
        for(int i = 0; i<(int)currPtsPointer.size(); ++i){
            cv::Point2d diff = computeXYDiff(currPtsPointer[i]->point, prevPoint.point);
            if(diff.x <= max_pixel_shift_x_ && diff.y <= max_pixel_shift_y_){
                nearestNeighbor = true;
                if (!prevPoint.insertedToSeq){
                    insertPointToSequence(prevPoint, prevPoint);
                }
                insertPointToSequence(*currPtsPointer[i], prevPoint);
                std::cout << "match\n";
                currPtsPointer.erase(currPtsPointer.begin()+i);
                break;
            }else{
                nearestNeighbor = false;
            }
        }
        if(nearestNeighbor == false){
            noNNFoundPrev.push_back(std::make_shared<PointState>(prevPoint));
        }
    }

    // for each point in current frame a correspondence is found - however points exist in previous frame where no correspondence was found -> insert VP in current frame
    if(noNNFoundPrev.size() != 0 && currPtsPointer.size() == 0){
        for(auto k : noNNFoundPrev){
            if(!k->insertedToSeq){
                insertPointToSequence(*k, *k);
            }
            insertVirtualPointToCurrentFrame(*k);
            insertPointToSequence(currPts.end()[-1], *k);
        }
    }

    // search for nearest neighbors within the bounding boxes for all not assigned points 
    if(noNNFoundPrev.size() != 0 && currPtsPointer.size() != 0){
        std::cout << "ERROR?!\n";  
        // checkBoundingBoxIntersection(noNNFoundPrev, currPtsPointer);
    }
}

cv::Point2d uvdar::alternativeHT::computeXYDiff(const cv::Point2d first, const cv::Point2d second){
    
    cv::Point2d difference; 
    difference.x = std::abs(first.x - second.x);
    difference.y = std::abs(first.y - second.y);
    return difference;
}

void alternativeHT::insertVirtualPointToCurrentFrame(const PointState signalPrevFrame){   
    PointState offState;
    offState.insertedToSeq = false;
    offState.point = signalPrevFrame.point;
    offState.ledState = false;
    offState.insertTime = ros::Time::now();
    buffer.end()[-1].push_back(offState);
}

// return val for sucess?
void alternativeHT::insertPointToSequence(PointState & signal, PointState prev){
    std::scoped_lock lock(mutex_generatedSequences_);
    
    // start new sequence - if the previous element wasn't inserted: signal = prev 
    if(generatedSequences_.size() == 0 || !prev.insertedToSeq){
        signal.insertedToSeq = true;
        std::vector<PointState> vect;
        vect.push_back(signal);
        generatedSequences_.push_back(vect);
        return;
    }

// find sequence where signal should be inserted
    if (prev.insertedToSeq){
        for( auto & sequence : generatedSequences_){
            if(equalPoints(prev, sequence.end()[-1])){
                signal.insertedToSeq = true;
                sequence.push_back(signal);
                if(sequence.size() > originalSequences_[0].size()){
                    sequence.erase(sequence.begin());
                }
                return;
            }
        }
    }
    
}

bool alternativeHT::equalPoints(const PointState & p1, const PointState & p2){

    if (p1.point.x == p2.point.x && p1.point.y == p2.point.y && p1.ledState == p2.ledState && p1.insertTime.toSec() == p2.insertTime.toSec()) {
        return true;
    }
    return false; 

}

void uvdar::alternativeHT::checkBoundingBoxIntersection(std::vector<std::shared_ptr<PointState>> noNNPrev, std::vector<std::shared_ptr<PointState>> notAssignedCurr){

    std::vector<std::pair<std::shared_ptr<PointState>,std::vector<std::shared_ptr<PointState>>>> correspondences; 
    for(const auto prev : noNNPrev){
        std::vector<std::shared_ptr<PointState>> ptsHit;
        if(prev->insertedToSeq){
            std::scoped_lock lock(mutex_generatedSequences_);
            for(const auto seq : generatedSequences_){
                if(equalPoints(*prev, seq.end()[-1])){
                    for(const auto curr : notAssignedCurr){
                        // if(checkForValidityWithNewInsertedPoint(seq, curr)){
                            // NOW check if boxes hit
                            // ptsHit.push_back(curr);
                        // }
                    
                    }
                }
            }
        }
        // else{
                
            // }
        // }


        // for( const auto curr : notAssignedCurr){

        //     if(bbIntersec(prev, curr)){
        //         ptsHit.push_back(curr);
        //     };
        // }
        // if((int)ptsHit.size() != 0){
        //     correspondences.push_back(std::make_pair(prev, ptsHit));
        // }
    }

}

bool uvdar::alternativeHT::bbIntersec(const std::shared_ptr<PointState> prev, const std::shared_ptr<PointState> curr){
    
    if(curr->bbLeftUp.x > prev->bbRightDown.x || prev->bbLeftUp.x > curr->bbRightDown.x || 
        curr->bbRightDown.y > prev->bbLeftUp.y || prev->bbRightDown.y > curr->bbLeftUp.y){
        return false;
    }
    return true;
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
//TODO: Change to ICP
PointState uvdar::alternativeHT::findClosest(const std::vector<PointState> boxMatchPoints, const PointState point){
    cv::Point2d min;
    PointState selected; 
    min.x = -1;
    min.y = -1; 
    for (const auto p : boxMatchPoints){
        cv::Point2d diff = computeXYDiff(p.point, point.point);
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

        std::vector<bool> p;

        for (const auto k : generatedSequences_){
            for(const auto l : k)
            p.push_back(l.ledState);
        }

        // printVectorIfNotEmpty(p, "seq");

        bool first = true;
        bool second = true;
        bool third = true;
        if(it->size() >= 3){
            first   = it->end()[-1].ledState;
            second  = it->end()[-2].ledState;
            third   =  it->end()[-3].ledState;
        }
        if (first == false && second == false && third == false){
            // std::cout << "THREE zero\n"; 
            it = generatedSequences_.erase(it);
            continue;
        }
        
        
        
        double insertTime = it->end()[-1].insertTime.toSec(); 
        double timeDiffLastInsert = std::abs(insertTime - ros::Time::now().toSec());
        if (timeDiffLastInsert > timeThreeFrames){ //&& i != generatedSequences_.end()){
            // std::cout << "TIME\n";
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



/*****************************************************************************/
// stuff for moving average 


void alternativeHT::checkIfThreeConsecutiveZeros(){
// TODO: check if necessary?!
    
    if (first_call_){
        return;
    }
    std::scoped_lock lock(mutex_buffer);
    std::vector<PointState> & ptsCurr= buffer.end()[-1];
    std::vector<PointState> & ptsSec = buffer.end()[-2];
    std::vector<PointState> & ptsThird = buffer.end()[-3];

    for (int i = 0; i < (int)ptsCurr.size(); i++){
        bool insertedCurr = (ptsCurr[i].insertedToSeq) ? true : false;
        bool boolCurr = ptsCurr[i].ledState; 
        for(int k = 0; k < (int)ptsSec.size(); k++){
            bool insertedSec = (ptsSec[i].insertedToSeq) ? true : false;
            bool boolSec = ptsSec[k].ledState;
            for (int j =0; j<(int)ptsThird.size(); j++){
                bool insertedThird = (ptsThird[i].insertedToSeq) ? true : false;
                bool boolThird = ptsThird[j].ledState; 
                if (insertedCurr && insertedSec && ptsThird[i].insertedToSeq){
                    if (boolThird == false && boolSec == false && boolCurr == false){
                        ROS_ERROR("At least here?!");
                        ptsCurr.erase(ptsCurr.begin()+i);
                        ptsThird.erase(ptsThird.begin()+j);
                        ptsSec.erase(ptsSec.begin()+k);
                    }
                }
            }

        }
    }
}