#include "alternativeHT.h"

using namespace uvdar;

alternativeHT::alternativeHT( int i_buffer_size ){
    
    buffer_size_ = i_buffer_size;
    initBuffer();
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
    
}

void alternativeHT::processBuffer(const mrs_msgs::ImagePointsWithFloatStampedConstPtr ptsMsg) {
    {   
        std::scoped_lock lock(mutex_buffer);
        std::vector<mrs_msgs::Point2DWithFloat> point3D(std::begin(ptsMsg->points), std::end(ptsMsg->points));
        std::vector<PointState> points; 
        for ( size_t i = 0; i < point3D.size(); i++ ) {
            PointState p;
            p.point = cv::Point(point3D[i].x, point3D[i].y);
            p.ledState = true; // every existent point is "on"
            p.insertTime = ptsMsg->stamp;
            p.iterator = generatedSequences_.end();
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
    // cleanPotentialBuffer(); // TODO: throws error
    checkIfThreeConsecutiveZeros();
    // std::cout << "The list size is : " << generatedSequences_.size() << "\n";
}

void alternativeHT::findClosestAndLEDState() {   
    
    std::scoped_lock lock(mutex_buffer);
    
    std::vector<PointState> & ptsCurrentImg = buffer.end()[-1]; // newest points;
    std::vector<PointState> & ptsPrevImg    = buffer.end()[-2]; // second newest points
    // std::cout << "=============================" <<std::endl; 
    for(int iPrev = 0;  iPrev < (int)ptsPrevImg.size(); iPrev++){
        bool nearestNeighbor = false;
        for(int iCurr = 0; iCurr < (int)ptsCurrentImg.size(); iCurr++){
            cv::Point2d diff = computeXYDiff(ptsCurrentImg[iCurr].point, ptsPrevImg[iPrev].point);
            if(diff.x <= max_pixel_shift_x_ && diff.y <= max_pixel_shift_y_){
                std::scoped_lock lock(mutex_generatedSequences_);
                nearestNeighbor = true;
                if (ptsPrevImg[iPrev].iterator == generatedSequences_.end()){
                    insertPointToSequence(ptsPrevImg[iPrev], generatedSequences_.end());
                }else{
                    insertPointToSequence(ptsCurrentImg[iPrev], ptsPrevImg[iPrev].iterator);
                    // insert to iterator at ptsPrevImg
                }
                
                break;
            }else{
                // std::cout << "The point vals are " << diff.x << ", " << diff.y << std::endl; 
                nearestNeighbor = false;
            }
        }
        if(nearestNeighbor == false){
            std::scoped_lock lock(mutex_generatedSequences_);
            if (ptsPrevImg[iPrev].iterator == generatedSequences_.end()){
                insertPointToSequence(ptsPrevImg[iPrev], generatedSequences_.end());
            }
            insertVirtualPoint(ptsPrevImg[iPrev]);
            insertPointToSequence(ptsCurrentImg.end()[-1], ptsPrevImg[iPrev].iterator);

        }
    }
}

void alternativeHT::insertPointToSequence(PointState & signal, std::list<std::vector<PointState>>::iterator it){

    signal.iterator = it; 
    
    // double timeThreeFrames = (1/60.0) * 4; 
    // double insertTime = signal.insertTime.toSec();
    // double timeDiffLastInsert = std::abs(insertTime - ros::Time::now().toSec());
    // if (timeDiffLastInsert > timeThreeFrames){
    //     return;
    // }

    if(it == generatedSequences_.end()){
        std::cout << "Insert new Seq" << generatedSequences_.size() << "\n";
        std::vector<PointState> p; 
        p.push_back(signal);
        std::list<std::vector<PointState>>::iterator k = generatedSequences_.end();
        std::advance(signal.iterator, generatedSequences_.size());
        generatedSequences_.push_back(p);
        return;
    }

    // start deleting first element of sequence vector if it is equivalent to the wanted size 
    if (it->size() == originalSequences_[0].size()){
        it->erase(it->begin());
    }
  
    if(it->empty()){
        ROS_ERROR("WHy");
    }

    it->push_back(signal);
    generatedSequences_.insert(it,*it);
    std::cout << "HERE " <<  generatedSequences_.size() << std::endl;

    //     std::cout << "EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEERROR Iterator exists but vector is empty????" << std::endl;
    //     std::vector<PointState> p; 
    //     p.push_back(signal);
    //     signal.iterator = generatedSequences_.insert(it, p);
    //     return;
    // }
    // generatedSequences_.insert(it,*it);
}

cv::Point2d uvdar::alternativeHT::computeXYDiff(const cv::Point2d first, const cv::Point2d second){
    
    cv::Point2d difference; 
    difference.x = std::abs(first.x - second.x);
    difference.y = std::abs(first.y - second.y);
    return difference;
}

void alternativeHT::insertVirtualPoint(const PointState signalPrevFrame){   

    PointState offState;
    offState.point = signalPrevFrame.point;
    offState.ledState = false;
    offState.insertTime = ros::Time::now();
    offState.iterator = signalPrevFrame.iterator;
    buffer.end()[-1].push_back(offState);
}

void alternativeHT::checkIfThreeConsecutiveZeros(){
    
    if (first_call_){
        return;
    }
    std::scoped_lock lock(mutex_buffer);
    std::vector<PointState> & ptsCurr= buffer.end()[-1];
    std::vector<PointState> & ptsSec = buffer.end()[-2];
    std::vector<PointState> & ptsThird = buffer.end()[-3];

    for (int i = 0; i < (int)ptsCurr.size(); i++){
        bool insertedCurr = (ptsCurr[i].iterator == generatedSequences_.end()) ? false : true;
        bool boolCurr = ptsCurr[i].ledState; 
        for(int k = 0; k < (int)ptsSec.size(); k++){
            bool insertedSec = (ptsSec[i].iterator == generatedSequences_.end()) ? false : true;
            bool boolSec = ptsSec[k].ledState;
            for (int j =0; j<(int)ptsThird.size(); j++){
                bool insertedThird = (ptsThird[i].iterator == generatedSequences_.end()) ? false : true;
                bool boolThird = ptsThird[j].ledState; 
                if (insertedCurr && insertedSec && insertedThird){
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

void alternativeHT::cleanPotentialBuffer(){
    double timeThreeFrames = (1/60.0) * 4; 

    std::scoped_lock lock(mutex_generatedSequences_);

    auto b = generatedSequences_;

    for (auto i = generatedSequences_.begin(); i != generatedSequences_.end(); ++i){ 
        if (i->empty()){
            continue;
        }
        double insertTime = i->end()[-1].insertTime.toSec(); 
        double timeDiffLastInsert = std::abs(insertTime - ros::Time::now().toSec());
        if (timeDiffLastInsert > timeThreeFrames && i != generatedSequences_.end()){
            ROS_WARN("DELETING"); 
            i = generatedSequences_.erase(i);
        }
    }

    // std::cout <<"Size of sequences" << generatedSequences_.size() << "\n";
    // for (auto k : generatedSequences_) { 
    //     printVectorIfNotEmpty(k.second.seq, "Sequence");
    // }
}

std::vector<std::pair<PointState, int>> alternativeHT::getResults(){

    std::scoped_lock lock(mutex_generatedSequences_);
    std::vector<std::pair<PointState, int>> retrievedSignals;

    for (auto points : generatedSequences_){
        std::vector<bool> ledStates;
        for (auto point : points){
            ledStates.push_back(point.ledState);
        }
        double diff =  std::abs(points.end()[-1].insertTime.toSec() - ros::Time::now().toSec()); 
        std::cout << "time diff " <<  diff << std::endl;
        printVectorIfNotEmpty(ledStates, "seq");
        int id = findMatch(ledStates);
        // std::cout << " THE ID " << id << " the size of ledStates "<< ledStates.size() << "- liosts size " << generatedSequences_.size()<< "\n"; 
        retrievedSignals.push_back(std::make_pair(points.end()[-1], id));
    }

    return retrievedSignals;
}

int alternativeHT::findMatch(std::vector<bool> sequence){

    for ( int i = 2; i < (int)sequence.size(); i++) {
        if (sequence[i] == false && sequence[i-1] == false && sequence[i-2] == false){
            return -2;
        }
    }
    int id = matcher_->matchSignalWithCrossCorr(sequence);

    return id;
}

alternativeHT::~alternativeHT() {
}



/*****************************************************************************/
// stuff for moving average 
// void uvdar::alternativeHT::checkBoundingBoxIntersection(PointState & point){

//     std::vector<PointState> & ptsCurr   = buffer_.end()[-1];
//     std::vector<PointState> & ptsSec    = buffer_.end()[-2];
//     std::vector<PointState> & ptsThird  = buffer_.end()[-3];
    
//     std::vector<PointState> boxMatchPoints;
//     for (auto & p : ptsSec){
//         if(p.bbLeftUp.x > point.bbRightDown.x || point.bbLeftUp.x > p.bbRightDown.x || 
//         p.bbRightDown.y > point.bbLeftUp.y || point.bbRightDown.y > p.bbLeftUp.y){
//         } else{
//             ROS_INFO("HIT");
//             boxMatchPoints.push_back(p);
//         }
//     }
//     if ((int)boxMatchPoints.size() == 1){

//     } else if ((int)boxMatchPoints.size() > 1){
//         PointState closest  = findClosest(boxMatchPoints, point);
//     } else {
//         ROS_INFO("NO MATCH FOUND - start new sequence!");
//     }

// }

// PointState uvdar::alternativeHT::findClosest(const std::vector<PointState> boxMatchPoints, const PointState point){
//     cv::Point2d min;
//     PointState selected; 
//     min.x = FRAME_SIZE_X;
//     min.y = FRAME_SIZE_Y; 
//     for (const auto p : boxMatchPoints){
//         cv::Point2d diff = computeXYDiff(p.point, point.point);
//         if(diff.x < min.x && diff.y < min.y){
//             selected = p;
//             min.x = diff.x; 
//             min.y = diff.y;
//         }
//     }
//     return selected;
// }
