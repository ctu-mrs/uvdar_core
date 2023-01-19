#include "alternativeHT.h"

using namespace uvdar;

alternativeHT::alternativeHT( int i_buffer_size ){
    
    buffer_size_ = i_buffer_size;
    initBuffer();
}

void alternativeHT::initBuffer(){

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

    std::vector<mrs_msgs::Point2DWithFloat> point3D(std::begin(ptsMsg->points), std::end(ptsMsg->points));
    std::vector<PointState> points; 
    for ( size_t i = 0; i < point3D.size(); i++ ) {
        PointState p;
        p.point = cv::Point(point3D[i].x, point3D[i].y);
        p.index = -1;
        p.ledState = true; // every existent point is "on"
        p.insertTime = ptsMsg->stamp;
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

    findClosestAndLEDState();

    checkIfThreeConsecutiveZeros();
    cleanPotentialBuffer();
}


void alternativeHT::findClosestAndLEDState() {   

    std::vector<PointState> & ptsCurrentImg = buffer.end()[-1]; // newest points;
    std::vector<PointState> & ptsPrevImg    = buffer.end()[-2]; // second newest points

    for(int iPrev = 0;  iPrev < (int)ptsPrevImg.size(); iPrev++){
        bool nearestNeighbor = false;
        for(int iCurr = 0; iCurr < (int)ptsCurrentImg.size(); iCurr++){
            cv::Point2d diff = computeXYDiff(ptsCurrentImg[iCurr].point, ptsPrevImg[iPrev].point);
            if(diff.x <= max_pixel_shift_x_ && diff.y <= max_pixel_shift_y_){
                nearestNeighbor = true;           
                if(ptsPrevImg[iPrev].index == -1){
                        {
                        std::scoped_lock lock(mutex_foundSequences_);
                        int size = (int)foundSequences_.size();  
                        ptsCurrentImg[iCurr].index  = size;
                        ptsPrevImg[iPrev].index     = size;
                        }
                }else{
                    ptsCurrentImg[iCurr].index = ptsPrevImg[iPrev].index;
                }
                if (buffer.size() < 3){
                    insertPointToSequence(ptsPrevImg[iPrev]);
                }
                std::cout << "index " << ptsCurrentImg[iCurr].index <<"\n";
                insertPointToSequence(ptsCurrentImg[iCurr]);
                break;
            }else{
                nearestNeighbor = false;
            }
        }
        if(nearestNeighbor == false){
            if (ptsPrevImg[iPrev].index == -1){
                {
                    std::scoped_lock lock(mutex_foundSequences_);
                    int size = (int)foundSequences_.size();  
                    ptsPrevImg[iPrev].index = size;
                    std::cout << "The size " << size << std::endl;
                    ROS_ERROR("wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww"); 
                }
                insertPointToSequence(ptsPrevImg[iPrev]);
            }
            std::cout << "VP index " << ptsPrevImg[iPrev].index << std::endl;
            insertVirtualPoint(ptsPrevImg[iPrev]);
            insertPointToSequence(ptsCurrentImg.end()[-1]);
        }
    }
}

cv::Point2d uvdar::alternativeHT::computeXYDiff(const cv::Point2d first, const cv::Point2d second){
    
    cv::Point2d difference; 
    difference.x = std::abs(first.x - second.x);
    difference.y = std::abs(first.y - second.y);
    return difference;
}

void alternativeHT::insertPointToSequence(PointState signal){
    // std::cout << "Found Seq file size" << foundSequences_.size()  << std::endl;
    std::scoped_lock lock(mutex_foundSequences_);

    auto k = foundSequences_.find(signal.index);
    
    // start new sequence, if signal index is not existent
    if (k == foundSequences_.end()){
        std::vector<bool> newSequence;
        newSequence.push_back(signal.ledState);
        sequenceWithPoint s; 
        s.lastInsertedPoint = signal;
        s.seq = newSequence;
        foundSequences_[signal.index] = s;
        std::cout << "here" << std::endl;
        return; 
    }

    // start deleting first element of sequence vector if it is equivalent to the wanted size 
    if (k->second.seq.size() == originalSequences_[0].size()){
        k->second.seq.erase(k->second.seq.begin());
    }

    k->second.lastInsertedPoint = signal;
    k->second.seq.push_back(signal.ledState);
}

void alternativeHT::insertVirtualPoint(const PointState signalPrevFrame){   

    PointState offState;
    offState.point = signalPrevFrame.point;
    offState.index = signalPrevFrame.index;
    offState.ledState = false;
    offState.insertTime = ros::Time::now();
    buffer.end()[-1].push_back(offState);
}

void alternativeHT::checkIfThreeConsecutiveZeros(){
    
    if (first_call_){
        return;
    }

    auto & currentFrame   =   buffer.end()[-1];
    auto & lastFrame      =   buffer.end()[-2];
    auto & thirdLastFrame = buffer.end()[-3];

    for (int i = 0; i < (int)currentFrame.size(); i++) {
        bool valCurrFrame = currentFrame[i].ledState;
        const int index = currentFrame[i].index;
        for (int k = 0; k < (int)lastFrame.size(); k++) {
            const int valLastFrame = lastFrame[k].ledState;
            const int indexLast = lastFrame[k].index; 
            if (index == indexLast) {
                for ( int l = 0; l < (int)thirdLastFrame.size(); l++ ){
                    const int valThirdLastFrame = thirdLastFrame[l].ledState;
                    const int indexThirdLast = thirdLastFrame[l].index; 
                    if (index == indexThirdLast){
                        if (!valCurrFrame && !valLastFrame && !valThirdLastFrame){
                            if (debug_){
                                ROS_WARN("[AlternativeHT]: Three consecutive points zero - Is Manchester Coding enabled?");
                            }
                            std::cout << "[AlternativeHT]: Three consecutive points zero - Is Manchester Coding enabled?" << std::endl;
                            currentFrame.erase(currentFrame.begin() + i);
                            lastFrame.erase(lastFrame.begin() + k);
                            thirdLastFrame.erase(thirdLastFrame.begin() + l);
                        }
                    }
                }
            }
        }
    }
}


void alternativeHT::cleanPotentialBuffer(){
    double timeThreeFrames = (1/60.0) * 4; 


    std::scoped_lock lock(mutex_foundSequences_);

    std::map<int, sequenceWithPoint> b = foundSequences_;
    
    for (auto k : b){
        double insertTime = k.second.lastInsertedPoint.insertTime.toSec();
        double timeDiffLastInsert = std::abs(insertTime - ros::Time::now().toSec());
        if ( timeDiffLastInsert > timeThreeFrames){
            ROS_WARN("DELETING"); 
            foundSequences_.erase(k.first);
        }
    }
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

std::vector<std::pair<cv::Point2d, int>> alternativeHT::getResult(){
    
    std::scoped_lock lock(mutex_foundSequences_);
    std::vector<std::pair<cv::Point2d, int>> retrievedSignals;

    for (auto k : foundSequences_){
        int id = findMatch(k.second.seq);
        std::cout << " The id " << id << std::endl;
        printVectorIfNotEmpty(k.second.seq, "Sequence");
        cv::Point2d originPoint = k.second.lastInsertedPoint.point;
        retrievedSignals.push_back(std::make_pair(originPoint, id));
        }
    return retrievedSignals;
}

alternativeHT::~alternativeHT() {
}