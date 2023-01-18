#include "alternativeHT.h"

using namespace uvdar;

alternativeHT::alternativeHT( int i_buffer_size ){
    
    buffer_size_ = i_buffer_size;
    ROS_WARN("PIXELSHITF?????"); // TODO:
    initBuffer();
}

void alternativeHT::initBuffer(){


    for (int i = 0; i < buffer_size_; i++){
        std::vector<BlinkSignal> signalWithPointDummy;
        buffer.push_back(signalWithPointDummy);
    }
}

void alternativeHT::setDebugFlags(bool debug, bool visual_debug){
    debug_ = debug;
    visual_debug_ = visual_debug;
}

void alternativeHT::setSequences(std::vector<std::vector<bool>> i_sequences){
  
    originalSequences_ = i_sequences;
    matcher_ = std::make_unique<SignalMatcher>(originalSequences_);
    
    number_sequences_ = originalSequences_.size(); 
}

void alternativeHT::processBuffer(const mrs_msgs::ImagePointsWithFloatStampedConstPtr ptsMsg) {


    vectPoint3D points(std::begin(ptsMsg->points), std::end(ptsMsg->points));
    std::vector<BlinkSignal> signals; 
    for ( size_t i = 0; i < points.size(); i++ ) {
        BlinkSignal s;
        s.point = cv::Point(points[i].x, points[i].y);
        s.index = -1;
        s.ledState = true; // every existent point is "on"
        s.insertTime = ptsMsg->stamp;
        signals.push_back(s);
    }

    if((int)buffer.size() < buffer_size_){
        buffer.push_back(signals);
    }else{
        buffer.erase(buffer.begin());
        buffer.push_back(signals);
    }

    if((int)buffer.size() < 2){
        return;
    }

    findClosestAndLEDState(buffer.end()[-1], buffer.end()[-2]);

    checkIfThreeConsecutiveZeros();
    cleanPotentialBuffer();


    std::cout << "The buffer size is: " << potentialSequences_.size() <<  std::endl;
    for (const auto l : potentialSequences_){
        printVectorIfNotEmpty(l.first, "sequence");
    }

}


void alternativeHT::findClosestAndLEDState(std::vector<BlinkSignal> & ptsCurrentImg, std::vector<BlinkSignal> & ptsPrevImg) {   

    for(int iPrevImg = 0;  iPrevImg < (int)ptsPrevImg.size(); iPrevImg++){
        bool nearestNeighbor = false;
        for(int iCurrentImg = 0; iCurrentImg < (int)ptsCurrentImg.size(); iCurrentImg++){
            cv::Point2d diff = computeXYDiff(ptsCurrentImg[iCurrentImg].point, ptsPrevImg[iPrevImg].point);
            if(diff.x <= max_pixel_shift_x_ && diff.y <= max_pixel_shift_y_){
                nearestNeighbor = true;           
                if(ptsPrevImg[iPrevImg].index == -1 && ptsCurrentImg[iCurrentImg].index == -1){
                    ptsPrevImg[iPrevImg].index         = potentialSequences_.size();
                    ptsCurrentImg[iCurrentImg].index   = potentialSequences_.size();
                }else if(ptsPrevImg[iPrevImg].index == -1 && ptsCurrentImg[iCurrentImg].index != -1){
                    ROS_WARN("POINT ALREADY ASSIGNED! WRONG ASSOCIATEN MAYBE OCURED");
                }else{
                    ptsCurrentImg[iCurrentImg].index = ptsPrevImg[iPrevImg].index;
                }
                if (buffer.size() < 3){
                    std::cout << "Should only be called in the beginning!\n"; 
                    insertPointToSequence(ptsPrevImg[iPrevImg]);
                }
                insertPointToSequence(ptsCurrentImg[iCurrentImg]);
                break;
            }else{
                nearestNeighbor = false;
            }
        }
        if(nearestNeighbor == false){
            if (ptsPrevImg[iPrevImg].index == -1){
                ptsPrevImg[iPrevImg].index = potentialSequences_.size();
                insertPointToSequence(ptsPrevImg[iPrevImg]);
            }
            insertVirtualPoint(ptsCurrentImg, ptsPrevImg[iPrevImg]);
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

void alternativeHT::insertPointToSequence(BlinkSignal signal){
    int insertIndex = signal.index;
    cv::Point2d p = signal.point;
    
    // start new sequence
    if (insertIndex >= potentialSequences_.size()){
        std::vector<bool> newSeq;
        newSeq.push_back(signal.ledState);
        potentialSequences_.push_back(std::make_pair(newSeq, p));
        return; 
    }

    if (potentialSequences_[insertIndex].first.size() == originalSequences_[0].size()){
        potentialSequences_[insertIndex].first.erase(potentialSequences_[insertIndex].first.begin());
    }

    potentialSequences_[insertIndex].first.push_back(signal.ledState);
    potentialSequences_[insertIndex].second = signal.point;

}

void alternativeHT::insertVirtualPoint(std::vector<BlinkSignal> & signalVector, const BlinkSignal signalPrevFrame){   

    BlinkSignal offState;
    offState.point = signalPrevFrame.point;
    offState.index = signalPrevFrame.index;
    offState.ledState = false;
    offState.insertTime = ros::Time::now(); //TODO: DOUBLE CHECK SAME FORMAT?! 
    signalVector.push_back(offState);
}


//TODO: Now not working!?
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

    for (int s = 0; s < potentialSequences_.size(); s++){
        if (potentialSequences_[s].first.size() > 2){
            for (int j = 2; j < potentialSequences_[s].first.size(); j++){
                // if (potentialSequences_[s].first[j-2] == 0 && potentialSequences_[s].first[j-1] == 0 && potentialSequences_[s].first[j] == 0){
                //     ROS_WARN("I'm here ");
                //     potentialSequences_.erase(potentialSequences_.begin()+s);

                // }
                bool thirdLast = potentialSequences_[s].first.end()[-3];
                bool secondLast = potentialSequences_[s].first.end()[-2];
                bool lastElement = potentialSequences_[s].first.end()[-1];
                if (!thirdLast && !secondLast && !lastElement){
                    ROS_WARN("HERE");
                    potentialSequences_[s].first.end()[-2] = true;
                    potentialSequences_[s].first.end()[-1] = true;
                }

            }
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

// TODO: not called right now!!
std::vector<std::pair<cv::Point2d, int>> alternativeHT::getResult(){
    
    std::vector<std::pair<cv::Point2d, int>> retrievedSignals;

        // std::cout << "The size of the sequence " << potentialSequences_.size() << std::endl;
        for (const auto  sequence : potentialSequences_) {
            // if (debug_) printVectorIfNotEmpty(sequence.first, "predicted sequence");
            int id = findMatch(sequence.first);
            
            // printVectorIfNotEmpty(sequence.first, "pot");

            cv::Point2d originPoint = sequence.second;
            retrievedSignals.push_back(std::make_pair(originPoint, id));
        }
    return retrievedSignals;
}

alternativeHT::~alternativeHT() {
}