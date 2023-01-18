#include "alternativeHT.h"

using namespace uvdar;

alternativeHT::alternativeHT( int i_buffer_size ){
    
    buffer_size_ = i_buffer_size;
    ROS_WARN("PIXELSHITF?????"); // TODO:
    initBuffer();
}

void alternativeHT::initBuffer(){

    for ( int i = 0; i < buffer_size_; i++ ) {
        std::vector<std::pair<mrs_msgs::Point2DWithFloat, int>> p; 
        buffer_3DPoint_seqIndex_.push_back(p);
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

void alternativeHT::processBuffer( vectPoint3DWithIndex & ptsCurrentFrame) {

    if ((int)buffer_3DPoint_seqIndex_.size() < buffer_size_){
        buffer_3DPoint_seqIndex_.push_back(ptsCurrentFrame);
    } else {
        buffer_3DPoint_seqIndex_.erase(buffer_3DPoint_seqIndex_.begin()); 
        buffer_3DPoint_seqIndex_.push_back(ptsCurrentFrame);
    } 

    if (buffer_3DPoint_seqIndex_.size() < 2) {
            return;
    }

    vectPoint3DWithIndex & ptsCurrFrame = buffer_3DPoint_seqIndex_.end()[-1];
    vectPoint3DWithIndex & ptsPrevFrame = buffer_3DPoint_seqIndex_.end()[-2];

    findClosestAndLEDState( ptsCurrFrame, ptsPrevFrame );

    checkIfThreeConsecutiveZeros();
    cleanPotentialBuffer();


    std::cout << "The buffer size is: " << potentialSequences_.size() <<  std::endl;
    for (const auto l : potentialSequences_){
        printVectorIfNotEmpty(l.first, "sequence");
    }

}


void alternativeHT::findClosestAndLEDState(vectPoint3DWithIndex & ptsCurrentImg, vectPoint3DWithIndex & ptsPrevImg) {   

    for(int iPrevImg = 0;  iPrevImg < (int)ptsPrevImg.size(); iPrevImg++){
        bool nearestNeighbor = false;
        for(int iCurrentImg = 0; iCurrentImg < (int)ptsCurrentImg.size(); iCurrentImg++){
            mrs_msgs::Point2DWithFloat diff = computeXYDiff(ptsCurrentImg[iCurrentImg].first, ptsPrevImg[iPrevImg].first);
            if(diff.x <= max_pixel_shift_x_ && diff.y <= max_pixel_shift_y_){
                nearestNeighbor = true;           
                if(ptsPrevImg[iPrevImg].second == -1 && ptsCurrentImg[iCurrentImg].second == -1){
                    ptsPrevImg[iPrevImg].second         = potentialSequences_.size();
                    ptsCurrentImg[iCurrentImg].second   = potentialSequences_.size();
                }else if(ptsPrevImg[iPrevImg].second == -1 && ptsCurrentImg[iCurrentImg].second != -1){
                    ROS_WARN("POINT ALREADY ASSIGNED! WRONG ASSOCIATEN MAYBE OCURED");
                }else{
                    ptsCurrentImg[iCurrentImg].second = ptsPrevImg[iPrevImg].second;
                }
                if (buffer_3DPoint_seqIndex_.size() < 3){
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
            if (ptsPrevImg[iPrevImg].second == -1){
                ptsPrevImg[iPrevImg].second = potentialSequences_.size();
                insertPointToSequence(ptsPrevImg[iPrevImg]);
            }
            insertVirtualPoint(ptsCurrentImg, ptsPrevImg[iPrevImg]);
            insertPointToSequence(ptsCurrentImg.end()[-1]);
        }
    }
}

mrs_msgs::Point2DWithFloat uvdar::alternativeHT::computeXYDiff(mrs_msgs::Point2DWithFloat first, mrs_msgs::Point2DWithFloat second){
    
    mrs_msgs::Point2DWithFloat difference; 
    difference.x = std::abs(first.x - second.x);
    difference.y = std::abs(first.y - second.y);
    return difference;
}

void alternativeHT::insertPointToSequence(const point3DWithIndex point){
    int insertIndex = point.second;
    int stateCurrentPoint    = point.first.value;  
    cv::Point2d p = cv::Point2d(point.first.x, point.first.y);
    bool ledState;
    if (stateCurrentPoint){
        ledState = true; 
    } else {
        ledState = false;
    }
    
    // start new sequence
    if (insertIndex >= potentialSequences_.size()){
        std::vector<bool> newSeq;
        newSeq.push_back(ledState);
        potentialSequences_.push_back(std::make_pair(newSeq, p));
        return; 
    }

    if (potentialSequences_[insertIndex].first.size() == originalSequences_[0].size()){
        potentialSequences_[insertIndex].first.erase(potentialSequences_[insertIndex].first.begin());
    }

    potentialSequences_[insertIndex].first.push_back(ledState);
    potentialSequences_[insertIndex].second = cv::Point2d(point.first.x, point.first.y);

}

void alternativeHT::insertVirtualPoint(vectPoint3DWithIndex & pointVector, const point3DWithIndex pointPrevFrame){   

    point3DWithIndex virtualPoint;
    virtualPoint.first = pointPrevFrame.first;
    virtualPoint.second = pointPrevFrame.second;
    
    virtualPoint.first.value = 0; // equals LED "off" state

    pointVector.push_back(virtualPoint);
}


//TODO: Now not working!?
void alternativeHT::checkIfThreeConsecutiveZeros(){
    
    if (first_call_){
        return;
    }

    auto & currentFrame   =   buffer_3DPoint_seqIndex_.end()[-1];
    auto & lastFrame      =   buffer_3DPoint_seqIndex_.end()[-2];
    auto & thirdLastFrame = buffer_3DPoint_seqIndex_.end()[-3];

    for (int i = 0; i < (int)currentFrame.size(); i++) {
        const int valCurrFrame = currentFrame[i].first.value;
        const int index        = currentFrame[i].second;
        for (int k = 0; k < (int)lastFrame.size(); k++) {
            const int valLastFrame  = lastFrame[k].first.value;
            const int indexLast     = lastFrame[k].second; 
            if (index == indexLast) {
                for ( int l = 0; l < (int)thirdLastFrame.size(); l++ ){
                    const int valThirdLastFrame = thirdLastFrame[l].first.value;
                    const int indexThirdLast = thirdLastFrame[l].second; 
                    if (index == indexThirdLast){
                        if ( valCurrFrame == 0 && valLastFrame == 0 && valThirdLastFrame == 0){
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