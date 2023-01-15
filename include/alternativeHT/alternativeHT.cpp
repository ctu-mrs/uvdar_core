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

    initSequenceBuffer();

}

void alternativeHT::initSequenceBuffer(){

    // TODO: right now out of bounds execption might happen!
    for ( int i = 0; i < number_sequences_; i++ ){
        const int sequenceLength = originalSequences_[0].size() - 2; // TODO: fix that constant - relative arbitrary 
        if (sequenceLength < 0 ) {ROS_ERROR("[AlternativeHT]: The sequence length is too short!");}
        std::pair<std::vector<bool>, cv::Point2d> pair;
        potentialSequences_.push_back(pair);
    }
}

void alternativeHT::processBuffer( vectPoint3DWithIndex & ptsCurrentFrame) {

    if ((int)buffer_3DPoint_seqIndex_.size() < buffer_size_){
        buffer_3DPoint_seqIndex_.push_back(ptsCurrentFrame);
    } else {
        // buffer_3DPoint_seqIndex_.pop_front();
        buffer_3DPoint_seqIndex_.erase(buffer_3DPoint_seqIndex_.begin());
        buffer_3DPoint_seqIndex_.push_back(ptsCurrentFrame);
    } 

    if (first_call_ ) {
            return;
    }

    vectPoint3DWithIndex & ptsCurrFrame = buffer_3DPoint_seqIndex_.end()[-1];
    vectPoint3DWithIndex & ptsPrevFrame = buffer_3DPoint_seqIndex_.end()[-2];

    findClosestAndLEDState( ptsCurrFrame, ptsPrevFrame );
    // printVectorIfNotEmpty(buffer_3DPoint_seqIndex_[buffer_cnt], "")

    checkIfThreeConsecutiveZeros();

    std::cout << "The buffer size " << ptsCurrentFrame.size() << std::endl;

    insertToSequencesBuffer(ptsCurrFrame);

}


void alternativeHT::findClosestAndLEDState(vectPoint3DWithIndex & ptsCurrentImg, vectPoint3DWithIndex & ptsPrevImg) {   

    for ( int iPrevImg = 0;  iPrevImg < (int)ptsPrevImg.size(); iPrevImg++ ) {
        bool nearestNeighbor = false;
        for ( int iCurrentImg = 0; iCurrentImg < (int)ptsCurrentImg.size(); iCurrentImg++ ) {
            
            mrs_msgs::Point2DWithFloat diff = computeXYDiff(ptsCurrentImg[iCurrentImg].first, ptsPrevImg[iPrevImg].first);
            if (diff.x <= max_pixel_shift_x_ && diff.y <= max_pixel_shift_y_){
                nearestNeighbor = true;
                swapIndex(ptsPrevImg[iPrevImg].second, ptsCurrentImg[iCurrentImg].second, ptsCurrentImg);
                break;
            } else {
                nearestNeighbor = false;
            }

        }
        if (nearestNeighbor == false) {
            insertVirtualPointAndUpdateIndices(ptsCurrentImg, ptsPrevImg[iPrevImg]);
        }
    }
    //     std::cout << "======================"  << std::endl;
    // for (auto p : ptsCurrentImg) {
    //     std::cout << "Point" << p.first.x  << "," << p.first.y << " Index " << p.second << std::endl;
    // }
    // std::cout << "==========" << std::endl;

}

mrs_msgs::Point2DWithFloat uvdar::alternativeHT::computeXYDiff(mrs_msgs::Point2DWithFloat first, mrs_msgs::Point2DWithFloat second){

    mrs_msgs::Point2DWithFloat p; 
    p.x = std::abs(first.x - second.x);
    p.y = std::abs(first.y - second.y);

    return p;

}

void alternativeHT::swapIndex(const int wantedIndex, const int currentIndex ,vectPoint3DWithIndex & points){
    
    if ( wantedIndex == currentIndex ) {
        return;
    }

    for ( auto & p : points ) {

        if ( p.second == wantedIndex ) {
            int swapIndexTemp = points[currentIndex].second;
            p.second = swapIndexTemp;
        } 
    }
    // std::cout << "SWAP - Wanted Prev" <<  wantedIndex << " Current " << currentIndex << std::endl;
    
    points[currentIndex].second = wantedIndex;

}

void alternativeHT::insertVirtualPointAndUpdateIndices(vectPoint3DWithIndex & pointVector, const point3DWithIndex pointPrevFrame)
{   

    point3DWithIndex p;
    p.first = pointPrevFrame.first;
    p.first.value = 0; // equals LED "off" state

    p.second = pointVector.size(); // prevention for inserting duplicated indices
    pointVector.push_back(p);
    swapIndex( pointPrevFrame.second, p.second, pointVector );

}

//TODO: Now not working!
void alternativeHT::checkIfThreeConsecutiveZeros(){

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
    // std::cout << "The sizes " << currentFrame.size() << "," << lastFrame.size() << ", " << thirdLastFrame.size()<< std::endl;

}

void alternativeHT::insertToSequencesBuffer(vectPoint3DWithIndex pts){

    std::scoped_lock lock(mutex_potSequence_);
    for (size_t i = 0; i < pts.size(); i++) {
        int sequenceIndex = pts[i].second;
        int currPointValue = pts[i].first.value;
        cv::Point2d currPoint = cv::Point(pts[i].first.x, pts[i].first.y);

        size_t sizeCurrSeq = potentialSequences_[sequenceIndex].first.size();

        if ( sizeCurrSeq == ( originalSequences_[0].size()) ) // TODO: hard coded value..
        {
            // printVectorIfNotEmpty(potentialSequences_[sequenceIndex].first, "insert");
            potentialSequences_[sequenceIndex].first.erase(potentialSequences_[sequenceIndex].first.begin()); 
        }

        if (sequenceIndex > number_sequences_) {
            ROS_WARN("[AlternativeHT]: Can't insert point to sequence buffer! The current maximal index number is: %d. The wanted insertion has the index: %d", number_sequences_, sequenceIndex); 
            break;
        }
        if(currPointValue == 1) {
            potentialSequences_[sequenceIndex].first.push_back(true); 
        } else {
            potentialSequences_[sequenceIndex].first.push_back(false);
        }
        potentialSequences_[sequenceIndex].second = currPoint;
    }
}

int alternativeHT::findMatch(std::vector<bool> sequence){

    for ( int i = 2; i < (int)sequence.size(); i++) {
        if (sequence[i] == false && sequence[i-1] == false && sequence[i-2] == false){
            return -2;
        }
    }
    
    int id = matcher_->matchSignalWithCrossCorr(sequence);
    // int id = matcher_->matchSignal(sequence);

    return id;
}


std::vector<std::pair<cv::Point2d, int>> alternativeHT::getResult(){
    
    std::vector<std::pair<cv::Point2d, int>> retrievedSignals;

    {
        std::scoped_lock lock(mutex_potSequence_);
        for (const auto  sequence : potentialSequences_) {
            // if (debug_) printVectorIfNotEmpty(sequence.first, "predicted sequence");
            int id = findMatch(sequence.first);
            cv::Point2d originPoint = sequence.second;
            retrievedSignals.push_back(std::make_pair(originPoint, id));
}
    }
    return retrievedSignals;
}

alternativeHT::~alternativeHT() {
}