#include "alternativeHT.h"
#include <iostream>

using namespace uvdar;

alternativeHT::alternativeHT( int i_buffer_size ){
    
    buffer_size_ = i_buffer_size;
    
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
  
    sequences_ = i_sequences;
    matcher_ = std::make_unique<SignalMatcher>(sequences_);
    
    number_sequences_ = sequences_.size(); 

    initSequenceBuffer();

}

void alternativeHT::initSequenceBuffer(){

    // TODO: right now out of bounds execption might happen!
    for ( int i = 0; i < number_sequences_; i++ ){
        const int sequenceLength = sequences_[0].size() - 2; // TODO: fix that constant - relative arbitrary 
        if (sequenceLength < 0 ) {ROS_ERROR("[AlternativeHT]: The sequence length is too short!");}
        std::vector<bool> k(sequenceLength, false);
        std::pair<std::vector<bool>, cv::Point2d> pair;
        pair.first = k;
        potentialSequences_.push_back(pair);
    }
}

void alternativeHT::processBuffer( vectPoint3DWithIndex & ptsCurrentFrame, const int buffer_cnt) {

    buffer_3DPoint_seqIndex_[buffer_cnt] = ptsCurrentFrame;

    if (first_call_ && buffer_cnt == 0 ) {
            return;
    }

    int indexPrevFrame = -1; 
    if (buffer_cnt == 0 ){
        indexPrevFrame = buffer_size_ - 1; 
    } else if ( buffer_cnt <= buffer_size_){
        indexPrevFrame = buffer_cnt - 1; 
    } 
    
    vectPoint3DWithIndex & ptsPrevFrame = buffer_3DPoint_seqIndex_[indexPrevFrame];

    findClosestAndLEDState( buffer_3DPoint_seqIndex_[buffer_cnt], ptsPrevFrame );

    insertToSequencesBuffer(buffer_3DPoint_seqIndex_[buffer_cnt]);

}


void alternativeHT::findClosestAndLEDState(vectPoint3DWithIndex & ptsCurrentImg, vectPoint3DWithIndex & ptsPrevImg) {   

    for ( size_t iPrevImg = 0;  iPrevImg < ptsPrevImg.size(); iPrevImg++ ) {
        bool nearestNeighbor = false;
        for ( size_t iCurrentImg = 0; iCurrentImg < ptsCurrentImg.size(); iCurrentImg++ ) {

            mrs_msgs::Point2DWithFloat diff = computeXYDifference(ptsCurrentImg[iCurrentImg].first, ptsPrevImg[iPrevImg].first);

            if (diff.x > max_pixel_shift_x_ || diff.y > max_pixel_shift_y_) {
                nearestNeighbor = false; 
            }
            else {   
                nearestNeighbor = true;
                ptsCurrentImg[iCurrentImg].first.value = 1; // match found - LED is "on"
                int wantedIndex = ptsPrevImg[iPrevImg].second;
                int currentIndex = ptsCurrentImg[iCurrentImg].second; 
                swapIndex( wantedIndex, currentIndex, ptsCurrentImg );
                break;
            }
        }
        if (nearestNeighbor == false) {
            insertVirtualPointAndUpdateIndices(ptsCurrentImg, ptsPrevImg[iPrevImg]);
        }
    }

}

void alternativeHT::insertToSequencesBuffer(vectPoint3DWithIndex & pts){

    for (size_t i = 0; i < pts.size(); i++) {
        int sequenceIndex = pts[i].second;
        cv::Point2d currPoint = cv::Point(pts[i].first.x, pts[i].first.y);
        int currPointValue = pts[i].first.value;
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
        int id = findMatch(potentialSequences_[sequenceIndex].first);
        std::cout << "the ID is " << id << std::endl;
        // if ( id >= 0 ){
        //     retrievedSignals_.push_back(std::make_pair(currPoint, id));
        // }
        // if (debug_) std::cout << "[AlternativeHT]: The current signal ID is: " << id << std::endl;
    }
}

mrs_msgs::Point2DWithFloat uvdar::alternativeHT::computeXYDifference(mrs_msgs::Point2DWithFloat first, mrs_msgs::Point2DWithFloat second){

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
    std::cout << wantedIndex << " Current " << currentIndex << std::endl;
    
    points[currentIndex].second = wantedIndex;

}

void alternativeHT::insertVirtualPointAndUpdateIndices(vectPoint3DWithIndex &pointVector, const point3DWithIndex pointPrevFrame)
{   
    point3DWithIndex p;
    p.first = pointPrevFrame.first;
    p.first.value = 0; // equals LED "off" state

    p.second = pointVector.size(); // prevention for inserting duplicated indices
    pointVector.push_back(p);

    swapIndex( pointPrevFrame.second, p.second, pointVector );

}

int alternativeHT::findMatch(std::vector<bool> sequence){

    if (sequence.size() > sequences_[0].size()){
        sequence.clear(); // TODO:
        return -2;
    }
    if (debug_) std::cout << "[AlternativeHT]: The signal sequence: ";
    for ( int i = 2; i < sequence.size(); i++) {
        if (debug_) {
                if (sequence[i]) {
                    std::cout << "1,";
                } else {
                std::cout << "0,";
                }
        }
        if (sequence[i] == false && sequence[i-1] == false && sequence[i-2] == false){
            sequence.clear(); // TODO:
            return -3;
        }
    }
    if (debug_) std::cout << std::endl;
    
    int id = matcher_->matchSignal(sequence);
    sequence.clear(); // TODO:
    return id;
}

std::vector<std::pair<cv::Point2d, int>> alternativeHT::getResult(){
    
    std::vector<std::pair<cv::Point2d, int>> retrievedSignals;

    // for (auto sequence : potentialSequences_) {
    //     int id = findMatch(sequence.first);
    //     std::cout << "THE IDE IS " << id << std::endl;
    //     cv::Point2d originPoint = sequence.second;
    //     retrievedSignals.push_back(std::make_pair(originPoint, id));
    // }

    
    return retrievedSignals;
}


alternativeHT::~alternativeHT() {
}