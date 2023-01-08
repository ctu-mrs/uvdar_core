#include "alternativeHT.h"
#include <iostream>
#include <ros/console.h>

using namespace uvdar;

alternativeHT::alternativeHT( int i_buffer_size ){
    
    buffer_size_ = i_buffer_size;
    
    initBuffer();
}

void alternativeHT::initBuffer(){
    
    for ( int i = 0; i < buffer_size_; i++ ) {
        std::vector<std::pair<mrs_msgs::Point2DWithFloat, int>> p; 
        buffer_3DPoint_seqIndex.push_back(p);
    }
}

void alternativeHT::setSequences(std::vector<std::vector<bool>> i_sequences){
  
    sequences_ = i_sequences;
    matcher_ = std::make_unique<SignalMatcher>(sequences_);
    
    number_sequences_ = sequences_.size(); 

    initSequenceBuffer();

}

void alternativeHT::initSequenceBuffer(){

    for ( int i = 0; i < number_sequences_; i++ ){
        std::vector<int> p(10,-1);
        std::vector<bool> k;
        // p.x = p.y = p.value = -1;
        potentialSequences_.push_back(k);
        signal.push_back(k);
    }
}

void alternativeHT::processBuffer( vectPoint3DWithIndex & ptsCurrentFrame, const int buffer_cnt) {

    buffer_3DPoint_seqIndex[buffer_cnt] = ptsCurrentFrame;
    // buffer_3DPoint_seqIndex.at(buffer_cnt) 

    if (first_call_ && buffer_cnt == 0 ) {
            return;
    }

    int indexPrevFrame = -1; 
    if (buffer_cnt == 0 ){
        indexPrevFrame = buffer_size_ - 1; 
    } else if ( buffer_cnt <= buffer_size_){
        indexPrevFrame = buffer_cnt - 1; 
    } 
    
    vectPoint3DWithIndex & ptsPrevFrame = buffer_3DPoint_seqIndex[indexPrevFrame];

    findClosestAndLEDState( buffer_3DPoint_seqIndex.at(buffer_cnt), ptsPrevFrame );

    // for (const auto k : ptsPrevFrame){
    //     std::cout << "The val " << k.first.value << std::endl;
    // }
    insertToSequencesBuffer(ptsPrevFrame);

}


void alternativeHT::findClosestAndLEDState(vectPoint3DWithIndex & ptsCurrentImg, vectPoint3DWithIndex & ptsPrevImg) {   

    // for (const auto k : ptsCurrentImg) {
    //     std::cout << "Index Current " << k.second << " Values x: " << k.first.x << " y " << k.first.y << std::endl;
    // }
   

    // for (const auto k : ptsPrevImg) {
    //     std::cout << "Index Prev " << k.second << " Values x: " << k.first.x << " y " << k.first.y << std::endl;
    // }

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
                // std::cout << wantedIndex << " Current " << currentIndex << std::endl;
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
    
    for (const auto p : pts) {
        if (p.second > number_sequences_) {
            ROS_WARN("[AlternativeHT]: Can't insert point to sequence buffer! The current maximal index number is: %d. The wanted insertion has the index: %d", number_sequences_, p.second); 
            break;
        }

        if(p.first.value == 1) {
            potentialSequences_.at(p.second).push_back(true); 
        } else {
            potentialSequences_.at(p.second).push_back(false);
        }
        if (potentialSequences_[p.second].size() > 12){
            for (auto r : potentialSequences_[p.second]){
                if (r) { std::cout << "1,";}
                else {std::cout << "0,";}
            }
            std::cout << std::endl;
            int k = matcher_->matchSignal(potentialSequences_[p.second]);
            potentialSequences_[p.second].clear();
            std::cout << "The signal is " << k  << std::endl;
        }
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
    
    points[currentIndex].second = wantedIndex;

}

void alternativeHT::insertVirtualPointAndUpdateIndices(vectPoint3DWithIndex &pointVector, const point3DWithIndex pointPrevFrame)
{   
    // std::cout << "Insert empty" << std::endl;
    point3DWithIndex p;
    p.first = pointPrevFrame.first;
    p.first.value = 0; // equals LED "off" state
    
    p.second = pointVector.size(); // prevention for inserting duplicated indices
    pointVector.push_back(p);

    swapIndex( pointPrevFrame.second, p.second, pointVector );

}


void alternativeHT::getResult(){
    // std::cout << "I'm  called!!!!" << std::endl;
    // int p = retrieveSignalID();


    // for ( int i = 0; i < potentialSequences_.size(); i++){
    //     std::cout << "The Sequence: " << std::endl;
        
    //     for (const auto k : potentialSequences_[i]){
    //         if (k) {
    //         std::cout << "1,";
    //         } else {
    //             std::cout << "0,";
    //         }
    //     }
    //     std::cout << std::endl;


    //     int k = matcher_->matchSignal(potentialSequences_[i]);
    //     std::cout << "The signal ID : " << k << std::endl;
    // }

    // for ( int i = 0; i < potentialSequences_.size(); i++){
    //     potentialSequences_[i].clear();
    // }

}


alternativeHT::~alternativeHT() {

}