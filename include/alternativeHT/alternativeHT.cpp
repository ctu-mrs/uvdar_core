#include "alternativeHT.h"
#include <iostream>
#include <ros/console.h>

using namespace uvdar;

alternativeHT::alternativeHT( int i_buffer_size ){
    
    buffer_size_ = i_buffer_size;
    
    initBuffer();
    initSequenceBuffer();
}

void alternativeHT::initBuffer(){
    
    for ( int i = 0; i < buffer_size_; i++ ) {
        std::vector<std::pair<mrs_msgs::Point2DWithFloat, int>> p; 
        buffer_3DPoint_seqIndex.push_back(p);
    }
}

void alternativeHT::initSequenceBuffer(){

    if (max_sequences_seen_ > 10000) {
        initialized_ = false;
        ROS_ERROR("[AlternativeHT]: The maximal sequence size is too big! ");
        return;
    }

    for ( int i = 0; i < max_sequences_seen_; i++ ){
        mrs_msgs::Point2DWithFloat p;
        p.x = p.y = p.value = -1;
        sequences.push_back(p);
    }
}

void alternativeHT::processBuffer( vectPoint3DWithIndex & ptsCurrentFrame, const int buffer_cnt) {

    if (!initialized_) return;
    // std::cout << " Size " << ptsCurrentFrame.size() << std::endl;

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

    insertToSequencesBuffer(ptsPrevFrame);




    // std::tuple<int,int,int> buffer_indices =  findCurrentIndexState(buffer_cnt);


    // evalulateBuffer(buffer_indices);

  
}


void alternativeHT::findClosestAndLEDState(vectPoint3DWithIndex & ptsCurrentImg, vectPoint3DWithIndex & ptsPrevImg) {   

    // std::reverse(ptsCurrentImg.begin(), ptsCurrentImg.end()); // just for testing!

    // for ( int i = 0; i < ptsPrevImg.size(); i++ ) {
    //     ptsPrevImg[i].second = ptsPrevImg.size()  + i + 100; 
    // }


    for (const auto k : ptsCurrentImg) {
        std::cout << "Index Current " << k.second << " Values x: " << k.first.x << " y " << k.first.y << std::endl;
    }
   

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
                std::cout << wantedIndex << " Current " << currentIndex << std::endl;
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
    
    for ( const auto p : pts ) {
        if ( p.second > max_sequences_seen_ ) {
            ROS_WARN("[AlternativeHT]: Can't insert point to sequence buffer! The max number of indices is: %d. The wanted insertion has the index: %d", max_sequences_seen_, p.second); 
            break;
        }
        sequences.at(p.second) = p.first; 

    }
}
mrs_msgs::Point2DWithFloat uvdar::alternativeHT::computeXYDifference(mrs_msgs::Point2DWithFloat first, mrs_msgs::Point2DWithFloat second){

    mrs_msgs::Point2DWithFloat p; 

    p.x = std::abs(first.x - second.x);
    p.y = std::abs(first.y - second.y);

    return p;

}

void alternativeHT::swapIndex( const int wantedIndex, const int currentIndex ,vectPoint3DWithIndex & points){
    
    
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



alternativeHT::~alternativeHT() {

}