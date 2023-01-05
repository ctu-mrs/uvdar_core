#include "alternativeHT.h"
#include <iostream>
// #include <ros/ros.h>


#include <ros/console.h>
using namespace uvdar;

alternativeHT::alternativeHT( int i_buffer_size ){
    
    buffer_size_ = i_buffer_size;
    
    for (int i = 0; i < buffer_size_; i++ ){
        vectPoint3D p;
        buffer_with_frame_points.push_back(p);
    }


    for ( int i = 0; i < buffer_size_; i++ ) {
        std::vector<std::pair<mrs_msgs::Point2DWithFloat, int>> p; 
        buffer_3DPoint_seqIndex.push_back(p);
    }

    for ( int i = 0; i < max_sequences_seen; i++ ){
        mrs_msgs::Point2DWithFloat p;
        sequences.push_back(p);
    }

}

alternativeHT::~alternativeHT()
{}

void alternativeHT::processBuffer( vectPoint3DWithIndex & ptsCurrentFrame, const int buffer_cnt) {

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


    for ( const auto p : ptsPrevFrame ) {
        if ( p.second > max_sequences_seen ) {
            ROS_WARN("[AlternativeHT]: Can't insert point to sequence buffer! The max number of indices is: %d. The wanted insertion has the index: %d", max_sequences_seen, p.second); 
            break;
        }

        sequences.at(p.second) = p.first; 

    }

    // std::tuple<int,int,int> buffer_indices =  findCurrentIndexState(buffer_cnt);


    // evalulateBuffer(buffer_indices);

  
}


void alternativeHT::findClosestAndLEDState(vectPoint3DWithIndex & ptsCurrentImg, vectPoint3DWithIndex & ptsPrevImg) {   

    // std::reverse(ptsCurrentImg.begin(), ptsCurrentImg.end()); // just for testing!

    for ( int i = 0; i < ptsPrevImg.size(); i++ ) {
        ptsPrevImg[i].second = ptsPrevImg.size()  + i + 100; 
    }


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

    for (const auto k : ptsCurrentImg) {
        std::cout << "After Swap " << k.second << std::endl;
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

// depricated??
std::tuple<int, int,int> alternativeHT::findCurrentIndexState(const int buffer_cnt){
    
    int firstIndex  = buffer_cnt;
    int secondIndex = buffer_cnt - 1; 
    int thirdIndex  = buffer_cnt - 2; 

    if ( firstIndex == 0 ){
        secondIndex = buffer_size_ - 1; 
        thirdIndex  = buffer_size_ - 2; 
    }
    if (secondIndex == 0 ) {
        thirdIndex = buffer_size_ - 1; 
    }

    return {firstIndex, secondIndex, thirdIndex};

}

void alternativeHT::evalulateBuffer( const std::tuple<int,int,int> buffer_indices ){

    if (first_call_) return; 
    
    const int first     = std::get<0>(buffer_indices);
    const int second    = std::get<1>(buffer_indices);
    const int third     = std::get<2>(buffer_indices);


    // for ( auto & firstFramePoint : fast_buffer_.at(currFrameIndex).get_pts_reference() ) {
    //     for ( auto & secondFramePoint : fast_buffer_.at(prevFrameIndex).get_pts_reference() ){
            
    for ( size_t firstBuff = 0; firstBuff <  buffer_with_frame_points[first].size(); firstBuff++ ) {
        for (size_t secondBuff = 0; secondBuff < buffer_with_frame_points[second].size(); secondBuff++ ) {
            
            mrs_msgs::Point2DWithFloat pFirstBuff =  buffer_with_frame_points[first][firstBuff];
            mrs_msgs::Point2DWithFloat pSecondBuff =  buffer_with_frame_points[second][secondBuff];

            mrs_msgs::Point2DWithFloat diffFirstSecond = computeXYDifference(pFirstBuff, pSecondBuff); 

        
            if (diffFirstSecond.x > max_pixel_shift_x_ || diffFirstSecond.y > max_pixel_shift_y_) {
                return;
            } else {   
                if ( pFirstBuff.value == 0 && pSecondBuff.value == 0 ) {

                    for ( size_t thirdBuff = 0; thirdBuff < buffer_with_frame_points[third].size(); thirdBuff++ ) {
                        
                        mrs_msgs::Point2DWithFloat pThirdBuff = buffer_with_frame_points[third][thirdBuff];
                        mrs_msgs::Point2DWithFloat diffSecondThird = computeXYDifference(pThirdBuff, pSecondBuff);
                        std::cout << "The sizes: " <<  buffer_with_frame_points[first].size() << " " <<  buffer_with_frame_points[second].size() << " " <<  buffer_with_frame_points[third].size() << std::endl;;
                        if ( diffSecondThird.x > max_pixel_shift_x_ || diffSecondThird.y > max_pixel_shift_y_) {
                            return;
                        } else {
                            if ( pThirdBuff.value == 0 ){
                                // delete point 
                                 removeEntity(buffer_with_frame_points[third], thirdBuff);
                            } 
                        }
                    }
                }
            }
            
        }
    }
    
}


void uvdar::alternativeHT::removeEntity(vectPoint3D & vec, size_t pos)
{
    std::cout << "REMOVING ENTITY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    vectPoint3D::iterator it = vec.begin();
    std::advance(it, pos);
    vec.erase(it);
}

