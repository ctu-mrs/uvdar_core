#include "alternativeHT.h"
#include <iostream>
#include <ros/ros.h>

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
    
}

alternativeHT::~alternativeHT()
{}
std::vector<vectPoint3DWithIndex> buf;  
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

    std::tuple<int,int,int> buffer_indices =  findCurrentIndexState(buffer_cnt);


    // evalulateBuffer(buffer_indices);

    std::cout << " Size of Frame " << ptsCurrentFrame.size() << " Prev Frame " << ptsPrevFrame.size();

    buf.push_back(ptsPrevFrame);

    std::cout << std::endl;

    if ( buf.size() >= 15 ) {
        std::cout << "BUFFER VALUES " << std::endl;
        for (const auto r : buf ) {
            for (const auto k : r ) {
            std::cout << k.first.value << ", ";
            }
            std::cout << std::endl << "-----" << std::endl;
        }
        
        std::cout << "==============" << std::endl;
        buf.clear();
    }
}


void alternativeHT::findClosestAndLEDState(vectPoint3DWithIndex & ptsCurrentImg, vectPoint3DWithIndex & ptsPrevImg) {   

    for ( size_t indexPrevImg = 0;  indexPrevImg < ptsPrevImg.size(); indexPrevImg++ ) {
        bool nearestNeighbor = false;
        for ( size_t indexCurrentImg = 0; indexCurrentImg < ptsCurrentImg.size(); indexCurrentImg++ ) {

            mrs_msgs::Point2DWithFloat diff = computeXYDifference(ptsCurrentImg[indexCurrentImg].first, ptsPrevImg[indexPrevImg].first);
            // std::cout << " The pixel diff is " << diff.x << " " << diff.y << std::endl;


            if (diff.x > max_pixel_shift_x_ || diff.y > max_pixel_shift_y_) {
                nearestNeighbor = false; 
            }
            else {   
                // std::cout << "Match!" << std::endl;
                nearestNeighbor = true;
                ptsCurrentImg[indexCurrentImg].first.value = 1; // match found - LED is "on"
                updateSequenceIndex( ptsPrevImg[indexPrevImg].second, indexCurrentImg, ptsCurrentImg );
                break;
            }
        }
        if (nearestNeighbor == true) break;
        if (nearestNeighbor == false) {
            insertVirtualPoint(ptsCurrentImg, ptsPrevImg[indexPrevImg]);
        }
    }


}    

mrs_msgs::Point2DWithFloat uvdar::alternativeHT::computeXYDifference(mrs_msgs::Point2DWithFloat first, mrs_msgs::Point2DWithFloat second){

    mrs_msgs::Point2DWithFloat p; 

    p.x = std::abs(first.x - second.x);
    p.y = std::abs(first.y - second.y);

    return p;

}

// not tested yet!! current implementation dangerous 
void alternativeHT::updateSequenceIndex( const int indexPrev, const int indexCurr ,vectPoint3DWithIndex & pointVector){
    
    if ( indexPrev == indexCurr ) return;

    for ( auto & p : pointVector ) {

        if ( p.second == indexPrev ) {
            p.second = pointVector[indexCurr].second;
            pointVector[indexCurr].second = indexPrev;
        }
    }
}
void alternativeHT::insertVirtualPoint(vectPoint3DWithIndex &pointVector, const point3DWithIndex point)
{   
    // std::cout << "Insert empty" << std::endl;
    point3DWithIndex p;
    p.first = point.first;
    p.first.value = 0; // equals LED "off" state
    
    p.second = point.second; 
    pointVector.push_back(p);
}

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

