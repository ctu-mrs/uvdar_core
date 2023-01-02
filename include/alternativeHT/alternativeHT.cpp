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
}

alternativeHT::~alternativeHT()
{}


std::vector<vectPoint3D> buf;

void alternativeHT::processBuffer(vectPoint3D & ptsCurrentFrame, const int buffer_cnt) {

    buffer_with_frame_points.at(buffer_cnt) = ptsCurrentFrame;
    std::cout << buffer_with_frame_points.at(buffer_cnt).size() << " Start " << std::endl;

    if (first_call_ && buffer_cnt == 0 ) {
            return;
    }
    
    int indexPrevFrame = -1; 
    if (buffer_cnt == 0 ){
        indexPrevFrame = buffer_size_ - 1; 
    } else if ( buffer_cnt <= buffer_size_){
        indexPrevFrame = buffer_cnt - 1; 
    } 
    vectPoint3D & ptsPrevFrame = buffer_with_frame_points.at(indexPrevFrame);

    findClosestAndLEDState( buffer_with_frame_points.at(buffer_cnt), ptsPrevFrame );

    // cleanUpBuffer(buffer_cnt);

    std::cout << " Size of Frame " << ptsCurrentFrame.size() << " Prev Frame " << ptsPrevFrame.size();

    buf.push_back(ptsPrevFrame);

    std::cout << std::endl;

    if ( buf.size() >= 15 ) {
        std::cout << "BUFFER VALUES " << std::endl;
        for (const auto r : buf ) {
            for (const auto k : r ) {
            std::cout << k.value << ", ";
            }
            std::cout << std::endl << "-----" << std::endl;
        }
        
        std::cout << "==============" << std::endl;
        buf.clear();
    }
}

void alternativeHT::findClosestAndLEDState(vectPoint3D & ptsCurrentImg, vectPoint3D & ptsPrevImg) {   

    for (auto & pPrevImg : ptsPrevImg) {
        bool nearestNeighbor = false;
        for (auto & pCurrentImg : ptsCurrentImg) {

            mrs_msgs::Point2DWithFloat diff = computeXYDifference(pCurrentImg, pPrevImg);
            // std::cout << " The pixel diff is " << diff.x << " " << diff.y << std::endl;



            if (diff.x > max_pixel_shift_x_ || diff.y > max_pixel_shift_y_) {
                nearestNeighbor = false; 
            }
            else {   
                // std::cout << "Match!" << std::endl;
                nearestNeighbor = true;
                pCurrentImg.value = 1; // match found - LED is "on"
                break;
            }
        }
        if (nearestNeighbor == true) break;
        if (nearestNeighbor == false) {
            insertEmptyPoint(ptsCurrentImg, pPrevImg);
        }
    }

    // for (const auto k : ptsPrevImg ) {
    //     std::cout << " value old " << k.value << std::endl;
    // }
    // for (const auto k : ptsCurrentImg ) {
    //     std::cout << " value new " << k.value << std::endl;
    // }
}    

mrs_msgs::Point2DWithFloat uvdar::alternativeHT::computeXYDifference(mrs_msgs::Point2DWithFloat first, mrs_msgs::Point2DWithFloat second){

    mrs_msgs::Point2DWithFloat p; 

    p.x = std::abs(first.x - second.x);
    p.y = std::abs(first.y - second.y);

    return p;

}

void alternativeHT::insertEmptyPoint(vectPoint3D &pointVector, const mrs_msgs::Point2DWithFloat point)
{   
    // std::cout << "Insert empty" << std::endl;
    mrs_msgs::Point2DWithFloat p;
    p = point;
    p.value = 0; // equals LED "off" state
    pointVector.push_back(p);
}

void alternativeHT::cleanUpBuffer(const int buffer_cnt_){

    if (first_call_) return; 

    int prevFrameIndex = buffer_cnt_ - 1; 
    int prevPrevFrameIndex = buffer_cnt_ - 2; 

    if (prevFrameIndex == 0 ) {
        prevPrevFrameIndex = buffer_size_ - 1; 
    }

    if ( buffer_cnt_ == 0 ){
        prevFrameIndex = buffer_size_ - 1; 
        prevPrevFrameIndex  = buffer_size_ - 2; 
    }

    // for ( auto & firstFramePoint : fast_buffer_.at(currFrameIndex).get_pts_reference() ) {
    //     for ( auto & secondFramePoint : fast_buffer_.at(prevFrameIndex).get_pts_reference() ){
            
    for ( int firstBuff = 0; firstBuff <  buffer_with_frame_points[buffer_cnt_].size(); firstBuff++ ) {
        for (int secondBuff = 0; secondBuff < buffer_with_frame_points[prevFrameIndex].size(); secondBuff++ ) {
            
            mrs_msgs::Point2DWithFloat pFirstBuff =  buffer_with_frame_points[buffer_cnt_][firstBuff];
            mrs_msgs::Point2DWithFloat pSecondBuff =  buffer_with_frame_points[buffer_cnt_][secondBuff];

            mrs_msgs::Point2DWithFloat diffFirstSecond = computeXYDifference(pFirstBuff, pSecondBuff); 

        
            if (diffFirstSecond.x > max_pixel_shift_x_ || diffFirstSecond.y > max_pixel_shift_y_) {
                return;
            } else {   
                if ( pFirstBuff.value == 0 && pSecondBuff.value == 0 ) {

                    for ( int thirdBuff = 0; thirdBuff < buffer_with_frame_points[prevPrevFrameIndex].size(); thirdBuff++ ) {
                        
                        mrs_msgs::Point2DWithFloat pThirdBuff = buffer_with_frame_points[prevPrevFrameIndex][thirdBuff];
                        mrs_msgs::Point2DWithFloat diffSecondThird = computeXYDifference(pThirdBuff, pSecondBuff);
                        std::cout << "The sizes: " <<  buffer_with_frame_points[buffer_cnt_].size() << " " <<  buffer_with_frame_points[prevFrameIndex].size() << " " <<  buffer_with_frame_points[prevPrevFrameIndex].size() << std::endl;;
                        if ( diffSecondThird.x > max_pixel_shift_x_ || diffSecondThird.y > max_pixel_shift_y_) {
                            return;
                        } else {
                            if ( pThirdBuff.value == 0 ){
                                // delete point 
                                 removeEntity(buffer_with_frame_points[prevPrevFrameIndex], thirdBuff);
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

