#include "alternativeHT.h"
#include <iostream>
#include <ros/ros.h>

using namespace uvdar;

alternativeHT::alternativeHT( int i_buffer_size ){
    
    buffer_size_ = i_buffer_size;
    
    for (int i = 0; i < buffer_size_; i++ ){
        Fast_Buffer f;
        fast_buffer_.push_back(f);
    }
}

alternativeHT::~alternativeHT()
{}

void alternativeHT::processBuffer(vectPoint3D & ptsCurrentFrame, const int buffer_cnt) {

    fast_buffer_.at(buffer_cnt).set_frame_pts(ptsCurrentFrame);

    if (first_call_) {
        if ( buffer_cnt == 0 ) {
            // if ( _debug_ ) ROS_INFO("[UVDAR_BP_Tim]: Buffer not filled with enough data.");
            return;
        }
        checkInsertVP(ptsCurrentFrame, fast_buffer_.at(buffer_cnt - 1).get_pts_reference() );
    }
    
    // select index for the previous frame dependent on the current buffer_cnt_ and if closest point can be computed/ virtual point is inserted
    int indexPrevFrame = -1;
    if (buffer_cnt == 0 ){
        indexPrevFrame = buffer_size_ - 1; 
    } else if ( buffer_cnt <= buffer_size_){
        indexPrevFrame = buffer_cnt - 1; 
    } 

    if (indexPrevFrame != -1 ) {
        if ( checkInsertVP(ptsCurrentFrame , fast_buffer_.at(indexPrevFrame).get_pts_reference() ) ) {
            return;
        }
        if ( bothFramesEmpty(ptsCurrentFrame, fast_buffer_.at(indexPrevFrame).get_pts_reference() ) ) {
             return; 
        }
    }

    findClosestAndLEDState( ptsCurrentFrame, fast_buffer_.at(indexPrevFrame).get_pts_reference());

    cleanUpBuffer(buffer_cnt);

}

bool alternativeHT::checkInsertVP(vectPoint3D &ptsCurrentImg, vectPoint3D &ptsPrevImg) {
    
    if (ptsCurrentImg.size() == 0 && ptsPrevImg.size() != 0){
        for ( auto & p : ptsPrevImg)
        {
            insertEmptyPoint(ptsCurrentImg, p);
            // p.value = 1;
        }
        return true; 
    }

    if (ptsPrevImg.size() == 0 && ptsCurrentImg.size() != 0)
    {        
        for ( auto & p : ptsCurrentImg)
        {  
            insertEmptyPoint(ptsPrevImg, p);
            // p.value = 1;
        }     
        return true; 
    }
    return false;
}

bool alternativeHT::bothFramesEmpty(vectPoint3D ptsCurrentImg, vectPoint3D ptsPrevImg) {

    if ( ptsCurrentImg.size () == 0 && ptsPrevImg.size()) return true;    
    return false;
}

void alternativeHT::findClosestAndLEDState(vectPoint3D & ptsCurrentImg, vectPoint3D & ptsPrevImg) {   


    std::cout << "Size of points old " << ptsPrevImg.size() << " Current " << ptsCurrentImg.size() << std::endl;

    for (auto & pointPrevImg : ptsPrevImg)
    {
        bool nearestNeighbor = true; // bool for predicting the LED state. Assumption: When in both images Points are existent. Some nearest neighbors will be found
        for (auto & pointCurrentImg : ptsCurrentImg)
        {

            double pixelShift_x = std::abs(pointCurrentImg.x - pointPrevImg.x);
            double pixelShift_y = std::abs(pointCurrentImg.y - pointPrevImg.y);

            if (pixelShift_x > max_pixel_shift_x_ || pixelShift_y > max_pixel_shift_y_) {
                nearestNeighbor = false;
                // if (_debug_){
                // std::cout << "Nearest Neighbor false!" << std::endl;
                // ROS_INFO("[UVDAR_BP_Tim]: Pixel shift is too big.");
                // }
            }
            else {   
                nearestNeighbor = true;
                pointCurrentImg.value = 1; // match found - LED is "on"
            }
        }

        // dangerous -> never exiting the for loop!
        if (!nearestNeighbor) { // no Match found! 
            // insert coordinate values of the current point into dummy point and set LED state to zero
            insertEmptyPoint(ptsCurrentImg, pointPrevImg);
        }
    }

    for (const auto k : ptsPrevImg ) {
        std::cout << " value old " << k.value << std::endl;
    }
    for (const auto k : ptsCurrentImg ) {
        std::cout << " value new " << k.value << std::endl;
    }
}    

void alternativeHT::insertEmptyPoint(vectPoint3D &pointVector, const mrs_msgs::Point2DWithFloat point)
{   
    std::cout << "Insert empty point" << std::endl;
    mrs_msgs::Point2DWithFloat p;
    p = point;
    p.value = 0; // equals LED "off" state
    pointVector.push_back(p);
}

void alternativeHT::cleanUpBuffer(const int buffer_cnt_){

    int currFrameIndex = buffer_cnt_;
    int prevFrameIndex = buffer_cnt_ - 1; 
    int prevPrevFrameIndex = buffer_cnt_ - 2; 

    if (prevFrameIndex == 0 ) {
        prevPrevFrameIndex = buffer_size_ - 1; 
    }

    if ( buffer_cnt_ == 0 ){
        prevFrameIndex = buffer_size_ - 1; 
        prevPrevFrameIndex  = buffer_size_ - 2; 
    }

    for ( auto & firstFramePoint : fast_buffer_.at(currFrameIndex).get_pts_reference() ) {
        for ( auto & secondFramePoint : fast_buffer_.at(prevFrameIndex).get_pts_reference() ){
            
            double pixelShift_x = std::abs(firstFramePoint.x - secondFramePoint.x);
            double pixelShift_y = std::abs(firstFramePoint.y - secondFramePoint.y);

            if (pixelShift_x > max_pixel_shift_x_ || pixelShift_y > max_pixel_shift_y_) {
                return;
            } else {   
                if ( firstFramePoint.value == 0 && secondFramePoint.value == 0 ) {
                    for ( auto & thirdFramePoint : fast_buffer_.at(prevPrevFrameIndex).get_pts_reference() ){
                        double pixelShift_x = std::abs(thirdFramePoint.x - secondFramePoint.x);
                        double pixelShift_y = std::abs(thirdFramePoint.y - secondFramePoint.y);

                        if (pixelShift_x > max_pixel_shift_x_ || pixelShift_y > max_pixel_shift_y_) {
                            return;
                        } else {
                            if ( thirdFramePoint.value == 0 ){
                                // delete point 
                                //  fast_buffer_.at(prevFrameIndex).().
                            } 
                        }
                    }
                }
            }
            
        }
    }
    

}
