#include "alternativeHT.h"
#include <iostream>
#include <ros/ros.h>

using namespace uvdar;

alternativeHT::alternativeHT( int i_buffer_size ){
    buffer_size_        = i_buffer_size;


    // initialize data structure
    for (int i = 0; i < buffer_size_; i++ ){
        vectPoint3D p; 
        fast_buffer_.push_back(p);
    }

}

alternativeHT::~alternativeHT()
{
}



void alternativeHT::processBuffer(std::vector<vectPoint3D> & ptsBufferImg, int buffer_cnt)
{

    // std::cout << "P " << small_buffer_[img_index][buffer_cnt_].size() <<std::endl;
    // std::cout << "Psize " << ptsBufferImg[buffer_cnt_].size() <<std::endl;

    vectPoint3D & currFrame     = ptsBufferImg[buffer_cnt];
    // std::cout << "Size CURRFRAME" << currFrame.size() << std::endl;
    // vectPoint3D & previousFrame = ptsBufferImg[buffer_cnt_]; // default initializing - will be overridden
    if (first_call_) {
        if ( buffer_cnt == 0 ) {
            // if ( _debug_ ) ROS_INFO("[UVDAR_BP_Tim]: Buffer not filled with enough data.");
            return;
        }
        checkInsertVP(currFrame, ptsBufferImg[buffer_cnt - 1]);
    }
    
    // select index for the previous frame dependent on the current buffer_cnt_ and if closest point can be computed/ virtual point is inserted
    int indexPrevFrame = -1;
    if (buffer_cnt == 0 ){
        indexPrevFrame = buffer_size_ - 1; 
        if ( checkInsertVP(currFrame, ptsBufferImg[ indexPrevFrame ] ) ){
            return;
        } 
        if (bothFramesEmpty(currFrame, ptsBufferImg[ indexPrevFrame ] ) ){
            return;
        }
    } else if ( buffer_cnt <= buffer_size_){
        indexPrevFrame = buffer_cnt - 1; 
        if ( checkInsertVP(currFrame , ptsBufferImg[ indexPrevFrame ] ) ) {
            return;
        }
        if ( bothFramesEmpty(currFrame, ptsBufferImg[ indexPrevFrame ] ) ) {
            return;
        }

    } 

    findClosestAndLEDState( currFrame, ptsBufferImg [ indexPrevFrame ]);


}



void alternativeHT::findClosestAndLEDState(vectPoint3D & ptsCurrentImg, vectPoint3D & ptsPrevImg)
{   

    // std::cout << "Size start" << ptsCurrentImg.size() << " " << ptsOlderImg.size() << std::endl;

    bool nearestNeighbor = true; // bool for predicting the LED state. Assumption: When in both images Points are existent. Some nearest neighbors will be found
    // std::cout << "new in find" << ptsNewerImg.size() << std::endl;
    // std::cout << "old in find" << ptsOlderImg.size() << std::endl;
    
    std::cout << "Size of points old " << ptsPrevImg.size() << " Current " << ptsCurrentImg.size() << std::endl;

    for (auto & pointOlderImg : ptsPrevImg)
    {
        // pointOlderImg.value = 1; // set LED state to "on" because point exists
        for (auto & pointNewerImg : ptsCurrentImg)
        {

            double pixelShift_x = std::abs(pointNewerImg.x - pointOlderImg.x);
            double pixelShift_y = std::abs(pointNewerImg.y - pointOlderImg.y);

            // std::cout << "Pixel shift x " << pixelShift_x << " PixelShift y " << pixelShift_y <<  std::endl;

            if (pixelShift_x > max_pixel_shift_x_ || pixelShift_y > max_pixel_shift_y_)
            {
                nearestNeighbor = false;
                // if (_debug_){
                // std::cout << "Nearest Neighbor false!" << std::endl;
                // ROS_INFO("[UVDAR_BP_Tim]: Pixel shift is too big.");
                // }
            }
            else
            {   
                nearestNeighbor = true;
                pointNewerImg.value = 1; // match found - LED is "on"
            }
        }
        // dangerous -> never exiting the for loop!
        if (!nearestNeighbor) { // no Match found! 
            // insert coordinate values of the current point into dummy point and set LED state to zero
            insertEmptyPoint(ptsCurrentImg, pointOlderImg);
        }
    }

    for (const auto k : ptsPrevImg ) {
        std::cout << " value old " << k.value << std::endl;
    }
    for (const auto k : ptsCurrentImg ) {
        std::cout << " value new " << k.value << std::endl;
    }
}    



/**
 * @brief if one of the frames is empty, virtual point is inserted into the empty image
 * 
 * @param ptsCurrentImg 
 * @param ptsOlderImg 
 * @return true 
 * @return false 
 */
bool alternativeHT::checkInsertVP(vectPoint3D &ptsCurrentImg, vectPoint3D &ptsPrevImg){
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


bool alternativeHT::bothFramesEmpty(vectPoint3D ptsCurrentImg, vectPoint3D ptsPrevImg){

    if ( ptsCurrentImg.size () == 0 && ptsPrevImg.size()) return true;    

    return false;
}

/**
 * @brief insert empty point into point Vector
 *
 * @param pointVector
 * @param point
 */
void alternativeHT::insertEmptyPoint(vectPoint3D &pointVector, const mrs_msgs::Point2DWithFloat point)
{   
    std::cout << "Insert empty point" << std::endl;
    mrs_msgs::Point2DWithFloat p;
    p = point;
    p.value = 0; // equals LED "off" state
    pointVector.push_back(p);
}

