#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <mrs_msgs/Point2DWithFloat.h>

namespace uvdar
{
    class alternativeHT
    {
    private:
        int buffer_size_;
        bool first_call_ = true; 
        
        const int max_pixel_shift_x_ = 3;
        const int max_pixel_shift_y_ = 3;

        // vector with points
        using vectPoint3D = std::vector<mrs_msgs::Point2DWithFloat>;

        // the buffer of size buffer_size_
        std::vector<vectPoint3D> fast_buffer_;

    public:
        alternativeHT( int i_buffer_size );
        ~alternativeHT();

        void processBuffer(std::vector<vectPoint3D> &, int );
        void findClosestAndLEDState(vectPoint3D &, vectPoint3D &);
        bool checkInsertVP(vectPoint3D &, vectPoint3D &);
        bool bothFramesEmpty(vectPoint3D , vectPoint3D );
        void insertEmptyPoint(vectPoint3D &, const mrs_msgs::Point2DWithFloat);

        void setFirstCallBool(bool i_first_call){
            first_call_ = i_first_call;
        }


    };
    

    
} // namespace uvdar

