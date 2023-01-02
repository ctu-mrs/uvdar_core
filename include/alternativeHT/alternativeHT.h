#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <mrs_msgs/Point2DWithFloat.h>

using vectPoint3D = std::vector<mrs_msgs::Point2DWithFloat>;

namespace uvdar
{

       class alternativeHT
    {
    private:
        int buffer_size_;
        bool first_call_ = true; 
        
        const int max_pixel_shift_x_ = 3;
        const int max_pixel_shift_y_ = 3;

        std::vector<vectPoint3D> buffer_with_frame_points;

    public:
        alternativeHT( int i_buffer_size );
        ~alternativeHT();
        void setFirstCallBool(bool i_first_call){
            first_call_ = i_first_call;
        }

        void processBuffer(vectPoint3D &, const int );
        void findClosestAndLEDState(vectPoint3D &, vectPoint3D &);
        void insertEmptyPoint(vectPoint3D &, const mrs_msgs::Point2DWithFloat);
        
        mrs_msgs::Point2DWithFloat computeXYDifference(mrs_msgs::Point2DWithFloat, mrs_msgs::Point2DWithFloat);

        void cleanUpBuffer(const int);

        void removeEntity(vectPoint3D &, size_t);



    };


    

    
} // namespace uvdar

