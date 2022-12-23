#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <mrs_msgs/Point2DWithFloat.h>

namespace uvdar
{
    class alternativeHT
    {
    private:
        int buffer_size_;
        int max_pixel_shift_x_;
        int max_pixel_shift_y_;

        using vectPoint3D = std::vector<mrs_msgs::Point2DWithFloat>;


        std::vector<vectPoint3D> fast_buffer_;

    public:
        alternativeHT(
            int i_buffer_size,
            int i_max_pixel_shift_x,
            int i_max_pixel_shift_y);
        ~alternativeHT();
    };
    

    
} // namespace uvdar

