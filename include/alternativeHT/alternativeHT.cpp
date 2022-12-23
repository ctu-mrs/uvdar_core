#include "alternativeHT.h"
#include <iostream>

using namespace uvdar;

alternativeHT::alternativeHT(
    int i_buffer_size,
    int i_max_pixel_shift_x,
    int i_max_pixel_shift_y)
{
    buffer_size_        = i_buffer_size;
    max_pixel_shift_x_  = i_max_pixel_shift_x;
    max_pixel_shift_y_  = i_max_pixel_shift_y;

    for (int i = 0; i < buffer_size_; i++ ){
        vectPoint3D p; 
        fast_buffer_.push_back(p);
    }

}

alternativeHT::~alternativeHT()
{
}
    