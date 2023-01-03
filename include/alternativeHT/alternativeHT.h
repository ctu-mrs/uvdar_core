#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <mrs_msgs/Point2DWithFloat.h>

using vectPoint3D = std::vector<mrs_msgs::Point2DWithFloat>;
using point3DWithIndex = std::pair<mrs_msgs::Point2DWithFloat, int>;
using vectPoint3DWithIndex = std::vector<point3DWithIndex>;

namespace uvdar
{

       class alternativeHT
    {
    private:
        int buffer_size_;
        bool first_call_ = true;
        bool enable_manchester_ = false; 
        
        const int max_pixel_shift_x_ = 3;
        const int max_pixel_shift_y_ = 3;

        std::vector<vectPoint3D> buffer_with_frame_points;
        std::vector<vectPoint3DWithIndex> buffer_3DPoint_seqIndex;


        std::vector<mrs_msgs::Point2DWithFloat> sequences;

    public:
        alternativeHT( int i_buffer_size );
        ~alternativeHT();
        void setFirstCallBool(bool i_first_call){
            first_call_ = i_first_call;
        }
        void setManchesterBoolTrue(){
            enable_manchester_ = true;
        }

        void processBuffer(vectPoint3DWithIndex &, const int);
        // probably depricated 
        // void processBuffer(vectPoint3D &, const int );
        void findClosestAndLEDState(vectPoint3DWithIndex &, vectPoint3DWithIndex &) ;
        mrs_msgs::Point2DWithFloat computeXYDifference(mrs_msgs::Point2DWithFloat, mrs_msgs::Point2DWithFloat );
        void updateSequenceIndex( const int, const int, vectPoint3DWithIndex & );
        void insertVirtualPoint(vectPoint3DWithIndex &, const point3DWithIndex );

        std::tuple<int,int,int>  findCurrentIndexState(const int);
        void evalulateBuffer( const std::tuple<int,int,int> buffer_indices );

        void removeEntity(vectPoint3D &, size_t);
        


    };


    

    
} // namespace uvdar

