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
        bool initialized_ = false;
        int buffer_size_;
        bool first_call_ = true;
        bool enable_manchester_ = false; 
        
        const int max_pixel_shift_x_ = 3;
        const int max_pixel_shift_y_ = 3;
        const int max_sequences_seen_ = 1000000; // the predefined max number of possible sequences 

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

        void initBuffer();
        void initSequenceBuffer();

        void processBuffer(vectPoint3DWithIndex &, const int);

        void findClosestAndLEDState(vectPoint3DWithIndex &, vectPoint3DWithIndex &) ;
        void insertToSequencesBuffer(vectPoint3DWithIndex & );

        mrs_msgs::Point2DWithFloat computeXYDifference(mrs_msgs::Point2DWithFloat, mrs_msgs::Point2DWithFloat );
        void swapIndex( const int, const int, vectPoint3DWithIndex & );
        void insertVirtualPointAndUpdateIndices(vectPoint3DWithIndex &, const point3DWithIndex );

    };


    

    
} // namespace uvdar

