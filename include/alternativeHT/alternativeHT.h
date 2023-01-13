#pragma once

#include <ros/console.h>

#include <opencv2/highgui/highgui.hpp>
#include <mrs_msgs/Point2DWithFloat.h>
#include "signal_matcher/signal_matcher.h"


namespace uvdar
{
    using vectPoint3D = std::vector<mrs_msgs::Point2DWithFloat>;
    using point3DWithIndex = std::pair<mrs_msgs::Point2DWithFloat, int>;
    using vectPoint3DWithIndex = std::vector<point3DWithIndex>;

    class alternativeHT {

    private:
        int buffer_size_;
        bool initialized_ = false;
        bool first_call_ = true;
        bool enable_manchester_ = false; 
        bool debug_, visual_debug_ = false;
        
        const int max_pixel_shift_x_ = 190;
        const int max_pixel_shift_y_ = 100;
        const int max_potentialSequence_length_ = 22;

        std::vector<vectPoint3D> buffer_with_frame_points;
        std::vector<vectPoint3DWithIndex> buffer_3DPoint_seqIndex_;

        std::vector<std::vector<bool>> originalSequences_;
        // std::vector<std::vector<bool>> potentialSequences_;
        std::vector<std::pair<std::vector<bool>,cv::Point2d>> potentialSequences_;

        int number_sequences_;
        std::unique_ptr<SignalMatcher> matcher_;

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
        void setDebugFlags(bool, bool);

        void setSequences(std::vector<std::vector<bool>>);

        void processBuffer(vectPoint3DWithIndex &, const int);

        void findClosestAndLEDState(vectPoint3DWithIndex &, vectPoint3DWithIndex &) ;
        void insertToSequencesBuffer(vectPoint3DWithIndex & );

        mrs_msgs::Point2DWithFloat computeXYDifference(mrs_msgs::Point2DWithFloat, mrs_msgs::Point2DWithFloat );
        void swapIndex( const int, const int, vectPoint3DWithIndex & );
        void insertVirtualPointAndUpdateIndices(vectPoint3DWithIndex &, const point3DWithIndex );
        int findMatch(std::vector<bool>&);
        std::vector<std::pair<cv::Point2d, int>> getResult();

    };

    // struct BlinkSignals{
    //         std::vector<std::vector<bool>> originalSequences_;
    //         std::vector<std::pair<std::vector<bool>,cv::Point2d>> potentialSequencesWithPoint_;
    //         std::vector<std::pair<cv::Point2d,int>> retrievedSignals_;
    // };

    

    
} // namespace uvdar

