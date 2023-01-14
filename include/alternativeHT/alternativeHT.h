#pragma once

#include <iostream>
#include <ros/console.h>

#include <mutex>
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
        
        
        const int max_pixel_shift_x_ = 5;
        const int max_pixel_shift_y_ = 5;
        const int max_potentialSequence_length_ = 22;

        std::vector<vectPoint3D> buffer_with_frame_points;
        std::vector<vectPoint3DWithIndex> buffer_3DPoint_seqIndex_;

        std::vector<std::vector<bool>> originalSequences_;
        // std::vector<std::vector<bool>> potentialSequences_;
        std::vector<std::pair<std::vector<bool>,cv::Point2d>> potentialSequences_;
        std::mutex mutex_potSequence_;


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
        void insertToSequencesBuffer(vectPoint3DWithIndex);

        mrs_msgs::Point2DWithFloat computeXYDifference(mrs_msgs::Point2DWithFloat, mrs_msgs::Point2DWithFloat );
        void swapIndex( const int, const int, vectPoint3DWithIndex & );
        void insertVirtualPointAndUpdateIndices(vectPoint3DWithIndex &, const point3DWithIndex );
        int findMatch(std::vector<bool>);
        std::vector<std::pair<cv::Point2d, int>> getResult();
        
        template <typename T>
        void printVectorIfNotEmpty(T vect, std::string name) {
            if (vect.empty())return;
            std::cout << "The " << name << " elements are: " << std::endl;
            for (const auto r : vect){
                if (r) std::cout << "1,";
                else std::cout << "0,";
            }
            std::cout << std::endl;
        }

    };

    // struct BlinkSignals{
    //         std::vector<std::vector<bool>> originalSequences_;
    //         std::vector<std::pair<std::vector<bool>,cv::Point2d>> potentialSequencesWithPoint_;
    //         std::vector<std::pair<cv::Point2d,int>> retrievedSignals_;
    // };

    

    
} // namespace uvdar

