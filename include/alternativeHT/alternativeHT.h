#pragma once

#include <iostream>
#include <ros/console.h>

#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <mrs_msgs/ImagePointsWithFloatStamped.h>
#include "signal_matcher/signal_matcher.h"


namespace uvdar
{

    struct BlinkSignal{
        cv::Point2d point;
        bool ledState; 
        int index;
        ros::Time insertTime;
    };

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
        
        
        const int max_pixel_shift_x_ = 2;
        const int max_pixel_shift_y_ = 2;

        std::vector<std::vector<BlinkSignal>> buffer;

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
            std::cout << "==============================================================" << std::endl;
            enable_manchester_ = true;
        }

        void initBuffer();
        void setDebugFlags(bool, bool);
        void setSequences(std::vector<std::vector<bool>>);

        void processBuffer(const mrs_msgs::ImagePointsWithFloatStampedConstPtr);

        void findClosestAndLEDState(std::vector<BlinkSignal> &, std::vector<BlinkSignal> &);
        cv::Point2d computeXYDiff(const cv::Point2d, const cv::Point2d);
        void insertPointToSequence(BlinkSignal);
        void insertVirtualPoint(std::vector<BlinkSignal> &, const BlinkSignal );

        void checkIfThreeConsecutiveZeros();
        void cleanPotentialBuffer();

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

