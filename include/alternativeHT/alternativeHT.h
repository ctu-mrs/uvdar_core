#pragma once

#include <iostream>
#include <map>
#include <ros/console.h>
// #include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <mrs_msgs/ImagePointsWithFloatStamped.h>
#include "signal_matcher/signal_matcher.h"



namespace uvdar
{

    struct PointState{
        cv::Point2d point;
        bool ledState; 
        int index;
        ros::Time insertTime;
    };

    struct sequenceWithPoint{
        std::vector<bool> seq;
        PointState lastInsertedPoint;
    };

    class alternativeHT {
    
    private:
        int buffer_size_;
        bool initialized_ = false;
        bool first_call_ = true;
        bool enable_manchester_ = false; 
        bool debug_, visual_debug_ = false;
        
        const int max_pixel_shift_x_ = 2;
        const int max_pixel_shift_y_ = 2;

        std::vector<std::vector<PointState>> buffer;

        std::vector<std::vector<bool>> originalSequences_;
        std::map<int, sequenceWithPoint> foundSequences_;

        std::unique_ptr<SignalMatcher> matcher_;

        std::mutex mutex_foundSequences_;
        std::mutex mutex_buffer;


    public:

        alternativeHT( int i_buffer_size );
        ~alternativeHT();
        void setFirstCallBool(bool i_first_call){
            first_call_ = i_first_call;
        }
        
        void initBuffer();
        void setDebugFlags(bool, bool);
        void setSequences(std::vector<std::vector<bool>>);

        void processBuffer(const mrs_msgs::ImagePointsWithFloatStampedConstPtr);

        void findClosestAndLEDState();
        cv::Point2d computeXYDiff(const cv::Point2d, const cv::Point2d);
        void insertPointToSequence(PointState);
        void insertVirtualPoint(const PointState);

        void checkIfThreeConsecutiveZeros();
        void cleanPotentialBuffer();

        int randomInt(const int);


        int findMatch(std::vector<bool>);
        std::vector<std::pair<PointState, int>> getResults();
        
        template <typename T>
        void printVectorIfNotEmpty(T vect, std::string name) {
            if (vect.empty())return;
            std::cout << "The " << name << " elements are: ";
            for (const auto r : vect){
                if (r) std::cout << "1,";
                else std::cout << "0,";
            }
            std::cout << std::endl;
        }

    };    
} // namespace uvdar

