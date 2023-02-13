#pragma once

#include <iostream>
#include <map>
#include <list>
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
        ros::Time insertTime;
        cv::Point2d bbLeftUp; 
        cv::Point2d bbRightDown;
    };

    class alternativeHT {
    
    private:
    
        bool debug_, visual_debug_ = false;
        
        const int max_pixel_shift_x_ = 2;
        const int max_pixel_shift_y_ = 2;
        const int boundingBox_x_Size_ = 3;
        const int boundingBox_y_Size_ = 3;

        std::vector<std::vector<bool>> originalSequences_;
        
        std::mutex mutex_generatedSequences_;
        std::list<std::vector<PointState>> generatedSequences_;

        std::unique_ptr<SignalMatcher> matcher_;


        void findClosestPixelAndInsert(std::vector<PointState>&);

        cv::Point2d computeXYDiff(const cv::Point2d, const cv::Point2d);
        void insertPointToSequence(std::vector<PointState> &, const PointState &);

        void inserVPIfNoNewPointArrived(std::vector<PointState> &);
        void cleanPotentialBuffer();

        // moving average approach 
        void checkBoundingBoxIntersection(std::vector<PointState> &, std::vector<std::vector<PointState>*> &);
        bool checkValidWithPotentialNewPoint(const std::vector<PointState>&);
        bool bbIntersect(const PointState &, const PointState &);
        void computeHypothesisSets(const std::vector<PointState> &, std::vector<std::vector<PointState>*>);
        void calcVariance(std::vector<PointState> &);
        void permute( std::vector<PointState>, int, int, std::vector<std::vector<PointState>> &);


        // check intersection
        bool doIntersect(PointState p1, PointState q1, PointState p2, PointState q2);
        bool onSegment(PointState p, PointState q, PointState r);
        int orientation(PointState p, PointState q, PointState r);



        std::vector<PointState>* findClosestWithinSelectedBB(std::vector<std::vector<PointState>*>  , const PointState);
        
        int findSequenceMatch(std::vector<bool>);

    public:

        alternativeHT();
        ~alternativeHT();
        
        void setSequences(std::vector<std::vector<bool>>);
        void setDebugFlags(bool, bool);

        void processBuffer(const mrs_msgs::ImagePointsWithFloatStampedConstPtr);

        std::vector<std::pair<PointState, int>> getResults();
        
        template <typename T>
        void printVectorIfNotEmpty(T vect, std::string name) {
            if (vect.empty())return;
            std::cout << "The " << name << " is: ";
            for (const auto r : vect){
                if (r) std::cout << "1,";
                else std::cout << "0,";
            }
            std::cout << std::endl;
        }

    };    
} // namespace uvdar