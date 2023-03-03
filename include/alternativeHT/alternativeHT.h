#pragma once

#include "helpFunctions.h"
#include <iostream>
#include <list>
// #include <mutex>
#include <mrs_msgs/ImagePointsWithFloatStamped.h>
#include <fstream>

#include "signal_matcher/signal_matcher.h"



namespace uvdar
{



    class alternativeHT {
    
    private:
    
        bool debug_, visual_debug_ = false;
        
        const int max_pixel_shift_x_ = 2;
        const int max_pixel_shift_y_ = 2;
        const double predictionMargin_ = 0.3;
        const int size_for_savedSequences_ = 10; // the multiplication factor how long the sequence should be for calculating the trajectory   
        double framerate_;

        std::vector<std::vector<bool>> originalSequences_;
        
        std::mutex mutex_generatedSequences_;
        std::list<std::vector<PointState>> generatedSequences_;

        std::unique_ptr<SignalMatcher> matcher_;


        void findClosestPixelAndInsert(std::vector<PointState>&);
        cv::Point2d computeXYDiff(const cv::Point2d, const cv::Point2d);
        void insertPointToSequence(std::vector<PointState> &, const PointState);


        // moving average approach 
        void expandedSearch(std::vector<PointState> &, std::vector<seqPointer>);
        bool checkSequenceValidityWithNewInsert(const seqPointer &);
        bool checkCoeffValidity(const SeqWithTrajectory &);
        void calculatePredictionTriangle(SeqWithTrajectory &, const double);
        void findOrthogonalVector(cv::Point2d);
        void insertVPforSequencesWithNoInsert(seqPointer &);

        
        void cleanPotentialBuffer();
        int findSequenceMatch(std::vector<bool>);

    public:

        alternativeHT();
        ~alternativeHT();
        
        void setDebugFlags(bool, bool);
        void setSequences(std::vector<std::vector<bool>>);
        void updateFramerate(double);

        void processBuffer(const mrs_msgs::ImagePointsWithFloatStampedConstPtr);

        std::vector<std::pair<PointState, int>> getResults();
        
    };    
} // namespace uvdar