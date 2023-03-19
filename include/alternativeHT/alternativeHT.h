#pragma once

#include "extendedSearch.h"
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
        const double predictionMargin_ = 0.0;
        const int size_for_savedSequences_ = 10; // the multiplication factor how long the sequence should be for calculating the trajectory   
        double framerate_;
        int poly_order_; 

        std::vector<std::vector<bool>> originalSequences_;
        
        std::mutex mutex_generatedSequences_;
        // std::list<std::vector<PointState>> generatedSequences_;
        std::vector<std::shared_ptr<std::vector<PointState>>> gen_sequences_;


        std::unique_ptr<SignalMatcher> matcher_;
        std::unique_ptr<ExtendedSearch> extended_search_;

        void findClosestPixelAndInsert(std::vector<PointState>&);
        cv::Point2d computeXYDiff(const cv::Point2d, const cv::Point2d);
        void insertPointToSequence(std::vector<PointState> &, const PointState);


        // moving average approach 
        void expandedSearch(std::vector<PointState>& , std::vector<seqPointer>&);
        bool checkSequenceValidityWithNewInsert(const seqPointer &);
        void calculatePredictionTriangle(seqPointer, const double);
        void findOrthogonalVector(cv::Point2d);
        void insertVPforSequencesWithNoInsert(seqPointer &);

        
        void cleanPotentialBuffer();
        int findSequenceMatch(std::vector<bool>);

    public:

        alternativeHT(double, int);
        ~alternativeHT();
        
        void setDebugFlags(bool, bool);
        void setSequences(std::vector<std::vector<bool>>);
        void updateFramerate(double);

        void processBuffer(const mrs_msgs::ImagePointsWithFloatStampedConstPtr);

        std::vector<std::pair<std::vector<PointState>, int>> getResults();
        
    };    
} // namespace uvdar