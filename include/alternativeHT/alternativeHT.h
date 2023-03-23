#pragma once

#include "extended_search.h"
#include <mrs_msgs/ImagePointsWithFloatStamped.h>
#include "signal_matcher/signal_matcher.h"



namespace uvdar
{

    struct PointState{
        cv::Point2d point;
        bool led_state; 
        ros::Time insert_time;
        bool computed_extended_search = false;
        std::vector<double> x_coeff;
        std::vector<double> y_coeff;
        cv::Point2d ellipse;
        cv::Point2d predicted;
    };
    
    using seqPointer = std::shared_ptr<std::vector<PointState>>;


    class alternativeHT {
    
    private:
    
        bool debug_, visual_debug_ = false;
        
        const int max_pixel_shift_x_ = 2;
        const int max_pixel_shift_y_ = 2;
        const double prediction_margin_ = 0.0;
        const int size_for_saved_seqs_ = 10; // the multiplication factor how long the sequence should be for calculating the trajectory   
        double framerate_;
        int poly_order_; 

        std::vector<std::vector<bool>> original_sequences_;
        
        std::mutex mutex_gen_sequences_;
        std::vector<std::shared_ptr<std::vector<PointState>>> gen_sequences_;


        std::unique_ptr<SignalMatcher> matcher_;
        std::unique_ptr<ExtendedSearch> extended_search_;

        void findClosestPixelAndInsert(std::vector<PointState>&);
        cv::Point2d computeXYDiff(const cv::Point2d, const cv::Point2d);
        void insertPointToSequence(std::vector<PointState> &, const PointState);


        // moving average approach 
        void expandedSearch(std::vector<PointState>& , std::vector<seqPointer>&);
        PredictionStatistics selectStatisticsValues(const std::vector<double>&, const std::vector<double>&, const double&, const int &);

        bool checkSequenceValidityWithNewInsert(const seqPointer &);
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