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
        bool extended_search = false;
        std::vector<double> x_coeff;
        std::vector<double> y_coeff;
        cv::Point2d confidence_interval;
        cv::Point2d predicted;
    };
    
    using seqPointer = std::shared_ptr<std::vector<PointState>>;

    struct loadedParamsForAHT{
        cv::Point max_pixel_shift;
        bool communication_mode;
        int stored_seq_len_factor; 
        int poly_order;
        double decay_factor;
        double conf_probab_percent;
        int seq_overlap_probab_percent;
        int threshold_values_len_for_poly_reg;
        int frame_tolerance;
    };

    class alternativeHT {
    
    private:
        int global_count = 0;
        bool debug_, visual_debug_ = false;
        
        cv::Point max_pixel_shift_;
        
        const double prediction_margin_ = 0.0;
        int stored_seq_len_factor_; // the multiplication factor how long the sequence should be for calculating the trajectory   
        double framerate_;
        int poly_order_; 
        double conf_probab_percent_; // in percentage
        bool communication_mode_;
        int frame_tolerance_;
        int seq_overlap_probab_percent_;
        int threshold_values_len_for_poly_reg_;

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
        PredictionStatistics selectStatisticsValues(const std::vector<double>&, const std::vector<double>&, const double&, const int &, bool&);

        bool checkSequenceValidityWithNewInsert(const seqPointer &);
        void findOrthogonalVector(cv::Point2d);
        void insertVPforSequencesWithNoInsert(seqPointer &);

        void cleanPotentialBuffer();

    public:

        alternativeHT(const loadedParamsForAHT&);
        ~alternativeHT();
        
        void setDebugFlags(bool, bool);
        void setSequences(std::vector<std::vector<bool>>);
        void updateFramerate(double);

        void processBuffer(const mrs_msgs::ImagePointsWithFloatStampedConstPtr);

        std::vector<std::pair<std::vector<PointState>, int>> getResults();
        
    };    
} // namespace uvdar