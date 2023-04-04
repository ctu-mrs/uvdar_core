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
        cv::Point max_px_shift;
        int max_zeros_consecutive;
        int max_ones_consecutive;
        int stored_seq_len_factor; // the multiplication factor how long the sequence should be for calculating the trajectory   
        int poly_order;
        double decay_factor;
        double conf_probab_percent;
        int threshold_values_len_for_poly_reg;
        int frame_tolerance;
        int allowed_BER_per_seq;
    };

    class alternativeHT {
    
    private:
        int global_count = 0;
        bool debug_, visual_debug_ = false;

        int gc=0; // TODO: Delete only for logging right now

        std::unique_ptr<loadedParamsForAHT> loaded_params_ = std::make_unique<loadedParamsForAHT>();
        
        double framerate_;
        const double prediction_margin_ = 0.0;
        
    
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
        bool setSequences(std::vector<std::vector<bool>>);
        void updateFramerate(double);

        void processBuffer(const mrs_msgs::ImagePointsWithFloatStampedConstPtr);

        std::vector<std::pair<std::vector<PointState>, int>> getResults();
        
    };    
} // namespace uvdar