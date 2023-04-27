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

        PredictionStatistics x_statistics;
        PredictionStatistics y_statistics;
        
    };
    
    using seqPointer = std::shared_ptr<std::vector<PointState>>;

    struct loadedParamsForOMTA{
        cv::Point max_px_shift;
        int max_zeros_consecutive;
        int max_ones_consecutive;
        int stored_seq_len_factor; // the multiplication factor how long the sequence should be for calculating the trajectory   
        int poly_order;
        double decay_factor;
        double conf_probab_percent;
        int frame_tolerance;
        int allowed_BER_per_seq;
        double std_threshold_poly_reg;
    };

    class OMTA {
    
    private:
        bool debug_ = false;
        int gc=0; // TODO: Delete only for logging right now

        std::unique_ptr<loadedParamsForOMTA> loaded_params_ = std::make_unique<loadedParamsForOMTA>();

        double framerate_;
        const double prediction_margin_ = 0.0;
        std::vector<std::vector<bool>> original_sequences_;
        std::mutex mutex_gen_sequences_;
        std::vector<seqPointer> gen_sequences_;
        std::unique_ptr<SignalMatcher> matcher_;
        std::unique_ptr<ExtendedSearch> extended_search_;

        /**
         * @brief check if distance between the last point in the sequences and point in current frame is within the "max_px_shift" allowed distance. If yes, point in current frame is inserted otherwise point is pushed into vector for expandedSearch()
         */
        void findClosestPixelAndInsert(std::vector<PointState>&);
        
        /**
         * @brief receives: sequences with no inserted points + points in current frame that were not inserted.
         * Calls selectStatisticsValues() and checks if point in current frame is in bounding box of the prediction. If it is inside bounding box the point is insert to the query sequence 
         */
        void expandedSearch(std::vector<PointState>& , std::vector<seqPointer>&);
        
        /**
         * @brief checks if it is allowed to insert a new "on"-point to the passed sequence
         * @return true if a new inserted point agrees with current settings for the sequence
         * @return false if new inserted point violates current settings for the sequence
         */
        bool checkSequenceValidityWithNewInsert(const seqPointer &);

        /**
         * @brief push the current point to the end of the sequence + delete first element if seq exceeds the wanted sequence length for the polynomial regression
         */
        void insertPointToSequence(std::vector<PointState> &, const PointState);

        /**
         * @brief insert "off"-point at the end of the sequence with current time with same position as last point in the frame 
         */
        void insertVPforSequencesWithNoInsert(seqPointer &);

        /**
         * @brief computes the expected prediction for a new appearing point by doing a polynomial regression and computing the prediction interval
         * @return PredictionStatistics 
         */
        PredictionStatistics selectStatisticsValues(const std::vector<double>&, const std::vector<double>&, const double&, const int &);

        /**
         * @brief checks all sequences if one violates the current sequence settings or if the time since a new inserted bit is too long ago
         */
        void cleanPotentialBuffer();

    public:

        OMTA(const loadedParamsForOMTA&);
        ~OMTA();
        
        void setDebugFlags(bool);
        void updateFramerate(double);

        /**
         * @brief Set the Sequences for the SignalMatcher
         * 
         * @return false if the passed vector is empty or the max number of zeros is higher than the length of the sequence
         * @return true if false is not triggered
         */
        bool setSequences(std::vector<std::vector<bool>>);

        /**
         * @brief called by bp - inserts point to custom data structure + calls findClosestPixelAndInsert() and cleanPotentialBuffer()
         */
        void processBuffer(const mrs_msgs::ImagePointsWithFloatStampedConstPtr);

        /**
        * @brief compares the original sequences with the extracted ones.
        * @return returns the sequences with seq id to the bp_tim.cpp 
        */
        std::vector<std::pair<seqPointer, int>> getResults();
        
    };    
} // namespace uvdar