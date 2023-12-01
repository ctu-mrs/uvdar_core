#pragma once

#include "extended_search.h"
#include <uvdar_core/ImagePointsWithFloatStamped.h>
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

    // loaded params from the launch file and passed to the OMTA
    struct loadedParamsForOMTA{
        cv::Point max_px_shift;
        int max_zeros_consecutive;
        int max_ones_consecutive;
        int stored_seq_len_factor; // the multiplication factor how long the sequence should be for calculating the trajectory   
        int max_buffer_length; // number of accepted sequences in the buffer
        int poly_order;
        double decay_factor;
        double conf_probab_percent;
        int allowed_BER_per_seq;
    };

    class OMTA {
    
    private:
        bool debug_ = false;

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
         * @param current_frame vector of points in the current frame
         */
        void findClosestPixelAndInsert(std::vector<PointState>&);
        
        /**
         * @brief receives: sequences with no inserted points + points in current frame that were not inserted.
         * Calls selectStatisticsValues() and checks if point in current frame is in bounding box of the prediction. If it is inside bounding box the point is insert to the query sequence 
         * 
         * @param no_nn_current_frame vector of points in the current frame, with no nearest neighbour in the current sequences
         * @param sequences_no_insert vector of sequences with no new inserted points in the current frame
         */
        void expandedSearch(std::vector<PointState>& , std::vector<seqPointer>&);
        
        /**
         * @brief checks if it is allowed to insert a new "on"-point to the passed sequence
         * @param seq check this sequence
         * @return true if a new inserted point agrees with current settings for the sequence
         * @return false if new inserted point violates current settings for the sequence
         */
        bool checkSequenceValidityWithNewInsert(const seqPointer &);

        /**
         * @brief push the current point to the end of the sequence + delete first element if seq exceeds the wanted sequence length for the polynomial regression
         * @param sequence sequence where query point will be inserted
         * @param signal query point
         */
        void insertPointToSequence(std::vector<PointState> &, const PointState);

        /**
         * @brief insert "off"-point at the end of the sequence with current time with same position as last point in the sequence
         * @param seq seq where the "off" will be inserted 
         */
        void insertVPforSequencesWithNoInsert(seqPointer &);

        /**
         * @brief computes the expected prediction for a new appearing point by doing a polynomial regression and computing the prediction interval
         * @param values x or y coordinate
         * @param time insert time of the x or y coordinate
         * @param insert_time the current time stamp for which the next pixel should be inserted
         * @return PredictionStatistics 
         */
        PredictionStatistics selectStatisticsValues(const std::vector<double>&, const std::vector<double>&, const double&);

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
         * @param i_sequences vector of the sequences
         * @return false if the passed vector is empty or the max number of zeros is higher than the length of the sequence
         * @return true if false is not triggered
         */
        bool setSequences(std::vector<std::vector<bool>>);

        /**
         * @brief called by blink processor - inserts point to custom data structure + calls findClosestPixelAndInsert() and cleanPotentialBuffer()
         * @param points in mrs_msgs format
         */
        void processBuffer(const uvdar_core::ImagePointsWithFloatStampedConstPtr);

        /**
        * @brief compares the original sequences with the extracted ones.
        * @return returns the sequences with seq id to the bp_tim.cpp 
        */
        std::vector<std::pair<seqPointer, int>> getResults();
        
    };    
} // namespace uvdar