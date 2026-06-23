#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Dense>
#include <boost/math/distributions/students_t.hpp>

#include "uvdar_core/tracking/blink_processor.hpp"
#include "uvdar_core/tracking/i_tracker.hpp"
#include "uvdar_core/tracking/signal_matcher.hpp"
#include "uvdar_core/tracking/types.hpp"

namespace uvdar_core::tracking::ami {

using Covariance2D = uvdar_core::tracking::Covariance2D;
using DefaultParams = uvdar_core::tracking::DefaultTrackerParams;
using ImagePoint = uvdar_core::tracking::ImagePoint;
using ImagePointsWithCovariancesStamped = uvdar_core::tracking::ImagePointsWithCovariancesStamped;
using PredictionStats = uvdar_core::tracking::PredictionStats;
using PointState = uvdar_core::tracking::TrackState;
using TrackResult = uvdar_core::tracking::TrackResult;

/**
 * @brief AMI algorithm tunable parameters.
 */
struct ParamsAMI : public DefaultParams {
    int allowed_BER_per_seq;
    int stored_seq_len_factor;
    int poly_order;
    double max_px_shift_x;
    double max_px_shift_y;
    int max_zeros_consecutive;
    int max_buffer_length;
    double decay_factor;
    double conf_probab_percent;

    /**
     * @brief AMI tuning parameter bundle for tracker instance.
     */
    ParamsAMI(
        std::string sequence_file,
        bool debug,
        bool minimal_output,
        int allowed_BER_per_seq,
        int stored_seq_len_factor,
        int poly_order,
        double max_px_shift_x,
        double max_px_shift_y,
        int max_zeros_consecutive,
        int max_buffer_length,
        double decay_factor,
        double conf_probab_percent)
        : DefaultParams(std::move(sequence_file), debug, minimal_output)
        , allowed_BER_per_seq(allowed_BER_per_seq)
        , stored_seq_len_factor(stored_seq_len_factor)
        , poly_order(poly_order)
        , max_px_shift_x(max_px_shift_x)
        , max_px_shift_y(max_px_shift_y)
        , max_zeros_consecutive(max_zeros_consecutive)
        , max_buffer_length(max_buffer_length)
        , decay_factor(decay_factor)
        , conf_probab_percent(conf_probab_percent)
    {
    }

    /**
     * @brief Create parameter struct with sane default AMI values.
     */
    static ParamsAMI create(std::string sequence_file, bool debug, bool minimal_output)
    {
        return ParamsAMI(
            std::move(sequence_file),
            debug,
            minimal_output,
            0,
            20,
            4,
            3.0,
            3.0,
            10,
            5000,
            0.1,
            95.0);
    }
};

using seqPointer = std::shared_ptr<std::vector<PointState>>;

/**
 * @brief Adaptive moving tracker matching AMI-like blinking sequences.
 */
class AMI : public uvdar_core::tracking::ITracker {
public:
    /**
     * @brief Construct tracker with selected AMI parameters.
     */
    explicit AMI(const ParamsAMI& params);
    /**
     * @brief Destroy tracker and release internal buffers.
     */
    ~AMI() override;

    /**
     * @brief Load blinking templates and reconfigure matcher.
     */
    void setupSequenceMatcher(std::vector<std::vector<bool>> sequences) override;
    /**
     * @brief Process one frame of point detections.
     */
    void processBuffer(const ImagePointsWithCovariancesStamped& points) override;
    /**
     * @brief Retrieve matched blinkers for current buffer state.
     */
    std::vector<TrackResult> getResults() const override;

private:
    /**
     * @brief Insert detections to closest existing trajectories and propagate unmatched ones.
     */
    void findClosestPixelAndInsert(std::vector<PointState>& current_frame, double stamp);
    /**
     * @brief Remove trajectories that have stayed unmatched for too long.
     */
    void cleanPotentialBuffer();
    /**
     * @brief Extend tracking with predictive model for unmatched trajectories.
     */
    void extendedSearch(std::vector<PointState>& no_nn_current_frame, std::vector<seqPointer>& sequences_no_insert);
    /**
     * @brief Check if point lies in a bounding box.
     */
    bool isInsideBox(const Eigen::Vector2d&, const Eigen::Vector2d&, const Eigen::Vector2d&) const;
    /**
     * @brief Compute Euclidean distance between two points.
     */
    double euclideanDistance(const Eigen::Vector2d&, const Eigen::Vector2d&) const;
    /**
     * @brief Append point and keep per-sequence buffer length bounded.
     */
    void addPointToSequenceAndCheckLength(std::vector<PointState>&, const PointState&);
    /**
     * @brief Append virtual zero point when no real point is associated.
     */
    void addVirtualPointToSequencesWithNoInsert(seqPointer&);
    /**
     * @brief Build statistics for coordinate prediction.
     */
    PredictionStats selectStatisticsValues(const std::vector<double>&, const std::vector<double>&, const double&);

    /**
     * @brief Weighted least squares sum of squared residuals.
     */
    double calcWSSR(const Eigen::VectorXd&, const std::vector<double>&, const std::vector<double>&) const;
    /**
     * @brief Fit weighted motion model and predict trajectory values.
     */
    std::tuple<std::vector<double>, Eigen::VectorXd> polyReg(const std::vector<double>&, const std::vector<double>&, const std::vector<double>&, int) const;
    /**
     * @brief Calculate time-dependent normalized exponential weights.
     */
    std::vector<double> calcNormalizedWeightVect(const std::vector<double>&) const;
    /**
     * @brief Compute weighted average with normalized weights.
     */
    double calcWeightedMean(const std::vector<double>&, const std::vector<double>&) const;
    /**
     * @brief Calculate confidence interval around prediction.
     */
    double confidenceInterval(const PredictionStats&, const std::vector<double>&, const std::vector<double>&, const std::vector<double>, const int&) const;
    /**
     * @brief Keep only trailing samples that match expected sequence length.
     */
    std::vector<PointState> processSequenceBasic(const seqPointer&, const std::vector<bool>&) const;

    ParamsAMI params_ami_;
    std::unique_ptr<uvdar_core::tracking::SignalMatcher> matcher_;
    std::vector<std::vector<bool>> sequences_;
    std::vector<seqPointer> buffer_;
};

struct BlinkProcessorTraits {
    using Params = ParamsAMI;
    using Tracker = AMI;

    static const char* name() { return "AMI"; }
};

using BlinkProcessor = uvdar_core::tracking::BlinkProcessor<BlinkProcessorTraits>;

} // namespace uvdar_core::tracking::ami
