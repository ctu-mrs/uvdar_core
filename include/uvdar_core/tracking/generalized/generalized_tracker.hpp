#pragma once

#include <memory>
#include <vector>

#include <boost/math/distributions/normal.hpp>
#include <boost/math/distributions/students_t.hpp>

#include "uvdar_core/tracking/blink_processor.hpp"
#include "uvdar_core/tracking/i_tracker.hpp"
#include "uvdar_core/tracking/signal_matcher.hpp"
#include "uvdar_core/tracking/types.hpp"

namespace uvdar_core::tracking::generalized {

using Covariance2D = uvdar_core::tracking::Covariance2D;
using ImagePoint = uvdar_core::tracking::ImagePoint;
using ImagePointsWithCovariancesStamped = uvdar_core::tracking::ImagePointsWithCovariancesStamped;
using PointState = uvdar_core::tracking::TrackState;
using PredictionStats = uvdar_core::tracking::PredictionStats;
using TrackResult = uvdar_core::tracking::TrackResult;

/**
 * @brief Generalized tracker tuning parameters.
 */
struct ParamsGeneralized {
    int allowed_BER_per_seq = 1;
    int stored_seq_len_factor = 3;
    int model_order = 3;
    double max_px_shift_x = 5.0;
    double max_px_shift_y = 5.0;
    int max_zeros_consecutive = 3;
    int max_buffer_length = 2000;
    double decay_factor = 0.01;
    double conf_probab_percent = 95.0;
    double association_gate_sigma = 3.0;
    double default_measurement_variance = 1.0;
    double process_noise_variance = 1.0;
    bool debug = false;

    /**
     * @brief Create parameters matching AMI defaults plus uncertainty gates.
     */
    static ParamsGeneralized create(bool debug = false)
    {
        ParamsGeneralized params;
        params.debug = debug;
        return params;
    }
};

/**
 * @brief Dynamic-buffer generalized tracker with propagated 2D uncertainty.
 */
class GeneralizedTracker : public uvdar_core::tracking::ITracker {
public:
    /**
     * @brief Construct tracker with selected generalized parameters.
     */
    explicit GeneralizedTracker(const ParamsGeneralized& params);
    /**
     * @brief Destroy tracker and release internal buffers.
     */
    ~GeneralizedTracker() override;

    /**
     * @brief Load blinking templates and reconfigure the ID matcher.
     */
    void setupSequenceMatcher(std::vector<std::vector<bool>> sequences) override;
    /**
     * @brief Process one timestamped frame of covariance-bearing detections.
     */
    void processBuffer(const ImagePointsWithCovariancesStamped& points) override;
    /**
     * @brief Retrieve matched tracks with propagated uncertainties.
     */
    std::vector<TrackResult> getResults() const override;

private:
    using SeqPointer = std::shared_ptr<std::vector<PointState>>;

    /**
     * @brief Associate detections to existing tracks with a covariance-aware local gate.
     */
    void localSearch(std::vector<PointState>& current_frame, std::vector<SeqPointer>& unmatched_sequences);
    /**
     * @brief Predict unmatched tracks and use the prediction covariance as the extended gate.
     */
    void extendedSearch(std::vector<PointState>& unmatched_points, std::vector<SeqPointer>& unmatched_sequences, double stamp);
    /**
     * @brief Remove stale tracks and enforce configured buffer limits.
     */
    void cleanPotentialBuffer();
    /**
     * @brief Append a real point and keep the per-track history bounded.
     */
    void addPointToSequenceAndCheckLength(std::vector<PointState>& sequence, const PointState& point);
    /**
     * @brief Append a predicted OFF-state when no real detection was associated.
     */
    void addVirtualPointToSequence(const SeqPointer& sequence, double stamp);
    /**
     * @brief Create a new track initialized by an unmatched detector point.
     */
    void startSequence(const PointState& point);
    /**
     * @brief Fit a weighted motion model for both axes at a requested timestamp.
     */
    bool predictSequence(const std::vector<PointState>& sequence, double target_time, PointState& prediction) const;
    /**
     * @brief Build one-axis weighted model prediction and variance.
     */
    PredictionStats selectStatisticsValues(
        const std::vector<double>& values,
        const std::vector<double>& times,
        const std::vector<double>& variances,
        double target_time) const;
    /**
     * @brief Compute exponentially decayed temporal weights.
     */
    std::vector<double> calcNormalizedWeightVect(const std::vector<double>& times) const;
    /**
     * @brief Mahalanobis distance squared between a point and a gate center.
     */
    double mahalanobisSquared(const Eigen::Vector2d& query, const Eigen::Vector2d& center, const Covariance2D& covariance) const;
    /**
     * @brief Symmetrize covariance and apply minimum diagonal variance.
     */
    Covariance2D regularizeCovariance(const Covariance2D& covariance) const;
    /**
     * @brief Add isotropic process noise to a covariance.
     */
    Covariance2D addProcessNoise(const Covariance2D& covariance) const;
    /**
     * @brief Combine independent 2D covariance estimates conservatively.
     */
    Covariance2D addCovariances(const Covariance2D& left, const Covariance2D& right) const;
    /**
     * @brief Keep only trailing samples used for blinking-sequence matching.
     */
    std::vector<PointState> processSequenceBasic(const SeqPointer& sequence, const std::vector<bool>& original_sequence) const;

    ParamsGeneralized params_;
    std::unique_ptr<uvdar_core::tracking::SignalMatcher> matcher_;
    std::vector<std::vector<bool>> sequences_;
    std::vector<SeqPointer> buffer_;
    std::vector<std::uint32_t> track_ids_;
    std::uint32_t next_track_id_ = 1;
};

struct BlinkProcessorTraits {
    using Params = ParamsGeneralized;
    using Tracker = GeneralizedTracker;

    static const char* name() { return "GeneralizedTracker"; }
};

using BlinkProcessor = uvdar_core::tracking::BlinkProcessor<BlinkProcessorTraits>;

} // namespace uvdar_core::tracking::generalized
