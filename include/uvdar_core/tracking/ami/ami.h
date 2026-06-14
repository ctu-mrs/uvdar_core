#ifndef AMI_H
#define AMI_H

#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <boost/math/distributions/students_t.hpp>

#include "uvdar_core/tracking/ami/helper_functions.h"
#include "uvdar_core/tracking/ami/signal_matcher.h"

namespace uvdar_core::tracking::ami {

/**
 * @brief AMI algorithm tunable parameters.
 */
struct ParamsAMI : public DefaultParams {
    int allowed_BER_per_seq;
    int stored_seq_len_factor;
    int poly_order;
    Point2D max_px_shift;
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
        Point2D max_px_shift,
        int max_zeros_consecutive,
        int max_buffer_length,
        double decay_factor,
        double conf_probab_percent)
        : DefaultParams(std::move(sequence_file), debug, minimal_output)
        , allowed_BER_per_seq(allowed_BER_per_seq)
        , stored_seq_len_factor(stored_seq_len_factor)
        , poly_order(poly_order)
        , max_px_shift(max_px_shift)
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
            1,
            3,
            3,
            Point2D(5, 5),
            3,
            2000,
            0.01,
            95.0);
    }
};

/**
 * @brief Result prediction statistics for one coordinate axis.
 */
struct PredictionStats {
    double time_pred = -1;
    bool poly_reg_computed = false;
    bool extended_search = false;
    std::vector<double> coeff;
    Eigen::VectorXd predicted_vals_past;
    double mean_dependent = -1;
    double mean_independent = -1;
    double predicted_coordinate = -1;
    double confidence_interval = -1;
};

/**
 * @brief Internal tracked point representation.
 */
struct PointState {
    Point2D px_cord;
    bool led_state = false;
    double stamp = 0.0;
    PredictionStats x_statistics;
    PredictionStats y_statistics;
};

using seqPointer = std::shared_ptr<std::vector<PointState>>;

/**
 * @brief Adaptive moving tracker matching AMI-like blinking sequences.
 */
class AMI {
public:
    /**
     * @brief Construct tracker with selected AMI parameters.
     */
    explicit AMI(const ParamsAMI& params);
    /**
     * @brief Destroy tracker and release internal buffers.
     */
    ~AMI();

    /**
     * @brief Load blinking templates and reconfigure matcher.
     */
    void setupSequenceMatcher(std::vector<std::vector<bool>> sequences);
    /**
     * @brief Process one frame of point detections.
     */
    void processBuffer(const ImagePointsWithCovariancesStamped& points);
    /**
     * @brief Retrieve matched blinkers for current buffer state.
     */
    std::vector<std::pair<std::pair<PointState, int>, std::vector<int>>> getResults();

private:
    /**
     * @brief Insert detections to closest existing trajectories and propagate unmatched ones.
     */
    void findClosestPixelAndInsert(std::vector<PointState>& current_frame);
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
    bool isInsideBB(const Point2D&, const Point2D&, const Point2D&) const;
    /**
     * @brief Compute Euclidean distance between two points.
     */
    double euclideanDistance(const Point2D&, const Point2D&) const;
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
    std::unique_ptr<SignalMatcher> matcher_;
    std::vector<std::vector<bool>> sequences_;
    std::vector<seqPointer> buffer_;
};

} // namespace uvdar_core::tracking::ami

#endif // AMI_H
