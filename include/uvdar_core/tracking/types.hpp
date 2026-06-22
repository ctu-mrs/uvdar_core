#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace uvdar_core::tracking {

/**
 * @brief Dense 2D covariance stored in scalar form for ROS message parity.
 */
struct Covariance2D {
    double c00 = 1.0;
    double c01 = 0.0;
    double c10 = 0.0;
    double c11 = 1.0;

    /**
     * @brief Convert scalar storage to a dense Eigen covariance matrix.
     */
    Eigen::Matrix2d matrix() const
    {
        Eigen::Matrix2d covariance;
        covariance << c00, c01, c10, c11;
        return covariance;
    }

    /**
     * @brief Store a dense Eigen covariance matrix in ROS-message scalar order.
     */
    static Covariance2D fromMatrix(const Eigen::Matrix2d& matrix)
    {
        return {matrix(0, 0), matrix(0, 1), matrix(1, 0), matrix(1, 1)};
    }
};

/**
 * @brief Detector point with image-plane measurement uncertainty.
 */
struct ImagePoint {
    double x = 0.0;
    double y = 0.0;
    Covariance2D covariance;
};

/**
 * @brief Timestamped list of detector points consumed by trackers.
 */
struct ImagePointsWithCovariancesStamped {
    double stamp = 0.0;
    std::uint16_t img_height = 0;
    std::uint16_t img_width = 0;
    std::vector<ImagePoint> points;
};

/**
 * @brief Shared parameter group for tracker helpers.
 */
struct DefaultTrackerParams {
    std::string sequence_file;
    bool debug = false;
    bool minimal_output = false;

    DefaultTrackerParams() = default;
    DefaultTrackerParams(std::string sequence_file_in, bool debug_in, bool minimal_output_in)
        : sequence_file(std::move(sequence_file_in))
        , debug(debug_in)
        , minimal_output(minimal_output_in)
    {
    }
};

/**
 * @brief Per-axis prediction diagnostics shared by tracker backends.
 */
struct PredictionStats {
    double target_time = 0.0;
    double reference_time = 0.0;
    bool model_reg_computed = false;
    bool extended_search = false;
    std::vector<double> coeff;
    Eigen::VectorXd predicted_vals_past;
    double mean_dependent = -1.0;
    double mean_independent = -1.0;
    double predicted_coordinate = -1.0;
    double prediction_variance = 1.0;
    double confidence_interval = -1.0;
};

/**
 * @brief Shared tracked blinker state returned by tracker backends.
 */
struct TrackState {
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    Eigen::Vector2d predicted_position = Eigen::Vector2d::Zero();
    Covariance2D covariance;
    Covariance2D measurement_covariance;
    Covariance2D prediction_covariance;
    bool led_state = false;
    bool virtual_point = false;
    bool associated_with_detection = false;
    double stamp = 0.0;
    PredictionStats x_statistics;
    PredictionStats y_statistics;
};

/**
 * @brief Public result for one active blinking-marker track.
 */
struct TrackResult {
    TrackState state;
    int id = -1;
    std::uint32_t track_id = 0;
};

} // namespace uvdar_core::tracking
