#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Dense>

namespace uvdar_core::tracking::generalized {

/**
 * @brief Dense 2D covariance stored in row-major scalar form for ROS message parity.
 */
struct Covariance2D {
    double c00 = 1.0;
    double c01 = 0.0;
    double c10 = 0.0;
    double c11 = 1.0;

    Eigen::Matrix2d matrix() const
    {
        Eigen::Matrix2d covariance;
        covariance << c00, c01, c10, c11;
        return covariance;
    }

    static Covariance2D fromMatrix(const Eigen::Matrix2d& matrix)
    {
        Covariance2D covariance;
        covariance.c00 = matrix(0, 0);
        covariance.c01 = matrix(0, 1);
        covariance.c10 = matrix(1, 0);
        covariance.c11 = matrix(1, 1);
        return covariance;
    }
};

/**
 * @brief Detector point with its image-plane measurement uncertainty.
 */
struct ImagePoint {
    double x = 0.0;
    double y = 0.0;
    Covariance2D covariance;
};

/**
 * @brief Lightweight stamped list of detector points consumed by the tracker.
 */
struct ImagePointsWithCovariancesStamped {
    double stamp = 0.0;
    std::uint16_t img_height = 0;
    std::uint16_t img_width = 0;
    std::vector<ImagePoint> points;
};

/**
 * @brief Per-axis model prediction diagnostics.
 */
struct PredictionStats {
    double target_time = 0.0;
    double reference_time = 0.0;
    bool model_reg_computed = false;
    bool extended_search = false;
    std::vector<double> coeff;
    Eigen::VectorXd predicted_vals_past;
    double mean_independent = 0.0;
    double predicted_coordinate = 0.0;
    double prediction_variance = 1.0;
    double confidence_interval = 1.0;
};

/**
 * @brief Internal tracked point representation with uncertainty propagation.
 */
struct PointState {
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
 * @brief Public tracker result for one active sequence.
 */
struct TrackResult {
    PointState state;
    int id = -1;
    std::uint32_t track_id = 0;
};

} // namespace uvdar_core::tracking::generalized
