#pragma once

#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace uvdar_core::calibration {

/** @brief Lens families available to the automatic calibrator and YAML loader. */
enum class CalibrationModelType {
    OcamCalib,
    Pinhole,
    FisheyeEquidistant,
    FisheyeEquisolid,
    FisheyeStereographic,
    FisheyeOrthographic,
};

CalibrationModelType calibrationModelFromString(const std::string& name);
std::string toString(CalibrationModelType model);

/** @brief One complete ordered observation of a planar calibration target. */
struct CalibrationObservation {
    std::vector<cv::Point2f> image_points;
    std::vector<cv::Point3f> object_points;
};

/** @brief Camera pose used for each target observation. */
struct CalibrationPose {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

/** @brief Numerical and model controls for multi-view bundle adjustment. */
struct CalibratorOptions {
    CalibrationModelType model = CalibrationModelType::OcamCalib;
    int minimum_views = 10;
    int max_iterations = 100;
    int outlier_refinement_iterations = 50;
    int ocam_inverse_polynomial_order = 9;
    int ocam_direct_polynomial_order = 4;
    /** @brief OCam initialization [[c,d],[e,1]]; identity is used by default. */
    Eigen::Matrix2d initial_stretch_matrix = Eigen::Matrix2d::Identity();
    double initial_damping = 1.0e-4;
    double huber_delta_px = 3.0;
    double view_outlier_factor = 2.5;
    double maximum_rms_px = 3.0;
    double step_tolerance = 1.0e-9;
    double gradient_tolerance = 1.0e-8;
    double relative_cost_tolerance = 1.0e-10;
};

/** @brief Coarse stages reported while the worker calibrates. */
enum class CalibrationEngineStage {
    Initializing,
    Optimizing,
    RejectingOutliers,
    Refining,
    Validating,
    Finished,
};

/** @brief Live progress values consumed by the ROS visualization. */
struct CalibrationProgress {
    CalibrationEngineStage stage = CalibrationEngineStage::Initializing;
    int iteration = 0;
    int maximum_iterations = 0;
    int accepted_iterations = 0;
    double rms_px = 0.0;
    double cost = 0.0;
    double damping = 0.0;
    std::string detail;
    double elapsed_seconds = 0.0;
};

using CalibrationProgressCallback =
    std::function<void(const CalibrationProgress&)>;

/** @brief Fully optimized calibration in the YAML loader's native schema. */
struct CalibrationResult {
    bool valid = false;
    CalibrationModelType model = CalibrationModelType::OcamCalib;
    int image_width = 0;
    int image_height = 0;
    std::vector<double> intrinsics;
    std::vector<double> distortion;
    std::vector<double> direct_polynomial;
    std::vector<double> inverse_polynomial;
    // Public pixel order [x/column, y/row]. The YAML writer swaps this for the
    // polynomial model's native [row, column] center convention.
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    Eigen::Matrix2d stretch_matrix = Eigen::Matrix2d::Identity();
    std::vector<CalibrationPose> poses;
    std::vector<bool> retained_views;
    std::vector<double> per_view_rms_px;
    std::vector<std::vector<cv::Point2f>> projected_points;
    double rms_px = 0.0;
    int iterations = 0;
    double initialization_seconds = 0.0;
    double optimization_seconds = 0.0;
    double outlier_refinement_seconds = 0.0;
    double validation_seconds = 0.0;
    double total_seconds = 0.0;
    std::string message;
};

/**
 * @brief Multi-model planar camera calibration with analytic bundle Jacobians.
 *
 * OpenCV is used only to produce a stable linear/pinhole initialization.  All
 * selected camera families then use the same shared analytic LM backend.
 */
class CameraCalibrator {
public:
    explicit CameraCalibrator(CalibratorOptions options = {});

    CalibrationResult calibrate(
        const std::vector<CalibrationObservation>& observations,
        const cv::Size& image_size,
        const CalibrationProgressCallback& progress = {}) const;

private:
    CalibratorOptions options_;
};

/** @brief Atomically write a calibration YAML accepted by loadLensModel(). */
void writeCalibrationYaml(
    const CalibrationResult& result,
    const std::filesystem::path& output_path);

} // namespace uvdar_core::calibration
