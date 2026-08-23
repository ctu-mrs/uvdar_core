#include "uvdar_core/calibration/calibrator.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>

#include "uvdar_core/calibration/fisheye/equidistant_model.hpp"
#include "uvdar_core/calibration/fisheye/ocam_model.hpp"
#include "uvdar_core/calibration/fisheye/radial_model.hpp"
#include "uvdar_core/calibration/pinhole/pinhole_model.hpp"
#include "uvdar_core/helpers/levenberg_marquardt.hpp"
#include "uvdar_core/helpers/math.hpp"
#include "uvdar_core/helpers/polynomial.hpp"

namespace uvdar_core::calibration {

namespace {

constexpr double epsilon = 1.0e-12;
constexpr double pi = 3.14159265358979323846;

struct CalibrationState {
    Eigen::VectorXd camera;
    std::vector<CalibrationPose> poses;
};

struct ProjectionLinearization {
    Eigen::Vector2d pixel = Eigen::Vector2d::Zero();
    Eigen::Matrix<double, 2, 3> point_jacobian =
        Eigen::Matrix<double, 2, 3>::Zero();
    Eigen::MatrixXd camera_jacobian;
};

struct InitialEstimate {
    double fx = 1.0;
    double fy = 1.0;
    double cx = 0.0;
    double cy = 0.0;
    std::array<double, 5> distortion {};
    std::vector<CalibrationPose> poses;
};

bool isRadialModel(const CalibrationModelType model)
{
    return model == CalibrationModelType::FisheyeEquisolid
        || model == CalibrationModelType::FisheyeStereographic
        || model == CalibrationModelType::FisheyeOrthographic;
}

Eigen::Vector3d toEigen(const cv::Point3f& point)
{
    return {point.x, point.y, point.z};
}

double idealFisheyeRadius(
    const CalibrationModelType model, const double theta)
{
    switch (model) {
        case CalibrationModelType::FisheyeEquidistant:
            return theta;
        case CalibrationModelType::FisheyeEquisolid:
            return 2.0 * std::sin(0.5 * theta);
        case CalibrationModelType::FisheyeStereographic:
            return 2.0 * std::tan(0.5 * std::min(theta, pi - 1.0e-8));
        case CalibrationModelType::FisheyeOrthographic:
            return std::sin(theta);
        default:
            return theta;
    }
}

int cameraParameterCount(
    const CalibrationModelType model, const CalibratorOptions& options)
{
    switch (model) {
        case CalibrationModelType::Pinhole:
            return 9;
        case CalibrationModelType::FisheyeEquidistant:
        case CalibrationModelType::FisheyeEquisolid:
        case CalibrationModelType::FisheyeStereographic:
        case CalibrationModelType::FisheyeOrthographic:
            return 8;
        case CalibrationModelType::OcamCalib:
            return 5 + std::max(2, options.ocam_inverse_polynomial_order + 1);
    }
    throw std::runtime_error("Unknown calibration model.");
}

InitialEstimate initializePinhole(
    const std::vector<CalibrationObservation>& observations,
    const cv::Size& image_size)
{
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    object_points.reserve(observations.size());
    image_points.reserve(observations.size());
    for (const auto& observation : observations) {
        object_points.push_back(observation.object_points);
        image_points.push_back(observation.image_points);
    }

    const double initial_focal =
        0.8 * static_cast<double>(std::max(image_size.width, image_size.height));
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3)
        << initial_focal, 0.0, 0.5 * image_size.width,
        0.0, initial_focal, 0.5 * image_size.height,
        0.0, 0.0, 1.0);
    cv::Mat distortion = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rotation_vectors;
    std::vector<cv::Mat> translation_vectors;

    try {
        cv::calibrateCamera(
            object_points,
            image_points,
            image_size,
            camera_matrix,
            distortion,
            rotation_vectors,
            translation_vectors,
            cv::CALIB_USE_INTRINSIC_GUESS,
            cv::TermCriteria(
                cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                60,
                1.0e-10));
    } catch (const cv::Exception&) {
        // A conservative centered pinhole still provides solvePnP poses below.
        camera_matrix = (cv::Mat_<double>(3, 3)
            << initial_focal, 0.0, 0.5 * image_size.width,
            0.0, initial_focal, 0.5 * image_size.height,
            0.0, 0.0, 1.0);
        distortion = cv::Mat::zeros(8, 1, CV_64F);
        rotation_vectors.clear();
        translation_vectors.clear();
    }

    InitialEstimate output;
    output.fx = std::max(camera_matrix.at<double>(0, 0), 1.0);
    output.fy = std::max(camera_matrix.at<double>(1, 1), 1.0);
    output.cx = camera_matrix.at<double>(0, 2);
    output.cy = camera_matrix.at<double>(1, 2);
    const cv::Mat flat_distortion = distortion.reshape(1, 1);
    for (int index = 0;
         index < std::min<int>(5, static_cast<int>(flat_distortion.total()));
         ++index) {
        output.distortion[static_cast<std::size_t>(index)] =
            flat_distortion.at<double>(0, index);
    }

    output.poses.reserve(observations.size());
    for (std::size_t view = 0U; view < observations.size(); ++view) {
        cv::Mat rotation_vector;
        cv::Mat translation_vector;
        if (view < rotation_vectors.size()) {
            rotation_vector = rotation_vectors[view];
            translation_vector = translation_vectors[view];
        } else {
            rotation_vector = cv::Mat::zeros(3, 1, CV_64F);
            translation_vector = cv::Mat::zeros(3, 1, CV_64F);
            bool solved = false;
            try {
                solved = cv::solvePnP(
                    observations[view].object_points,
                    observations[view].image_points,
                    camera_matrix,
                    distortion,
                    rotation_vector,
                    translation_vector,
                    false,
                    cv::SOLVEPNP_ITERATIVE);
            } catch (const cv::Exception&) {
                solved = false;
            }
            if (!solved) {
                translation_vector.at<double>(2) =
                    std::max(1.0, 4.0 * observations[view].object_points.size());
            }
        }

        cv::Mat rotation_matrix;
        cv::Rodrigues(rotation_vector, rotation_matrix);
        CalibrationPose pose;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                pose.rotation(row, column) =
                    rotation_matrix.at<double>(row, column);
            }
            pose.translation(row) = translation_vector.at<double>(row);
        }
        output.poses.push_back(std::move(pose));
    }
    return output;
}

CalibrationState makeInitialState(
    const CalibrationModelType model,
    const CalibratorOptions& options,
    const InitialEstimate& initial)
{
    CalibrationState state;
    state.camera = Eigen::VectorXd::Zero(cameraParameterCount(model, options));
    state.poses = initial.poses;
    if (model == CalibrationModelType::Pinhole) {
        state.camera << std::log(initial.fx), std::log(initial.fy),
            initial.cx, initial.cy,
            initial.distortion[0], initial.distortion[1],
            initial.distortion[2], initial.distortion[3],
            initial.distortion[4];
    } else if (model != CalibrationModelType::OcamCalib) {
        state.camera.head<4>() << std::log(initial.fx), std::log(initial.fy),
            initial.cx, initial.cy;
    } else {
        // The polynomial angle is measured from the image plane. An
        // equidistant initial model is therefore rho=f*(theta+pi/2). The
        // three independent stretch entries start at the identity matrix;
        // fixing the lower-right entry to one establishes the scale gauge.
        state.camera.head<5>() << initial.cx, initial.cy,
            options.initial_stretch_matrix(0, 0),
            options.initial_stretch_matrix(0, 1),
            options.initial_stretch_matrix(1, 0);
        state.camera(5) = 0.5 * pi * 0.5 * (initial.fx + initial.fy);
        state.camera(6) = 0.5 * (initial.fx + initial.fy);
    }
    return state;
}

ProjectionLinearization pinholeProjection(
    const Eigen::VectorXd& camera,
    const Eigen::Vector3d& point,
    const bool with_jacobians)
{
    pinhole::PinholeModel::Parameters parameters;
    parameters.fx = std::exp(camera(0));
    parameters.fy = std::exp(camera(1));
    parameters.cx = camera(2);
    parameters.cy = camera(3);
    parameters.k1 = camera(4);
    parameters.k2 = camera(5);
    parameters.p1 = camera(6);
    parameters.p2 = camera(7);
    parameters.k3 = camera(8);
    const pinhole::PinholeModel model(parameters);

    ProjectionLinearization output;
    output.pixel = model.project(point);
    if (!with_jacobians) {
        return output;
    }
    output.point_jacobian = model.projectJacobian(point);
    output.camera_jacobian = Eigen::MatrixXd::Zero(2, camera.size());

    const double z = std::abs(point.z()) < epsilon
        ? std::copysign(epsilon, point.z() == 0.0 ? 1.0 : point.z())
        : point.z();
    const double x = point.x() / z;
    const double y = point.y() / z;
    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    output.camera_jacobian(0, 0) = output.pixel.x() - parameters.cx;
    output.camera_jacobian(1, 1) = output.pixel.y() - parameters.cy;
    output.camera_jacobian(0, 2) = 1.0;
    output.camera_jacobian(1, 3) = 1.0;
    output.camera_jacobian(0, 4) = parameters.fx * x * r2;
    output.camera_jacobian(1, 4) = parameters.fy * y * r2;
    output.camera_jacobian(0, 5) = parameters.fx * x * r4;
    output.camera_jacobian(1, 5) = parameters.fy * y * r4;
    output.camera_jacobian(0, 6) = parameters.fx * 2.0 * x * y;
    output.camera_jacobian(1, 6) = parameters.fy * (r2 + 2.0 * y * y);
    output.camera_jacobian(0, 7) = parameters.fx * (r2 + 2.0 * x * x);
    output.camera_jacobian(1, 7) = parameters.fy * 2.0 * x * y;
    output.camera_jacobian(0, 8) = parameters.fx * x * r6;
    output.camera_jacobian(1, 8) = parameters.fy * y * r6;
    return output;
}

ProjectionLinearization fisheyeProjection(
    const CalibrationModelType type,
    const Eigen::VectorXd& camera,
    const Eigen::Vector3d& point,
    const bool with_jacobians)
{
    const double fx = std::exp(camera(0));
    const double fy = std::exp(camera(1));
    const double cx = camera(2);
    const double cy = camera(3);
    ProjectionLinearization output;

    if (type == CalibrationModelType::FisheyeEquidistant) {
        fisheye::EquidistantModel::Parameters parameters;
        parameters.fx = fx;
        parameters.fy = fy;
        parameters.cx = cx;
        parameters.cy = cy;
        parameters.k1 = camera(4);
        parameters.k2 = camera(5);
        parameters.k3 = camera(6);
        parameters.k4 = camera(7);
        const fisheye::EquidistantModel model(parameters);
        output.pixel = model.project(point);
        if (with_jacobians) {
            output.point_jacobian = model.projectJacobian(point);
        }
    } else {
        fisheye::RadialModel::Parameters parameters;
        parameters.projection =
            type == CalibrationModelType::FisheyeStereographic
            ? fisheye::RadialModel::Projection::Stereographic
            : type == CalibrationModelType::FisheyeOrthographic
            ? fisheye::RadialModel::Projection::Orthographic
            : fisheye::RadialModel::Projection::EquisolidAngle;
        parameters.fx = fx;
        parameters.fy = fy;
        parameters.cx = cx;
        parameters.cy = cy;
        parameters.k1 = camera(4);
        parameters.k2 = camera(5);
        parameters.k3 = camera(6);
        parameters.k4 = camera(7);
        const fisheye::RadialModel model(parameters);
        output.pixel = model.project(point);
        if (with_jacobians) {
            output.point_jacobian = model.projectJacobian(point);
        }
    }

    if (!with_jacobians) {
        return output;
    }
    output.camera_jacobian = Eigen::MatrixXd::Zero(2, camera.size());
    output.camera_jacobian(0, 0) = output.pixel.x() - cx;
    output.camera_jacobian(1, 1) = output.pixel.y() - cy;
    output.camera_jacobian(0, 2) = 1.0;
    output.camera_jacobian(1, 3) = 1.0;

    const double xy_norm = std::hypot(point.x(), point.y());
    if (xy_norm > epsilon) {
        const double theta = std::atan2(xy_norm, point.z());
        const double radius = idealFisheyeRadius(type, theta);
        double power = radius * radius * radius;
        for (int coefficient = 0; coefficient < 4; ++coefficient) {
            const double x_derivative =
                fx * point.x() / xy_norm * power;
            const double y_derivative =
                fy * point.y() / xy_norm * power;
            output.camera_jacobian(0, 4 + coefficient) = x_derivative;
            output.camera_jacobian(1, 4 + coefficient) = y_derivative;
            power *= radius * radius;
        }
    }
    return output;
}

fisheye::OcamModel makeOcamModel(const Eigen::VectorXd& camera)
{
    fisheye::OcamModel model;
    model.xc = camera(1); // native row center = public y
    model.yc = camera(0); // native column center = public x
    model.c = camera(2);
    model.d = camera(3);
    model.e = camera(4);
    model.length_invpol = static_cast<int>(camera.size()) - 5;
    for (int index = 0; index < model.length_invpol; ++index) {
        model.invpol[static_cast<std::size_t>(index)] = camera(5 + index);
    }
    model.length_pol = 1;
    model.pol[0] = -1.0;
    return model;
}

ProjectionLinearization ocamProjection(
    const Eigen::VectorXd& camera,
    const Eigen::Vector3d& point,
    const bool with_jacobians)
{
    const fisheye::OcamModel model = makeOcamModel(camera);
    ProjectionLinearization output;
    output.pixel = model.project(point);
    if (!with_jacobians) {
        return output;
    }
    output.point_jacobian = model.projectJacobian(point);
    output.camera_jacobian = Eigen::MatrixXd::Zero(2, camera.size());
    output.camera_jacobian(0, 0) = 1.0;
    output.camera_jacobian(1, 1) = 1.0;

    const double radius = std::hypot(point.x(), point.y());
    if (radius <= epsilon) {
        return output;
    }
    const double theta = std::atan2(-point.z(), radius);
    const double rho = uvdar_core::helpers::evaluatePolynomialAscending(
        model.invpol.begin(),
        model.invpol.begin() + model.length_invpol,
        theta);
    const double qx = point.x() / radius * rho;
    const double qy = point.y() / radius * rho;
    output.camera_jacobian(1, 2) = qy;
    output.camera_jacobian(1, 3) = qx;
    output.camera_jacobian(0, 4) = qy;

    double theta_power = 1.0;
    for (int coefficient = 0; coefficient < model.length_invpol;
         ++coefficient) {
        output.camera_jacobian(0, 5 + coefficient) = theta_power
            * (point.x() + model.e * point.y()) / radius;
        output.camera_jacobian(1, 5 + coefficient) = theta_power
            * (model.d * point.x() + model.c * point.y()) / radius;
        theta_power *= theta;
    }
    return output;
}

ProjectionLinearization project(
    const CalibrationModelType model,
    const Eigen::VectorXd& camera,
    const Eigen::Vector3d& point,
    const bool with_jacobians)
{
    if (model == CalibrationModelType::Pinhole) {
        return pinholeProjection(camera, point, with_jacobians);
    }
    if (model == CalibrationModelType::OcamCalib) {
        return ocamProjection(camera, point, with_jacobians);
    }
    return fisheyeProjection(model, camera, point, with_jacobians);
}

std::pair<double, double> robustResidualScale(
    const Eigen::Vector2d& residual, const double huber_delta)
{
    const double norm = residual.norm();
    if (huber_delta <= 0.0 || norm <= huber_delta || norm <= epsilon) {
        return {1.0, 0.0};
    }
    const double rho = huber_delta * (norm - 0.5 * huber_delta);
    const double scale = std::sqrt(2.0 * rho) / norm;
    const double derivative = scale * 0.5
        * (huber_delta / rho - 2.0 / norm);
    return {scale, derivative};
}

bool validState(
    const CalibrationModelType model, const CalibrationState& state)
{
    if (!state.camera.allFinite()) {
        return false;
    }
    if (model != CalibrationModelType::OcamCalib
        && (state.camera(0) < std::log(1.0)
            || state.camera(0) > std::log(1.0e5)
            || state.camera(1) < std::log(1.0)
            || state.camera(1) > std::log(1.0e5))) {
        return false;
    }
    if (model == CalibrationModelType::OcamCalib
        && std::abs(state.camera(2)
               - state.camera(3) * state.camera(4))
            < 1.0e-3) {
        return false;
    }
    return std::all_of(
        state.poses.begin(), state.poses.end(), [](const auto& pose) {
            return pose.rotation.allFinite() && pose.translation.allFinite();
        });
}

std::optional<uvdar_core::helpers::LeastSquaresLinearization> linearize(
    const CalibrationModelType model,
    const CalibrationState& state,
    const std::vector<CalibrationObservation>& observations,
    const double huber_delta,
    const bool with_jacobian)
{
    if (!validState(model, state)
        || observations.size() != state.poses.size()) {
        return std::nullopt;
    }
    std::size_t point_count = 0U;
    for (const auto& observation : observations) {
        if (observation.image_points.size()
            != observation.object_points.size()) {
            return std::nullopt;
        }
        point_count += observation.image_points.size();
    }
    const Eigen::Index parameter_count = state.camera.size()
        + 6 * static_cast<Eigen::Index>(state.poses.size());
    uvdar_core::helpers::LeastSquaresLinearization output;
    output.residual.resize(2 * static_cast<Eigen::Index>(point_count));
    if (with_jacobian) {
        output.jacobian = Eigen::MatrixXd::Zero(
            output.residual.size(), parameter_count);
    }

    Eigen::Index row = 0;
    for (std::size_t view = 0U; view < observations.size(); ++view) {
        const CalibrationPose& pose = state.poses[view];
        for (std::size_t point_index = 0U;
             point_index < observations[view].image_points.size();
             ++point_index) {
            const Eigen::Vector3d object_point =
                toEigen(observations[view].object_points[point_index]);
            const Eigen::Vector3d rotated = pose.rotation * object_point;
            const Eigen::Vector3d camera_point =
                rotated + pose.translation;
            if (!camera_point.allFinite() || camera_point.norm() <= epsilon
                || (model == CalibrationModelType::Pinhole
                    && camera_point.z() <= epsilon)) {
                return std::nullopt;
            }
            const ProjectionLinearization projection = project(
                model, state.camera, camera_point, with_jacobian);
            const cv::Point2f& measured =
                observations[view].image_points[point_index];
            const Eigen::Vector2d raw_residual = projection.pixel
                - Eigen::Vector2d(measured.x, measured.y);
            if (!raw_residual.allFinite()) {
                return std::nullopt;
            }
            const auto [scale, scale_derivative] =
                robustResidualScale(raw_residual, huber_delta);
            output.residual.segment<2>(row) = scale * raw_residual;
            if (with_jacobian) {
                Eigen::Matrix2d robust_jacobian =
                    scale * Eigen::Matrix2d::Identity();
                if (raw_residual.norm() > epsilon) {
                    robust_jacobian += scale_derivative
                        / raw_residual.norm()
                        * raw_residual * raw_residual.transpose();
                }
                output.jacobian.block(
                    row, 0, 2, state.camera.size()) =
                    robust_jacobian * projection.camera_jacobian;
                Eigen::Matrix<double, 3, 6> point_pose_jacobian;
                point_pose_jacobian.leftCols<3>() =
                    -uvdar_core::helpers::skew(rotated);
                point_pose_jacobian.rightCols<3>() =
                    Eigen::Matrix3d::Identity();
                output.jacobian.block<2, 6>(
                    row,
                    state.camera.size()
                        + 6 * static_cast<Eigen::Index>(view)) =
                    robust_jacobian * projection.point_jacobian
                    * point_pose_jacobian;
            }
            row += 2;
        }
    }
    return output.residual.allFinite()
            && (!with_jacobian || output.jacobian.allFinite())
        ? std::optional<uvdar_core::helpers::LeastSquaresLinearization>(
              std::move(output))
        : std::nullopt;
}

CalibrationState retract(
    const CalibrationState& state, const Eigen::VectorXd& delta)
{
    CalibrationState candidate = state;
    candidate.camera += delta.head(state.camera.size());
    Eigen::Index offset = state.camera.size();
    for (CalibrationPose& pose : candidate.poses) {
        pose.rotation = uvdar_core::helpers::expSO3(
                            delta.segment<3>(offset))
            * pose.rotation;
        pose.translation += delta.segment<3>(offset + 3);
        offset += 6;
    }
    return candidate;
}

std::vector<double> perViewRms(
    const CalibrationModelType model,
    const CalibrationState& state,
    const std::vector<CalibrationObservation>& observations,
    std::vector<std::vector<cv::Point2f>>* projected = nullptr)
{
    std::vector<double> output;
    output.reserve(observations.size());
    if (projected) {
        projected->clear();
        projected->resize(observations.size());
    }
    for (std::size_t view = 0U; view < observations.size(); ++view) {
        double squared_error = 0.0;
        for (std::size_t point_index = 0U;
             point_index < observations[view].image_points.size();
             ++point_index) {
            const Eigen::Vector3d camera_point = state.poses[view].rotation
                    * toEigen(observations[view].object_points[point_index])
                + state.poses[view].translation;
            const Eigen::Vector2d pixel =
                project(model, state.camera, camera_point, false).pixel;
            const cv::Point2f& measured =
                observations[view].image_points[point_index];
            squared_error +=
                (pixel - Eigen::Vector2d(measured.x, measured.y))
                    .squaredNorm();
            if (projected) {
                (*projected)[view].emplace_back(
                    static_cast<float>(pixel.x()),
                    static_cast<float>(pixel.y()));
            }
        }
        output.push_back(std::sqrt(
            squared_error / std::max<std::size_t>(
                                1U, observations[view].image_points.size())));
    }
    return output;
}

double aggregateRms(
    const std::vector<double>& per_view,
    const std::vector<CalibrationObservation>& observations)
{
    double squared_error = 0.0;
    std::size_t point_count = 0U;
    for (std::size_t view = 0U; view < per_view.size(); ++view) {
        const std::size_t count = observations[view].image_points.size();
        squared_error += per_view[view] * per_view[view]
            * static_cast<double>(count);
        point_count += count;
    }
    return std::sqrt(
        squared_error / static_cast<double>(std::max<std::size_t>(1U, point_count)));
}

std::vector<double> fitOcamDirectPolynomial(
    const Eigen::VectorXd& camera,
    const std::vector<CalibrationObservation>& observations,
    const std::vector<CalibrationPose>& poses,
    const int requested_order)
{
    const int order = std::max(2, requested_order);
    double maximum_theta = -0.1;
    for (std::size_t view = 0U; view < observations.size(); ++view) {
        for (const cv::Point3f& object : observations[view].object_points) {
            const Eigen::Vector3d point =
                poses[view].rotation * toEigen(object)
                + poses[view].translation;
            maximum_theta = std::max(
                maximum_theta,
                std::atan2(-point.z(), std::hypot(point.x(), point.y())));
        }
    }
    maximum_theta = std::clamp(maximum_theta + 0.1, -0.1, 0.5 * pi - 0.05);
    constexpr int sample_count = 1000;
    std::vector<double> radii;
    std::vector<double> z_values;
    radii.reserve(sample_count);
    z_values.reserve(sample_count);
    for (int sample = 0; sample < sample_count; ++sample) {
        const double fraction = static_cast<double>(sample)
            / static_cast<double>(sample_count - 1);
        const double theta = -0.5 * pi + 1.0e-3
            + fraction * (maximum_theta + 0.5 * pi - 1.0e-3);
        const double rho = uvdar_core::helpers::evaluatePolynomialAscending(
            camera.data() + 5, camera.data() + camera.size(), theta);
        const double z = rho * std::tan(theta);
        if (std::isfinite(rho) && std::isfinite(z) && rho >= 0.0) {
            radii.push_back(rho);
            z_values.push_back(z);
        }
    }
    if (radii.size() < static_cast<std::size_t>(order + 1)) {
        return {-std::abs(camera(6)), 0.0};
    }
    const double radius_scale = std::max(
        1.0, *std::max_element(radii.begin(), radii.end()));
    // Match OCamCalib's conventional a1=0 constraint.
    Eigen::MatrixXd system(radii.size(), order);
    Eigen::VectorXd targets(radii.size());
    for (std::size_t row = 0U; row < radii.size(); ++row) {
        const double normalized = radii[row] / radius_scale;
        system(static_cast<Eigen::Index>(row), 0) = 1.0;
        double power = normalized * normalized;
        for (int column = 1; column < order; ++column) {
            system(static_cast<Eigen::Index>(row), column) = power;
            power *= normalized;
        }
        targets(static_cast<Eigen::Index>(row)) = z_values[row];
    }
    const Eigen::VectorXd fitted =
        system.colPivHouseholderQr().solve(targets);
    std::vector<double> coefficients(static_cast<std::size_t>(order + 1), 0.0);
    coefficients[0] = fitted(0);
    for (int power = 2; power <= order; ++power) {
        coefficients[static_cast<std::size_t>(power)] =
            fitted(power - 1) / std::pow(radius_scale, power);
    }
    return coefficients;
}

void fillResultParameters(
    CalibrationResult& result,
    const CalibrationState& state,
    const std::vector<CalibrationObservation>& observations,
    const CalibratorOptions& options)
{
    if (result.model == CalibrationModelType::Pinhole) {
        result.intrinsics = {
            std::exp(state.camera(0)), std::exp(state.camera(1)),
            state.camera(2), state.camera(3)};
        result.distortion.assign(
            state.camera.data() + 4, state.camera.data() + state.camera.size());
    } else if (result.model != CalibrationModelType::OcamCalib) {
        result.intrinsics = {
            std::exp(state.camera(0)), std::exp(state.camera(1)),
            state.camera(2), state.camera(3)};
        result.distortion.assign(
            state.camera.data() + 4, state.camera.data() + state.camera.size());
    } else {
        result.center = state.camera.head<2>();
        result.stretch_matrix << state.camera(2), state.camera(3),
            state.camera(4), 1.0;
        result.inverse_polynomial.assign(
            state.camera.data() + 5, state.camera.data() + state.camera.size());
        result.direct_polynomial = fitOcamDirectPolynomial(
            state.camera,
            observations,
            state.poses,
            options.ocam_direct_polynomial_order);
    }
}

void report(
    const CalibrationProgressCallback& callback,
    const CalibrationEngineStage stage,
    const std::string& detail,
    const int iteration = 0,
    const int maximum_iterations = 0,
    const int accepted_iterations = 0,
    const double rms = 0.0,
    const double cost = 0.0,
    const double damping = 0.0,
    const double elapsed_seconds = 0.0)
{
    if (callback) {
        callback(CalibrationProgress {
            stage,
            iteration,
            maximum_iterations,
            accepted_iterations,
            rms,
            cost,
            damping,
            detail,
            elapsed_seconds,
        });
    }
}

} // namespace

CalibrationModelType calibrationModelFromString(const std::string& name)
{
    if (name == "ocamcalib" || name == "ocam") {
        return CalibrationModelType::OcamCalib;
    }
    if (name == "pinhole") {
        return CalibrationModelType::Pinhole;
    }
    if (name == "fisheye_equidistant" || name == "equidistant") {
        return CalibrationModelType::FisheyeEquidistant;
    }
    if (name == "fisheye_equisolid" || name == "equisolid"
        || name == "equisolid_angle") {
        return CalibrationModelType::FisheyeEquisolid;
    }
    if (name == "fisheye_stereographic" || name == "stereographic") {
        return CalibrationModelType::FisheyeStereographic;
    }
    if (name == "fisheye_orthographic" || name == "orthographic") {
        return CalibrationModelType::FisheyeOrthographic;
    }
    throw std::invalid_argument("Unsupported calibration model '" + name + "'.");
}

std::string toString(const CalibrationModelType model)
{
    switch (model) {
        case CalibrationModelType::OcamCalib:
            return "ocamcalib";
        case CalibrationModelType::Pinhole:
            return "pinhole";
        case CalibrationModelType::FisheyeEquidistant:
            return "fisheye_equidistant";
        case CalibrationModelType::FisheyeEquisolid:
            return "fisheye_equisolid";
        case CalibrationModelType::FisheyeStereographic:
            return "fisheye_stereographic";
        case CalibrationModelType::FisheyeOrthographic:
            return "fisheye_orthographic";
    }
    return "unknown";
}

CameraCalibrator::CameraCalibrator(CalibratorOptions options)
    : options_(std::move(options))
{
    const double stretch_determinant =
        options_.initial_stretch_matrix.determinant();
    if (!options_.initial_stretch_matrix.allFinite()
        || std::abs(options_.initial_stretch_matrix(1, 1) - 1.0) > 1.0e-12
        || std::abs(stretch_determinant) < 1.0e-6) {
        throw std::invalid_argument(
            "The initial OCam stretch matrix must be finite, nonsingular, "
            "and have a lower-right entry of one.");
    }
}

CalibrationResult CameraCalibrator::calibrate(
    const std::vector<CalibrationObservation>& observations,
    const cv::Size& image_size,
    const CalibrationProgressCallback& progress) const
{
    const auto total_started_at = std::chrono::steady_clock::now();
    const auto elapsedSeconds = [&]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - total_started_at).count();
    };
    CalibrationResult result;
    result.model = options_.model;
    result.image_width = image_size.width;
    result.image_height = image_size.height;
    result.retained_views.assign(observations.size(), true);
    if (image_size.width <= 0 || image_size.height <= 0) {
        throw std::invalid_argument("Calibration image size must be positive.");
    }
    if (observations.size()
        < static_cast<std::size_t>(std::max(3, options_.minimum_views))) {
        throw std::invalid_argument("Not enough calibration views.");
    }
    for (const auto& observation : observations) {
        if (observation.image_points.size() < 4U
            || observation.image_points.size()
                != observation.object_points.size()) {
            throw std::invalid_argument(
                "Every calibration view must contain at least four matched points.");
        }
    }

    report(progress, CalibrationEngineStage::Initializing,
        "Estimating intrinsics and target poses");
    const auto initialization_started_at = std::chrono::steady_clock::now();
    const InitialEstimate initial =
        initializePinhole(observations, image_size);
    CalibrationState state =
        makeInitialState(options_.model, options_, initial);
    result.initialization_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - initialization_started_at).count();
    const Eigen::Index parameter_count = state.camera.size()
        + 6 * static_cast<Eigen::Index>(state.poses.size());

    uvdar_core::helpers::LevenbergMarquardtOptions lm_options;
    lm_options.max_iterations = std::max(1, options_.max_iterations);
    lm_options.initial_damping = std::max(options_.initial_damping, 1.0e-15);
    lm_options.step_tolerance = std::max(options_.step_tolerance, 0.0);
    lm_options.gradient_tolerance = std::max(options_.gradient_tolerance, 0.0);
    lm_options.relative_cost_tolerance =
        std::max(options_.relative_cost_tolerance, 0.0);
    lm_options.residual_tolerance = 0.0;
    report(progress, CalibrationEngineStage::Optimizing,
        "Jointly optimizing lens and all target poses", 0,
        lm_options.max_iterations, 0, 0.0, 0.0, 0.0, elapsedSeconds());
    const auto optimization_started_at = std::chrono::steady_clock::now();
    auto optimized = uvdar_core::helpers::levenbergMarquardt(
        state,
        parameter_count,
        [&](const CalibrationState& candidate, const bool with_jacobian) {
            return linearize(
                options_.model,
                candidate,
                observations,
                options_.huber_delta_px,
                with_jacobian);
        },
        retract,
        lm_options,
        [&](const uvdar_core::helpers::LevenbergMarquardtIteration& info) {
            const double displayed_cost =
                info.accepted ? info.candidate_cost : info.cost;
            std::size_t point_count = 0U;
            for (const auto& observation : observations) {
                point_count += observation.image_points.size();
            }
            report(progress, CalibrationEngineStage::Optimizing,
                info.accepted ? "Accepted LM step" : "Increasing damping",
                info.iteration + 1,
                lm_options.max_iterations,
                info.accepted_iterations,
                std::sqrt(2.0 * displayed_cost
                    / static_cast<double>(std::max<std::size_t>(1U, point_count))),
                displayed_cost,
                info.damping,
                elapsedSeconds());
        });
    state = optimized.state;
    result.iterations += optimized.iterations;
    result.optimization_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - optimization_started_at).count();

    report(progress, CalibrationEngineStage::RejectingOutliers,
        "Checking per-view reprojection error", 0, 0, 0, 0.0, 0.0, 0.0,
        elapsedSeconds());
    const auto refinement_started_at = std::chrono::steady_clock::now();
    std::vector<double> first_errors =
        perViewRms(options_.model, state, observations);
    std::vector<double> sorted_errors = first_errors;
    std::sort(sorted_errors.begin(), sorted_errors.end());
    const double median = sorted_errors[sorted_errors.size() / 2U];
    const double outlier_threshold = std::max(
        options_.huber_delta_px,
        options_.view_outlier_factor * std::max(median, 0.1));
    std::vector<std::size_t> kept_indices;
    for (std::size_t view = 0U; view < first_errors.size(); ++view) {
        if (first_errors[view] <= outlier_threshold) {
            kept_indices.push_back(view);
        } else {
            result.retained_views[view] = false;
        }
    }
    const std::size_t minimum_views = static_cast<std::size_t>(
        std::max(3, options_.minimum_views));
    if (kept_indices.size() < minimum_views) {
        std::vector<std::size_t> order(observations.size());
        std::iota(order.begin(), order.end(), 0U);
        std::sort(order.begin(), order.end(), [&](const auto first, const auto second) {
            return first_errors[first] < first_errors[second];
        });
        kept_indices.assign(order.begin(), order.begin() + minimum_views);
        result.retained_views.assign(observations.size(), false);
        for (const std::size_t index : kept_indices) {
            result.retained_views[index] = true;
        }
        std::sort(kept_indices.begin(), kept_indices.end());
    }

    std::vector<CalibrationObservation> retained_observations;
    CalibrationState retained_state;
    retained_state.camera = state.camera;
    retained_observations.reserve(kept_indices.size());
    retained_state.poses.reserve(kept_indices.size());
    for (const std::size_t index : kept_indices) {
        retained_observations.push_back(observations[index]);
        retained_state.poses.push_back(state.poses[index]);
    }

    if (options_.outlier_refinement_iterations > 0) {
        lm_options.max_iterations = options_.outlier_refinement_iterations;
        const Eigen::Index retained_parameter_count = retained_state.camera.size()
            + 6 * static_cast<Eigen::Index>(retained_state.poses.size());
        report(progress, CalibrationEngineStage::Refining,
            "Refining after per-view outlier rejection", 0,
            lm_options.max_iterations, 0, 0.0, 0.0, 0.0,
            elapsedSeconds());
        const auto refined = uvdar_core::helpers::levenbergMarquardt(
            retained_state,
            retained_parameter_count,
            [&](const CalibrationState& candidate, const bool with_jacobian) {
                return linearize(
                    options_.model,
                    candidate,
                    retained_observations,
                    options_.huber_delta_px,
                    with_jacobian);
            },
            retract,
            lm_options,
            [&](const uvdar_core::helpers::LevenbergMarquardtIteration& info) {
                report(progress, CalibrationEngineStage::Refining,
                    info.accepted ? "Accepted refinement step"
                                  : "Increasing damping",
                    info.iteration + 1,
                    lm_options.max_iterations,
                    info.accepted_iterations,
                    0.0,
                    info.accepted ? info.candidate_cost : info.cost,
                    info.damping,
                    elapsedSeconds());
            });
        retained_state = refined.state;
        result.iterations += refined.iterations;
    }
    result.outlier_refinement_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - refinement_started_at).count();

    report(progress, CalibrationEngineStage::Validating,
        "Computing final reprojection statistics", 0, 0, 0, 0.0, 0.0, 0.0,
        elapsedSeconds());
    const auto validation_started_at = std::chrono::steady_clock::now();
    std::vector<std::vector<cv::Point2f>> retained_projected;
    const std::vector<double> retained_errors = perViewRms(
        options_.model,
        retained_state,
        retained_observations,
        &retained_projected);
    result.rms_px = aggregateRms(retained_errors, retained_observations);
    result.poses = retained_state.poses;
    result.per_view_rms_px = first_errors;
    result.projected_points.resize(observations.size());
    for (std::size_t retained = 0U; retained < kept_indices.size(); ++retained) {
        const std::size_t original = kept_indices[retained];
        result.per_view_rms_px[original] = retained_errors[retained];
        result.projected_points[original] =
            std::move(retained_projected[retained]);
    }
    fillResultParameters(
        result, retained_state, retained_observations, options_);
    result.valid = std::isfinite(result.rms_px)
        && result.rms_px <= options_.maximum_rms_px
        && retained_observations.size() >= minimum_views;
    result.message = result.valid
        ? "Calibration converged"
        : "Final RMS exceeds maximum_rms_px";
    result.validation_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - validation_started_at).count();
    result.total_seconds = elapsedSeconds();
    report(progress, CalibrationEngineStage::Finished,
        result.message,
        result.iterations,
        result.iterations,
        result.iterations,
        result.rms_px,
        0.0,
        0.0,
        result.total_seconds);
    return result;
}

void writeCalibrationYaml(
    const CalibrationResult& result,
    const std::filesystem::path& output_path)
{
    if (!result.valid) {
        throw std::invalid_argument("Refusing to write an invalid calibration.");
    }
    if (output_path.empty() || output_path.filename().empty()) {
        throw std::invalid_argument("Calibration output path is empty.");
    }
    if (!output_path.parent_path().empty()) {
        std::filesystem::create_directories(output_path.parent_path());
    }

    YAML::Node root;
    root["calibration_model"] = toString(result.model);
    if (result.model == CalibrationModelType::OcamCalib) {
        root["direct_polynomial"] = result.direct_polynomial;
        root["inverse_polynomial"] = result.inverse_polynomial;
        root["center"] = std::vector<double> {
            result.center.y(), result.center.x()};
        YAML::Node stretch_matrix;
        stretch_matrix.push_back(std::vector<double> {
            result.stretch_matrix(0, 0), result.stretch_matrix(0, 1)});
        stretch_matrix.push_back(std::vector<double> {
            result.stretch_matrix(1, 0), result.stretch_matrix(1, 1)});
        root["stretch_matrix"] = stretch_matrix;
        root["affine"] = std::vector<double> {
            result.stretch_matrix(0, 0), result.stretch_matrix(0, 1),
            result.stretch_matrix(1, 0)};
        root["image_size"] = std::vector<int> {
            result.image_height, result.image_width};
    } else {
        root["intrinsics"] = result.intrinsics;
        root["distortion"] = result.distortion;
        root["image_width"] = result.image_width;
        root["image_height"] = result.image_height;
    }
    root["calibration_rms_px"] = result.rms_px;
    root["calibration_views"] = static_cast<int>(result.poses.size());
    YAML::Node timing;
    timing["initialization"] = result.initialization_seconds;
    timing["optimization"] = result.optimization_seconds;
    timing["outlier_refinement"] = result.outlier_refinement_seconds;
    timing["validation"] = result.validation_seconds;
    timing["total"] = result.total_seconds;
    root["calibration_timing_seconds"] = timing;

    YAML::Emitter emitter;
    emitter.SetDoublePrecision(16);
    emitter << root;
    if (!emitter.good()) {
        throw std::runtime_error("Failed to serialize calibration YAML.");
    }
    const std::filesystem::path temporary =
        output_path.string() + ".tmp";
    {
        std::ofstream stream(temporary, std::ios::trunc);
        if (!stream.is_open()) {
            throw std::runtime_error(
                "Could not open calibration output '" + temporary.string()
                + "'.");
        }
        stream << "# Generated by uvdar_core calibrator_node.\n";
        stream << emitter.c_str() << '\n';
        if (!stream.good()) {
            throw std::runtime_error(
                "Failed while writing calibration output '"
                + temporary.string() + "'.");
        }
    }
    std::filesystem::rename(temporary, output_path);
}

} // namespace uvdar_core::calibration
