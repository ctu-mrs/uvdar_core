#include "uvdar_core/pose_estimation/kf_pose.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace uvdar_core::pose_estimation {

namespace {

bool hasNan(const Eigen::MatrixXd& matrix)
{
    return matrix.array().isNaN().any();
}

} // namespace

KfPose::KfPose(KfPoseConfig config)
    : config_(std::move(config))
{
    // Use lower process noise for indoor motion and larger position noise when
    // odometry is unavailable.
    if (config_.indoor) {
        vl_ = 1.0;
        vv_ = 0.5;
    }
    if (!config_.odometry_available) {
        sn_ = 4.0;
    }
}

void KfPose::applyMeasurements(const std::vector<KfPoseMeasurement>& measurements)
{
    if (measurements.empty()) {
        return;
    }
    if (config_.anonymous_measurements) {
        applyMeasurementsAnonymous(measurements);
    } else {
        applyMeasurementsWithIdentity(measurements);
    }
}

void KfPose::spin(double now)
{
    removeNans();
    if (config_.anonymous_measurements) {
        // Anonymous tracks can duplicate the same physical target; overlap
        // removal keeps only the more reliable Gaussian.
        removeOverlaps();
    }

    for (std::size_t i = 0; i < states_.size();) {
        const double age = now - states_[i].latest_measurement;
        const double decay_age = states_[i].state.update_count > config_.min_measurements_to_validation
            ? config_.decay_age_normal
            : config_.decay_age_unvalidated;
        if (age > decay_age) {
            states_.erase(states_.begin() + static_cast<long>(i));
            continue;
        }
        // Standard linear prediction to the current publication time.
        predictTillTime(states_[i], now, true);
        ++i;
    }
}

std::vector<KfPoseState> KfPose::validatedStates() const
{
    std::vector<KfPoseState> output;
    for (const auto& state : states_) {
        if (state.state.update_count >= config_.min_measurements_to_validation) {
            output.push_back(state.state);
        }
    }
    return output;
}

std::vector<KfPoseState> KfPose::tentativeStates() const
{
    std::vector<KfPoseState> output;
    for (const auto& state : states_) {
        if (state.state.update_count < config_.min_measurements_to_validation) {
            output.push_back(state.state);
        }
    }
    return output;
}

void KfPose::initiateNew(const KfPoseMeasurement& measurement, int id)
{
    if (states_.size() > 20U) {
        return;
    }

    KfPoseMeasurement local = measurement;
    // Guard anonymous long-range initializations whose covariance is larger
    // than the observed range by pulling them to a conservative ray.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(local.covariance.topLeftCorner<3, 3>());
    if (solver.info() == Eigen::Success) {
        Eigen::Vector3d eigenvalues = solver.eigenvalues();
        bool changed = false;
        for (int i = 0; i < 3; ++i) {
            if (eigenvalues(i) > local.x.head<3>().norm()) {
                eigenvalues(i) = 5.0;
                changed = true;
            }
        }
        if (changed && id < 0) {
            local.covariance.topLeftCorner<3, 3>() = solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
            local.x.head<3>() = local.x.head<3>().normalized() * 15.0;
        }
    }

    KfPoseState state;
    state.id = id < 0 ? next_id_++ : id;
    state.covariance = local.covariance;
    state.covariance.topRightCorner(3, 3).setZero();
    state.covariance.bottomLeftCorner(3, 3).setZero();

    if (config_.use_velocity && !config_.anonymous_measurements) {
        // Velocity matrices are available, but velocity states cannot be
        // initialized from pose-only measurements.
        return;
    }
    state.x = local.x;

    states_.push_back({state, local.stamp, local.stamp});
}

void KfPose::applyMeasurementsAnonymous(const std::vector<KfPoseMeasurement>& measurements)
{
    if (states_.empty()) {
        for (const auto& measurement : measurements) {
            initiateNew(measurement, -1);
        }
        return;
    }

    // Match score is the peak of the product of two Gaussian position PDFs.
    Eigen::MatrixXd match_matrix(measurements.size(), states_.size());
    std::vector<std::vector<FilterData>> tentative(measurements.size());
    for (std::size_t m = 0; m < measurements.size(); ++m) {
        for (std::size_t s = 0; s < states_.size(); ++s) {
            tentative[m].push_back(states_[s]);
            double match_level = 0.0;
            correctWithMeasurement(tentative[m].back(), measurements[m], match_level, true, true);
            constexpr double camera_burst_padding = 0.1;
            if (measurements[m].receipt_stamp > 0.0
                && measurements[m].receipt_stamp - states_[s].latest_measurement < camera_burst_padding) {
                tentative[m].back().state = predictTillTime(
                    tentative[m].back(),
                    measurements[m].receipt_stamp + camera_burst_padding,
                    false);
            }
            match_matrix(static_cast<int>(m), static_cast<int>(s)) = match_level;
        }
    }

    std::vector<std::pair<int, int>> matches;
    for (int s = 0; s < static_cast<int>(states_.size()); ++s) {
        int best_index = -1;
        double best_match_level = -1.0;
        int best_update_count = -1;
        for (int m = 0; m < static_cast<int>(measurements.size()); ++m) {
            const double match_level = match_matrix(m, s);
            if (match_level > config_.match_level_threshold_associate
                && (states_[static_cast<std::size_t>(s)].state.update_count > best_update_count
                    || (states_[static_cast<std::size_t>(s)].state.update_count == best_update_count && match_level > best_match_level))) {
                best_index = m;
                best_match_level = match_level;
                best_update_count = states_[static_cast<std::size_t>(s)].state.update_count;
            }
        }
        if (best_index >= 0) {
            matches.emplace_back(best_index, s);
            for (int other = 0; other < static_cast<int>(states_.size()); ++other) {
                match_matrix(best_index, other) = std::numeric_limits<double>::quiet_NaN();
            }
        }
        for (int m = 0; m < static_cast<int>(measurements.size()); ++m) {
            if (match_matrix(m, s) > config_.match_level_threshold_associate) {
                match_matrix(m, s) = std::numeric_limits<double>::quiet_NaN();
            }
        }
    }

    for (const auto& [measurement_index, state_index] : matches) {
        states_[static_cast<std::size_t>(state_index)] = tentative[static_cast<std::size_t>(measurement_index)][static_cast<std::size_t>(state_index)];
    }

    const int original_size = static_cast<int>(states_.size());
    for (int s = 0; s < original_size; ++s) {
        for (int m = 0; m < static_cast<int>(measurements.size()); ++m) {
            if (!std::isnan(match_matrix(m, s))) {
                initiateNew(measurements[static_cast<std::size_t>(m)], -1);
                for (int other = 0; other < original_size; ++other) {
                    match_matrix(m, other) = std::numeric_limits<double>::quiet_NaN();
                }
            }
        }
    }
}

void KfPose::applyMeasurementsWithIdentity(const std::vector<KfPoseMeasurement>& measurements)
{
    for (const auto& measurement : measurements) {
        const int id = measurement.id % 1000;
        auto target = std::find_if(states_.begin(), states_.end(), [&](const FilterData& state) {
            return state.state.id == id;
        });
        if (target == states_.end()) {
            initiateNew(measurement, id);
        } else {
            double match_level = 0.0;
            correctWithMeasurement(*target, measurement, match_level, true, true);
        }
    }
}

KfPoseState KfPose::predictTillTime(FilterData& data, double target_time, bool apply_update)
{
    const double dt = std::max(0.0, std::min(target_time - data.latest_update, target_time - data.latest_measurement));
    const Eigen::MatrixXd a = aDt(dt);
    KfPoseState predicted = data.state;
    // Linear Gaussian prediction: x'=A x, P'=A P A^T + Q.
    predicted.x = a * data.state.x;
    predicted.covariance = a * data.state.covariance * a.transpose() + qDt(dt);
    if (apply_update) {
        data.state = predicted;
        data.latest_update = target_time;
    }
    return predicted;
}

KfPoseState KfPose::correctWithMeasurement(FilterData& data, const KfPoseMeasurement& measurement, double& match_level, bool prior_predict, bool apply_update)
{
    FilterData local = data;
    if (prior_predict) {
        predictTillTime(local, measurement.stamp, true);
    }

    const int angle_offset = (config_.use_velocity && !config_.anonymous_measurements) ? 6 : 3;
    local.state.x[angle_offset + 0] = fixAngle(local.state.x[angle_offset + 0], measurement.x[3]);
    local.state.x[angle_offset + 1] = fixAngle(local.state.x[angle_offset + 1], measurement.x[4]);
    local.state.x[angle_offset + 2] = fixAngle(local.state.x[angle_offset + 2], measurement.x[5]);

    // Association is based only on 3D position overlap, not orientation.
    match_level = gaussJointMaxVal(
        measurement.covariance.topLeftCorner<3, 3>(),
        local.state.covariance.topLeftCorner<3, 3>(),
        measurement.x.head<3>(),
        local.state.x.head<3>());

    Eigen::MatrixXd r = measurement.covariance;
    r.topRightCorner(3, 3).setZero();
    r.bottomLeftCorner(3, 3).setZero();
    if (!std::isfinite(match_level) || match_level < 1.0e-9) {
        r = Eigen::MatrixXd::Identity(6, 6) * 10000.0;
    } else {
        // Inflate position covariance by inverse positional overlap before
        // correction, then apply the eigenvalue padding below.
        r.topLeftCorner(3, 3) *= 1.0 / match_level;
    }

    const Eigen::MatrixXd h_matrix = h();
    // Linear Kalman correction for measurement z=[p,rpy].
    const Eigen::MatrixXd s = h_matrix * local.state.covariance * h_matrix.transpose() + r;
    const Eigen::MatrixXd k = local.state.covariance * h_matrix.transpose() * s.inverse();
    local.state.x = local.state.x + k * (measurement.x - h_matrix * local.state.x);
    local.state.covariance = (Eigen::MatrixXd::Identity(local.state.x.size(), local.state.x.size()) - k * h_matrix) * local.state.covariance;

    const auto eigens = local.state.covariance.topLeftCorner<3, 3>().eigenvalues().real();
    local.state.covariance.topLeftCorner<3, 3>() += Eigen::Matrix3d::Identity() * (eigens.minCoeff() * match_level);

    local.state.x[angle_offset + 0] = fixAngle(local.state.x[angle_offset + 0], 0.0);
    local.state.x[angle_offset + 1] = fixAngle(local.state.x[angle_offset + 1], 0.0);
    local.state.x[angle_offset + 2] = fixAngle(local.state.x[angle_offset + 2], 0.0);
    const bool accepted = !config_.accepts_correction
        || config_.accepts_correction(local.state.x.head<3>(), measurement.camera_frame, measurement.stamp);
    if (accepted) {
        local.latest_update = measurement.stamp;
        local.latest_measurement = measurement.stamp;
        ++local.state.update_count;
    }

    if (apply_update && accepted) {
        data = local;
    }
    return local.state;
}

double KfPose::gaussJointMaxVal(const Eigen::MatrixXd& sigma0, const Eigen::MatrixXd& sigma1, const Eigen::VectorXd& mu0, const Eigen::VectorXd& mu1) const
{
    const Eigen::MatrixXd gain = sigma0 * (sigma0 + sigma1).inverse();
    const Eigen::VectorXd delta = mu1 - mu0;
    const Eigen::VectorXd d0 = gain * delta;
    const Eigen::VectorXd d1 = (gain - Eigen::MatrixXd::Identity(gain.rows(), gain.cols())) * delta;
    const double exponent = -0.5 * (
        (d0.transpose() * sigma0.inverse() * d0)(0, 0)
        + (d1.transpose() * sigma1.inverse() * d1)(0, 0));
    return std::exp(exponent);
}

void KfPose::removeNans()
{
    states_.erase(std::remove_if(states_.begin(), states_.end(), [](const FilterData& data) {
        return hasNan(data.state.x) || hasNan(data.state.covariance);
    }), states_.end());
}

void KfPose::removeOverlaps()
{
    for (std::size_t i = 0; i + 1 < states_.size(); ++i) {
        bool removed_first = false;
        for (std::size_t j = i + 1; j < states_.size(); ++j) {
            const double match = gaussJointMaxVal(
                states_[i].state.covariance.topLeftCorner<3, 3>(),
                states_[j].state.covariance.topLeftCorner<3, 3>(),
                states_[i].state.x.head<3>(),
                states_[j].state.x.head<3>());
            if (match <= config_.match_level_threshold_remove) {
                continue;
            }

            const bool i_tentative = states_[i].state.update_count < config_.min_measurements_to_validation;
            const bool j_tentative = states_[j].state.update_count < config_.min_measurements_to_validation;
            std::size_t remove_index = i;
            if (i_tentative == j_tentative) {
                const double size_i = states_[i].state.covariance.topLeftCorner<3, 3>().eigenvalues().real().norm();
                const double size_j = states_[j].state.covariance.topLeftCorner<3, 3>().eigenvalues().real().norm();
                remove_index = size_j > size_i ? j : i;
            } else {
                remove_index = i_tentative ? i : j;
            }
            states_.erase(states_.begin() + static_cast<long>(remove_index));
            if (remove_index == i) {
                removed_first = true;
                break;
            }
            --j;
        }
        if (removed_first) {
            --i;
        }
    }
}

Eigen::MatrixXd KfPose::aDt(double dt) const
{
    if (config_.use_velocity && !config_.anonymous_measurements) {
        // Constant-velocity model for position; orientation is static.
        Eigen::MatrixXd a = Eigen::MatrixXd::Identity(9, 9);
        a(0, 3) = dt;
        a(1, 4) = dt;
        a(2, 5) = dt;
        return a;
    }
    return Eigen::MatrixXd::Identity(6, 6);
}

Eigen::MatrixXd KfPose::h() const
{
    if (config_.use_velocity && !config_.anonymous_measurements) {
        Eigen::MatrixXd h_matrix = Eigen::MatrixXd::Zero(6, 9);
        h_matrix(0, 0) = h_matrix(1, 1) = h_matrix(2, 2) = 1.0;
        h_matrix(3, 6) = h_matrix(4, 7) = h_matrix(5, 8) = 1.0;
        return h_matrix;
    }
    return Eigen::MatrixXd::Identity(6, 6);
}

Eigen::MatrixXd KfPose::qDt(double dt) const
{
    if (config_.anonymous_measurements) {
        // Anonymous mode uses direct pose random-walk process noise.
        Eigen::MatrixXd q(6, 6);
        q << vl_, 0, 0, 0, 0, 0,
            0, vl_, 0, 0, 0, 0,
            0, 0, vv_, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;
        return q;
    }
    if (config_.use_velocity) {
        // Identified velocity mode has independent process noise on p, v, rpy.
        Eigen::MatrixXd q = Eigen::MatrixXd::Zero(9, 9);
        q.diagonal() << sn_ * sn_, sn_ * sn_, sn_ * sn_, vl_, vl_, vv_, 1.0, 1.0, 1.0;
        return q;
    }

    // Non-velocity mode uses the integrated position-noise heuristic.
    Eigen::MatrixXd q(6, 6);
    q << 0.5 * sn_ * sn_ + 0.16667 * vl_ * vl_ * dt * dt, 0, 0, 0, 0, 0,
        0, 0.5 * sn_ * sn_ + 0.16667 * vl_ * vl_ * dt * dt, 0, 0, 0, 0,
        0, 0, 0.5 * sn_ * sn_ + 0.16667 * vv_ * vv_ * dt * dt, 0, 0, 0,
        0, 0, 0, 0.5, 0, 0,
        0, 0, 0, 0, 0.5, 0,
        0, 0, 0, 0, 0, 0.5;
    return q;
}

double KfPose::fixAngle(double original, double measurement) const
{
    double fixed = std::fmod(original, 2.0 * M_PI);
    if (fixed > M_PI) {
        fixed -= 2.0 * M_PI;
    }
    if (fixed < -M_PI) {
        fixed += 2.0 * M_PI;
    }
    if (std::fabs(measurement - fixed) < M_PI) {
        return fixed;
    }
    return fixed > measurement ? fixed - 2.0 * M_PI : fixed + 2.0 * M_PI;
}

} // namespace uvdar_core::pose_estimation
