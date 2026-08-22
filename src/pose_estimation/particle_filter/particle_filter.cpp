#include "uvdar_core/pose_estimation/particle_filter/particle_filter.hpp"
#include "uvdar_core/pose_estimation/particle_filter/sampling.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>

namespace uvdar_core::pose_estimation::particle_filter {

namespace {

Eigen::Matrix<double, 6, 6> singletonCovariance()
{
    // A single verified particle has no sample spread; keep a finite covariance
    // so downstream filters still receive a usable Gaussian measurement.
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
    covariance.topLeftCorner<3, 3>() = 0.5 * Eigen::Matrix3d::Identity();
    covariance.bottomRightCorner<3, 3>() = 0.5 * Eigen::Matrix3d::Identity();
    return covariance;
}

} // namespace

ParticleFilter::ParticleFilter(ParticleFilterConfig config, uvdar_core::pose_estimation::BodyModel body, std::vector<int> signal_ids, ReprojectionModelPtr reprojection_model)
    : config_(std::move(config))
    , body_(std::move(body))
    , signal_ids_(std::move(signal_ids))
    , signals_per_target_(std::max(1, body_.maxSignalId() + 1))
    , reprojection_model_(std::move(reprojection_model))
    , rng_(std::random_device {}())
{
}

void ParticleFilter::processFrame(
    std::size_t camera_index,
    const std::vector<TrackedPoint>& points,
    int,
    int,
    double stamp,
    const Eigen::Isometry3d& camera_to_output,
    const Eigen::Isometry3d& output_to_camera)
{
    std::vector<TrackedPoint> usable_points;
    for (const auto& point : points) {
        if (point.id >= 0 && point.id <= 200 && !point.virtual_point) {
            usable_points.push_back(point);
        }
    }
    if (usable_points.empty()) {
        return;
    }

    // The filter reasons per target/image cluster, then merges accepted
    // particles into one measurement hull per target.
    const std::vector<ImageCluster> clusters = separateBySignals(usable_points);
    std::scoped_lock lock(mutex_);

    for (const auto& cluster : clusters) {
        const auto new_hypotheses = reprojection_model_->extractHypotheses(
            cluster.points,
            cluster.id,
            camera_index,
            camera_to_output,
            output_to_camera,
            stamp);
        if (new_hypotheses.empty()) {
            continue;
        }

        auto existing = std::find_if(hypothesis_buffer_.begin(), hypothesis_buffer_.end(), [&](const AssociatedHypotheses& hypotheses) {
            return hypotheses.target == (cluster.id % 1000);
        });
        if (existing == hypothesis_buffer_.end()) {
            AssociatedHypotheses associated;
            associated.target = cluster.id % 1000;
            associated.add(new_hypotheses);
            hypothesis_buffer_.push_back(std::move(associated));
        } else {
            existing->add(new_hypotheses);
        }
    }

    for (auto& hypotheses : hypothesis_buffer_) {
        // Existing particles are re-scored against the current frame with two
        // gates: a loose unfit gate and a tight verified gate.
        checkHypothesisFitness(
            hypotheses,
            camera_index,
            reprojection_model_->reprojectionThresholdUnfit(camera_index),
            reprojection_model_->reprojectionThresholdVerified(camera_index),
            clusters,
            output_to_camera,
            stamp);
        hypotheses.removeUnfit();
    }
}

TimedPoseMeasurements ParticleFilter::scatterAndMeasure(double now, double stamp)
{
    std::scoped_lock lock(mutex_);

    for (auto& hypotheses : hypothesis_buffer_) {
        // Diffusion step: grow the particle cloud around existing hypotheses,
        // then prune by age/count before publishing the hull.
        const int mutation_count = static_cast<int>(hypotheses.hypotheses.size() / 2U);
        hypotheses.add(mutateHypotheses(hypotheses, mutation_count));
        removeExtraHypotheses(hypotheses, now);
    }

    TimedPoseMeasurements output;
    output.stamp = stamp;
    output.frame_id = config_.output_frame;
    for (const auto& hypotheses : hypothesis_buffer_) {
        if (auto measurement = measurementHull(hypotheses); measurement) {
            output.poses.push_back(*measurement);
        }
    }

    propagate(now);
    return output;
}

std::vector<PoseMeasurement> ParticleFilter::verifiedHypotheses() const
{
    std::scoped_lock lock(mutex_);
    std::vector<PoseMeasurement> output;
    for (const auto& set : hypothesis_buffer_) {
        for (const auto& hypothesis : set.hypotheses) {
            if (hypothesis.flag == HypothesisFlag::Verified) {
                output.push_back({hypothesis.index, hypothesis.pose, Eigen::Matrix<double, 6, 6>::Identity() * 0.01, "PF"});
            }
        }
    }
    return output;
}

std::vector<PoseMeasurement> ParticleFilter::tentativeHypotheses() const
{
    std::scoped_lock lock(mutex_);
    std::vector<PoseMeasurement> output;
    for (const auto& set : hypothesis_buffer_) {
        for (const auto& hypothesis : set.hypotheses) {
            if (hypothesis.flag == HypothesisFlag::Neutral) {
                output.push_back({hypothesis.index, hypothesis.pose, Eigen::Matrix<double, 6, 6>::Identity() * 0.01, "PF"});
            }
        }
    }
    return output;
}

std::vector<ImageCluster> ParticleFilter::separateBySignals(const std::vector<TrackedPoint>& points) const
{
    std::vector<ImageCluster> separated;
    for (const auto& point : points) {
        const int target = targetForSignal(signal_ids_, signals_per_target_, point.id);
        if (target < 0) {
            continue;
        }

        auto cluster = std::find_if(separated.begin(), separated.end(), [&](const ImageCluster& candidate) {
            return candidate.id == target;
        });
        if (cluster == separated.end()) {
            separated.push_back({target, {{point.id, Eigen::Vector2i(static_cast<int>(std::llround(point.x)), static_cast<int>(std::llround(point.y)))}}});
        } else {
            cluster->points.push_back({point.id, Eigen::Vector2i(static_cast<int>(std::llround(point.x)), static_cast<int>(std::llround(point.y)))});
        }
    }

    if (!config_.separate_by_distance) {
        return separated;
    }

    // Multiple physical targets can emit the same signal set. Split one target
    // id into synthetic clusters by image-space centroid distance.
    for (std::size_t s = 0; s < separated.size(); ++s) {
        std::vector<ImageCluster> clusters;
        std::vector<Eigen::Vector2d> centroids;
        for (const auto& point : separated[s].points) {
            bool found = false;
            for (std::size_t j = 0; j < clusters.size(); ++j) {
                if ((point.position.cast<double>() - centroids[j]).norm() < config_.max_cluster_distance) {
                    clusters[j].points.push_back(point);
                    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
                    for (const auto& clustered : clusters[j].points) {
                        centroid += clustered.position.cast<double>();
                    }
                    centroids[j] = centroid / static_cast<double>(clusters[j].points.size());
                    found = true;
                    break;
                }
            }
            if (!found) {
                clusters.push_back({separated[s].id, {point}});
                centroids.push_back(point.position.cast<double>());
            }
        }

        if (clusters.size() > 1U) {
            const int original_id = separated[s].id;
            separated.erase(separated.begin() + static_cast<long>(s));
            for (std::size_t i = 0; i < clusters.size(); ++i) {
                clusters[i].id = static_cast<int>(i) * 1000 + original_id;
                separated.insert(separated.begin() + static_cast<long>(s + i), clusters[i]);
            }
            s += clusters.size() - 1U;
        }
    }

    return separated;
}

void ParticleFilter::checkHypothesisFitness(
    AssociatedHypotheses& hypotheses,
    std::size_t camera_index,
    double threshold_unfit,
    double threshold_verified,
    const std::vector<ImageCluster>& clusters,
    const Eigen::Isometry3d& output_to_camera,
    double stamp)
{
    ReprojectionContext context;
    context.camera_index = camera_index;
    context.output_to_camera = output_to_camera;
    context.body = body_;

    for (const auto& cluster : clusters) {
        if ((cluster.id % 1000) != (hypotheses.target % 1000)) {
            continue;
        }

        context.observed_points = cluster.points;
        for (auto it = hypotheses.hypotheses.begin(); it != hypotheses.hypotheses.end(); ++it) {
            context.target = it->index;
            const double error = reprojection_model_->hypothesisError(*it, context);
            const double scaled_unfit = static_cast<double>(cluster.points.size()) * threshold_unfit;
            const double scaled_verified = static_cast<double>(cluster.points.size()) * threshold_verified;
            if (error > scaled_unfit) {
                hypotheses.setUnfit(it);
            } else if (error < scaled_verified) {
                hypotheses.setVerified(it);
                it->observed = stamp;
            } else {
                hypotheses.setNeutral(it);
            }
        }
    }
}

void ParticleFilter::removeExtraHypotheses(AssociatedHypotheses& hypotheses, double now)
{
    for (auto it = hypotheses.hypotheses.begin(); it != hypotheses.hypotheses.end();) {
        if (now - it->observed > config_.max_hypothesis_age) {
            it = hypotheses.erase(it);
        } else {
            ++it;
        }
    }

    while (static_cast<int>(hypotheses.hypotheses.size()) > config_.max_hypothesis_count) {
        // Prefer removing tentative particles; if all are verified, thin the
        // verified set randomly to keep runtime bounded.
        std::vector<std::list<Hypothesis>::iterator> removable;
        for (auto it = hypotheses.hypotheses.begin(); it != hypotheses.hypotheses.end(); ++it) {
            if (it->flag != HypothesisFlag::Verified) {
                removable.push_back(it);
            }
        }

        if (removable.empty()) {
            for (auto it = hypotheses.hypotheses.begin(); it != hypotheses.hypotheses.end(); ++it) {
                removable.push_back(it);
            }
        }

        const int selected = std::min(
            static_cast<int>(randomUniform01(rng_) * static_cast<double>(removable.size())),
            static_cast<int>(removable.size()) - 1);
        hypotheses.erase(removable[static_cast<std::size_t>(selected)]);
    }
}

std::vector<Hypothesis> ParticleFilter::mutateHypotheses(const AssociatedHypotheses& hypotheses, int count) const
{
    if (hypotheses.hypotheses.empty() || count <= 0) {
        return {};
    }

    std::vector<Hypothesis> mutations;
    for (int i = 0; i < count; ++i) {
        const int parent_index = static_cast<int>(randomUniform01(rng_) * static_cast<double>(hypotheses.hypotheses.size()));
        auto selected = hypotheses.hypotheses.begin();
        std::advance(selected, std::min(parent_index, static_cast<int>(hypotheses.hypotheses.size()) - 1));

        if (selected->flag == HypothesisFlag::Verified) {
            // Verified particles receive both velocity and pose mutations to
            // track motion while preserving nearby pose alternatives.
            auto velocity_mutations = generateVelocityMutations(
                *selected,
                10,
                config_.mutation_velocity_max_step,
                rng_);
            mutations.insert(mutations.end(), velocity_mutations.begin(), velocity_mutations.end());
            auto pose_mutations = generatePoseMutations(
                *selected,
                10,
                config_.mutation_position_max_step,
                config_.mutation_orientation_max_step,
                rng_);
            mutations.insert(mutations.end(), pose_mutations.begin(), pose_mutations.end());
        } else if (selected->flag == HypothesisFlag::Neutral) {
            // Tentative particles only receive a light pose mutation.
            auto pose_mutations = generatePoseMutations(
                *selected,
                1,
                config_.mutation_position_max_step,
                config_.mutation_orientation_max_step,
                rng_);
            mutations.insert(mutations.end(), pose_mutations.begin(), pose_mutations.end());
        } else {
            --i;
        }
    }
    return mutations;
}

void ParticleFilter::propagate(double now)
{
    for (auto& hypotheses : hypothesis_buffer_) {
        for (auto& hypothesis : hypotheses.hypotheses) {
            hypothesis.pose.position += hypothesis.twist.linear * std::max(0.0, now - hypothesis.propagated);
            hypothesis.propagated = now;
        }
    }
}

std::optional<PoseMeasurement> ParticleFilter::measurementHull(const AssociatedHypotheses& hypotheses) const
{
    if (hypotheses.verified_count < 1) {
        return std::nullopt;
    }

    if (hypotheses.verified_count == 1) {
        for (const auto& hypothesis : hypotheses.hypotheses) {
            if (hypothesis.flag == HypothesisFlag::Verified) {
                return PoseMeasurement {hypotheses.target, hypothesis.pose, singletonCovariance(), "PF"};
            }
        }
    }

    Eigen::Vector3d mean_position = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> positions;
    for (const auto& hypothesis : hypotheses.hypotheses) {
        if (hypothesis.flag == HypothesisFlag::Verified) {
            positions.push_back(hypothesis.pose.position);
            mean_position += hypothesis.pose.position;
        }
    }
    mean_position /= static_cast<double>(positions.size());

    const Eigen::Quaterniond mean_orientation = averageOrientation(hypotheses.verified());
    std::vector<Eigen::Vector3d> position_diff;
    std::vector<Eigen::Vector3d> orientation_diff;
    for (const auto& hypothesis : hypotheses.hypotheses) {
        if (hypothesis.flag == HypothesisFlag::Verified) {
            position_diff.push_back(hypothesis.pose.position - mean_position);
            orientation_diff.push_back(uvdar_core::helpers::quaternionToRpy(hypothesis.pose.orientation * mean_orientation.inverse()));
        }
    }

    // Pose measurement covariance comes from the spread of verified particles.
    // The body model shapes reprojection scoring, not this uncertainty estimate.
    const auto position_hull = enclosingEllipsoid(position_diff);
    const auto orientation_hull = enclosingEllipsoid(orientation_diff);
    if (position_hull.first.array().isNaN().any() || position_hull.second.array().isNaN().any()
        || orientation_hull.first.array().isNaN().any() || orientation_hull.second.array().isNaN().any()) {
        return std::nullopt;
    }

    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
    covariance.topLeftCorner<3, 3>() = position_hull.second;
    covariance.bottomRightCorner<3, 3>() = orientation_hull.second;

    Pose pose;
    pose.position = mean_position + position_hull.first;
    pose.orientation = mean_orientation * uvdar_core::helpers::rpyToQuaternion(orientation_hull.first);
    return PoseMeasurement {hypotheses.target, pose, covariance, "PF"};
}

std::pair<Eigen::Vector3d, Eigen::Matrix3d> ParticleFilter::enclosingEllipsoid(const std::vector<Eigen::Vector3d>& points) const
{
    if (points.empty()) {
        return {Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity()};
    }
    if (points.size() == 1U) {
        return {points.front(), Eigen::Matrix3d::Identity() * 1.0e-3};
    }

    // Khachiyan-style minimum-volume enclosing ellipsoid iteration. The output
    // covariance is the ellipsoid shape matrix around the verified spread.
    constexpr int d = 3;
    const double n = static_cast<double>(d + 1);
    const int count_points = static_cast<int>(points.size());
    Eigen::MatrixXd p(d, count_points);
    for (int i = 0; i < count_points; ++i) {
        p.col(i) = points[static_cast<std::size_t>(i)];
    }

    Eigen::MatrixXd q(d + 1, count_points);
    q << p, Eigen::RowVectorXd::Ones(count_points);
    Eigen::VectorXd u = Eigen::VectorXd::Constant(count_points, 1.0 / static_cast<double>(count_points));

    int count = 0;
    while (count < 1000) {
        const Eigen::Matrix4d x = q * u.asDiagonal() * q.transpose();
        const Eigen::VectorXd m = (q.transpose() * x.inverse() * q).diagonal();
        Eigen::Index jp = 0;
        Eigen::Index jm = 0;
        const double maximum = m.maxCoeff(&jp);
        const double minimum = m.minCoeff(&jm);
        const double eps_plus = maximum / n - 1.0;
        const double eps_minus = 1.0 - minimum / n;
        const double eps = std::max(eps_plus, eps_minus);
        const bool done = !((m.array() > ((1.0 + eps) * n)).any()
            || (((m.array() < ((1.0 - eps) * n)) && (u.array() > 0.00001)).any()));
        if (done) {
            break;
        }

        Eigen::VectorXd new_u;
        double step_size = 0.0;
        if (eps_plus >= eps_minus) {
            step_size = (maximum - n) / (n * (maximum - 1.0));
            new_u = (1.0 - step_size) * u;
            new_u(jp) += step_size;
        } else {
            step_size = std::min((n - minimum) / (n * (minimum - 1.0)), u(jm) / (1.0 - u(jm)));
            new_u = (1.0 + step_size) * u;
            new_u(jm) -= step_size;
        }
        u = new_u;
        ++count;
    }

    const Eigen::MatrixXd weights = u.asDiagonal();
    const Eigen::Vector3d center = p * u;
    const Eigen::Matrix3d covariance = static_cast<double>(d) * (p * weights * p.transpose() - center * center.transpose());
    return {center, covariance};
}

Eigen::Quaterniond ParticleFilter::averageOrientation(const std::vector<Hypothesis>& hypotheses) const
{
    if (hypotheses.empty()) {
        return Eigen::Quaterniond::Identity();
    }

    // Markley's quaternion mean: dominant eigenvector of sum(q q^T).
    Eigen::Matrix4d accumulator = Eigen::Matrix4d::Zero();
    for (const auto& hypothesis : hypotheses) {
        accumulator += hypothesis.pose.orientation.coeffs() * hypothesis.pose.orientation.coeffs().transpose();
    }
    accumulator /= static_cast<double>(hypotheses.size());

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(accumulator, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Index index = 0;
    svd.singularValues().maxCoeff(&index);
    const Eigen::Vector4d coeffs = svd.matrixU().col(index);
    return Eigen::Quaterniond(coeffs(3), coeffs(0), coeffs(1), coeffs(2)).normalized();
}

} // namespace uvdar_core::pose_estimation::particle_filter
