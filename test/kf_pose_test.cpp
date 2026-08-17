// Regression tests for defects that are not observable from a running demo:
// the 9D velocity state never being created, a singular covariance deleting
// tracks, process noise depending on the spin rate, the validation/decay
// threshold mismatch, and unobservable pose directions collapsing to zero
// variance.

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "uvdar_core/pose_estimation/kf_pose.hpp"
#include "uvdar_core/pose_estimation/uncertainty.hpp"

namespace pe = uvdar_core::pose_estimation;

namespace {

pe::KfPoseMeasurement makeMeasurement(int id, const Eigen::Vector3d& position, double stamp, double variance = 1.0)
{
    pe::KfPoseMeasurement measurement;
    measurement.id = id;
    measurement.x = Eigen::VectorXd::Zero(6);
    measurement.x.head<3>() = position;
    measurement.covariance = Eigen::MatrixXd::Identity(6, 6) * variance;
    measurement.stamp = stamp;
    measurement.receipt_stamp = stamp;
    return measurement;
}

std::size_t trackCount(const pe::KfPose& filter)
{
    return filter.validatedStates().size() + filter.tentativeStates().size();
}

} // namespace

// Defect 2: initiateNew() returned before pushing a state whenever the 9D
// layout was selected, so both output topics stayed permanently empty.
TEST(KfPoseTest, VelocityStateCreatesTracks)
{
    pe::KfPoseConfig config;
    config.use_velocity = true;
    config.anonymous_measurements = false;

    pe::KfPose filter(config);
    filter.applyMeasurements({makeMeasurement(1, {5.0, 0.0, 0.0}, 0.0)});
    filter.spin(0.05);

    const auto states = filter.tentativeStates();
    ASSERT_EQ(states.size(), 1U);
    EXPECT_EQ(states.front().x.size(), 9);
    EXPECT_EQ(states.front().covariance.rows(), 9);
    // Velocity starts at zero with the configured prior variance.
    EXPECT_DOUBLE_EQ(states.front().x.segment<3>(3).norm(), 0.0);
    EXPECT_GT(states.front().covariance(3, 3), 0.0);
}

// Defect 3: a singular position covariance makes the Gaussian overlap score
// NaN. "NaN <= threshold" is false, so the old guard fell through to erase().
TEST(KfPoseTest, SingularCovarianceDoesNotDeleteTracks)
{
    pe::KfPoseConfig config;
    config.anonymous_measurements = true;

    auto first = makeMeasurement(-1, {10.0, 0.0, 0.0}, 0.0);
    auto second = makeMeasurement(-1, {-10.0, 0.0, 0.0}, 0.0);
    // Rank-deficient position block, as the geometric solver can emit for
    // degenerate LED geometry.
    first.covariance.topLeftCorner<3, 3>().setZero();
    second.covariance.topLeftCorner<3, 3>().setZero();

    pe::KfPose filter(config);
    filter.applyMeasurements({first, second});
    ASSERT_EQ(trackCount(filter), 2U);

    filter.spin(0.05);
    EXPECT_EQ(trackCount(filter), 2U);
}

// Defect 4: Q was applied once per spin() regardless of elapsed time, so the
// estimate silently depended on output_framerate.
TEST(KfPoseTest, ProcessNoiseIsIndependentOfSpinRate)
{
    pe::KfPoseConfig config;
    config.min_measurements_to_validation = 0;

    pe::KfPose slow(config);
    pe::KfPose fast(config);
    const auto measurement = makeMeasurement(1, {5.0, 0.0, 0.0}, 0.0);
    slow.applyMeasurements({measurement});
    fast.applyMeasurements({measurement});

    for (int i = 1; i <= 20; ++i) {
        slow.spin(static_cast<double>(i) * 0.05); // 20 Hz
    }
    for (int i = 1; i <= 100; ++i) {
        fast.spin(static_cast<double>(i) * 0.01); // 100 Hz
    }

    const auto slow_states = slow.validatedStates();
    const auto fast_states = fast.validatedStates();
    ASSERT_EQ(slow_states.size(), 1U);
    ASSERT_EQ(fast_states.size(), 1U);
    EXPECT_NEAR(slow_states.front().covariance.trace(), fast_states.front().covariance.trace(), 1.0e-9);
}

// Defect 7: spin() used ">" while validatedStates() used ">=", so a track with
// exactly min_measurements_to_validation updates was published as validated
// while still decaying on the short unvalidated timeout.
TEST(KfPoseTest, ValidationAndDecayThresholdsAgree)
{
    pe::KfPoseConfig config;
    config.min_measurements_to_validation = 2;
    config.decay_age_unvalidated = 0.01;
    config.decay_age_normal = 10.0;

    pe::KfPose filter(config);
    filter.applyMeasurements({makeMeasurement(1, {5.0, 0.0, 0.0}, 0.0)});
    filter.applyMeasurements({makeMeasurement(1, {5.0, 0.0, 0.0}, 0.1)});
    filter.applyMeasurements({makeMeasurement(1, {5.0, 0.0, 0.0}, 0.2)});

    // update_count is now exactly min_measurements_to_validation.
    ASSERT_EQ(filter.validatedStates().size(), 1U);

    // Well past the unvalidated timeout, well inside the validated one.
    filter.spin(0.7);
    EXPECT_EQ(filter.validatedStates().size(), 1U);
}

// The geometric solver's covariance source: a truncated pseudo-inverse mapped
// unobservable directions to zero variance, i.e. total confidence along the
// one axis the geometry failed to constrain.
TEST(UncertaintyTest, UnobservableDirectionGetsLargeVariance)
{
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity() * 1.0e6;
    information(2, 2) = 0.0;

    const Eigen::Matrix<double, 6, 6> covariance =
        uvdar_core::pose_estimation::uncertainty::covarianceFromInformation(information);

    ASSERT_TRUE(covariance.allFinite());
    EXPECT_GT(covariance(2, 2), 1.0e3);
    EXPECT_LE(covariance(2, 2), 1.0e4 + 1.0);
    // Well-constrained directions are unaffected.
    EXPECT_NEAR(covariance(0, 0), 1.0e-6, 1.0e-9);
}
