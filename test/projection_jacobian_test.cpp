// Finite-difference validation of the pose-estimation Jacobians.
//
// The geometric solver reported a line-of-sight position sigma ~28x smaller
// than the scatter actually observed in flight, and claimed range was better
// determined than bearing -- the inverse of what the geometry allows. These
// tests localise that by checking each analytic Jacobian against central
// differences of the function it claims to differentiate.

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "uvdar_core/calibration/fisheye/ocam_model.hpp"
#include "uvdar_core/calibration/pinhole/pinhole_model.hpp"
#include "uvdar_core/pose_estimation/camera_model.hpp"
#include "uvdar_core/pose_estimation/math.hpp"
#include "uvdar_core/pose_estimation/uncertainty.hpp"

namespace unc = uvdar_core::pose_estimation::uncertainty;
using uvdar_core::pose_estimation::CameraModel;
using uvdar_core::pose_estimation::expSO3;

namespace {

// A plain perspective camera. Using pinhole rather than the OCamCalib fisheye
// keeps these tests about the pose geometry: if they fail here, the fault is
// not in the lens polynomial.
CameraModel makeCamera()
{
    uvdar_core::calibration::pinhole::PinholeModel::Parameters p;
    p.fx = 400.0;
    p.fy = 400.0;
    p.cx = 376.0;
    p.cy = 240.0;
    p.width = 752;
    p.height = 480;
    CameraModel camera;
    camera.lens = std::make_shared<uvdar_core::calibration::pinhole::PinholeModel>(p);
    camera.image_width = p.width;
    camera.image_height = p.height;
    return camera;
}

// The OCamCalib fisheye actually flown, from
// uvdar_gazebo_plugin/tmux/two_drones_uvdar/config/camera/ocam.yaml.
CameraModel makeOcamCamera()
{
    auto model = std::make_shared<uvdar_core::calibration::fisheye::OcamModel>();
    const std::vector<double> pol {-259.5395, 0.0, 0.001626945, -0.000002328091, 0.000000006601777};
    const std::vector<double> invpol {388.557547, 220.910537, -10.127377, 31.174444, 16.299989,
                                      0.874110, 31.882430, 44.770601, 21.586002, 3.593416};
    model->length_pol = static_cast<int>(pol.size());
    for (std::size_t i = 0; i < pol.size(); ++i) {
        model->pol[i] = pol[i];
    }
    model->length_invpol = static_cast<int>(invpol.size());
    for (std::size_t i = 0; i < invpol.size(); ++i) {
        model->invpol[i] = invpol[i];
    }
    model->xc = 229.581672;
    model->yc = 364.430465;
    model->c = 1.000314;
    model->d = -0.000012;
    model->e = -0.000158;
    model->width = 752;
    model->height = 480;

    CameraModel camera;
    camera.lens = model;
    camera.image_width = model->width;
    camera.image_height = model->height;
    return camera;
}

// Target 5.4 m down the optical axis, matching the observed flight geometry.
unc::CameraPose makePose()
{
    unc::CameraPose pose;
    pose.rotation = expSO3(Eigen::Vector3d(0.05, -0.03, 0.02));
    pose.translation = Eigen::Vector3d(0.4, -0.2, 5.4);
    return pose;
}

// Four LEDs on the x500's 0.2804 m ring.
std::vector<Eigen::Vector3d> makeBodyPoints()
{
    return {
        {0.2018, 0.1947, 0.0},
        {0.1947, -0.2018, 0.0},
        {-0.1947, 0.2018, 0.0},
        {-0.2018, -0.1947, 0.0},
    };
}

} // namespace

// The lens itself: d(project)/d(camera_point).
TEST(ProjectionJacobianTest, LensJacobianMatchesFiniteDifference)
{
    const CameraModel camera = makeCamera();
    const Eigen::Vector3d point(0.6, -0.35, 5.4);
    const Eigen::Matrix<double, 2, 3> analytic = camera.lens->projectJacobian(point);

    constexpr double h = 1.0e-6;
    Eigen::Matrix<double, 2, 3> numeric;
    for (int k = 0; k < 3; ++k) {
        Eigen::Vector3d plus = point;
        Eigen::Vector3d minus = point;
        plus(k) += h;
        minus(k) -= h;
        numeric.col(k) = (camera.lens->project(plus) - camera.lens->project(minus)) / (2.0 * h);
    }

    for (int r = 0; r < 2; ++r) {
        for (int c = 0; c < 3; ++c) {
            EXPECT_NEAR(analytic(r, c), numeric(r, c), 1.0e-3 * std::max(1.0, std::abs(numeric(r, c))))
                << "lens jacobian mismatch at (" << r << "," << c << ")";
        }
    }
}

// The fisheye actually flown. The pinhole test above isolates the pose
// geometry; this one isolates the lens model that the live system uses.
TEST(ProjectionJacobianTest, OcamLensJacobianMatchesFiniteDifference)
{
    const CameraModel camera = makeOcamCamera();
    const Eigen::Vector3d point(0.6, -0.35, 5.4);
    const Eigen::Matrix<double, 2, 3> analytic = camera.lens->projectJacobian(point);

    constexpr double h = 1.0e-6;
    Eigen::Matrix<double, 2, 3> numeric;
    for (int k = 0; k < 3; ++k) {
        Eigen::Vector3d plus = point;
        Eigen::Vector3d minus = point;
        plus(k) += h;
        minus(k) -= h;
        numeric.col(k) = (camera.lens->project(plus) - camera.lens->project(minus)) / (2.0 * h);
    }

    std::cout << "  ocam analytic:\n" << analytic << "\n  ocam numeric:\n" << numeric << "\n";
    for (int r = 0; r < 2; ++r) {
        for (int c = 0; c < 3; ++c) {
            EXPECT_NEAR(analytic(r, c), numeric(r, c), 1.0e-3 * std::max(1.0, std::abs(numeric(r, c))))
                << "ocam jacobian mismatch at (" << r << "," << c << ")";
        }
    }
}

// The pose Jacobian must differentiate the SAME parameterisation that
// GeometricSolver::refinePose steps along:
//     translation += delta.head<3>()
//     rotation     = expSO3(delta.tail<3>()) * rotation      <- t held fixed
// so d(X_c)/d(theta) = -skew(R * X_b), NOT -skew(X_c).
TEST(ProjectionJacobianTest, PoseJacobianMatchesSolverUpdateConvention)
{
    const CameraModel camera = makeCamera();
    const unc::CameraPose pose = makePose();
    const Eigen::Vector3d body_point(0.2018, 0.1947, 0.0);

    const Eigen::Matrix<double, 2, 6> analytic =
        unc::imageProjectionJacobian(camera, pose, body_point);

    constexpr double h = 1.0e-7;
    Eigen::Matrix<double, 2, 6> numeric;
    for (int k = 0; k < 6; ++k) {
        Eigen::Matrix<double, 6, 1> delta_plus = Eigen::Matrix<double, 6, 1>::Zero();
        Eigen::Matrix<double, 6, 1> delta_minus = Eigen::Matrix<double, 6, 1>::Zero();
        delta_plus(k) = h;
        delta_minus(k) = -h;

        auto apply = [&](const Eigen::Matrix<double, 6, 1>& d) {
            unc::CameraPose perturbed;
            perturbed.translation = pose.translation + d.head<3>();
            perturbed.rotation = expSO3(d.tail<3>()) * pose.rotation;
            return camera.lens->project(perturbed.rotation * body_point + perturbed.translation);
        };
        numeric.col(k) = (apply(delta_plus) - apply(delta_minus)) / (2.0 * h);
    }

    for (int r = 0; r < 2; ++r) {
        for (int c = 0; c < 6; ++c) {
            const double scale = std::max(1.0, std::abs(numeric(r, c)));
            EXPECT_NEAR(analytic(r, c), numeric(r, c), 1.0e-3 * scale)
                << (c < 3 ? "translation" : "rotation") << " block mismatch at (" << r << "," << c << ")";
        }
    }
}

// The physical acceptance criterion. A single bearing carries no information
// about range, so range is always the weakest direction; the reported
// covariance must say so.
TEST(ProjectionJacobianTest, RangeIsLessCertainThanBearing)
{
    const CameraModel camera = makeCamera();
    const unc::CameraPose pose = makePose();
    const auto body_points = makeBodyPoints();
    const std::vector<Eigen::Matrix2d> pixel_covariances(
        body_points.size(), Eigen::Matrix2d::Identity() * 1.08);

    const Eigen::Matrix<double, 6, 6> covariance = unc::poseCovarianceFromPixelsLinearized(
        pose, body_points, pixel_covariances, camera, 1.0e-6);

    const Eigen::Matrix3d position = covariance.topLeftCorner<3, 3>();
    const Eigen::Vector3d line_of_sight = pose.translation.normalized();
    const double radial = line_of_sight.transpose() * position * line_of_sight;
    const double tangential = std::max(0.0, position.trace() - radial) / 2.0;

    ASSERT_GT(radial, 0.0);
    ASSERT_GT(tangential, 0.0);

    std::cout << "  sigma_radial     = " << std::sqrt(radial) * 1000.0 << " mm\n"
              << "  sigma_tangential = " << std::sqrt(tangential) * 1000.0 << " mm\n"
              << "  ratio            = " << std::sqrt(radial / tangential) << "x\n";

    // Range from apparent LED separation: sigma_r ~ r^2 * sigma_theta / baseline.
    // Here 5.4^2 * (1.04/400) / 0.4 ~ 0.19 m, versus ~13 mm laterally, so the
    // radial variance must dominate by a wide margin.
    EXPECT_GT(std::sqrt(radial), std::sqrt(tangential))
        << "range reported as better determined than bearing: sigma_radial="
        << std::sqrt(radial) << " m, sigma_tangential=" << std::sqrt(tangential) << " m";
    EXPECT_GT(std::sqrt(radial) / std::sqrt(tangential), 3.0)
        << "range/bearing uncertainty ratio far below what the geometry implies";
}
