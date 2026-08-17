#include "uvdar_core/calibration/fisheye/ocam_model.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "uvdar_core/pose_estimation/math.hpp"

namespace uvdar_core::calibration::fisheye {

namespace {

constexpr double epsilon = 1.0e-12;

std::string nextDataLine(std::ifstream& input)
{
    std::string line;
    while (std::getline(input, line)) {
        const auto first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos || line[first] == '#') {
            continue;
        }
        return line.substr(first);
    }
    throw std::runtime_error("Unexpected end of OCamCalib file.");
}

double evalPolynomial(const std::array<double, max_polynomial_length>& coefficients, int length, double x)
{
    // Horner evaluation for OCamCalib polynomial coefficients.
    double value = 0.0;
    for (int i = length - 1; i >= 0; --i) {
        value = value * x + coefficients[static_cast<std::size_t>(i)];
    }
    return value;
}

void evalPolynomialAndDerivative(
    const std::array<double, max_polynomial_length>& coefficients,
    int length,
    double x,
    double& value,
    double& derivative)
{
    // Horner evaluation of f(x) and f'(x) in one pass.
    value = 0.0;
    derivative = 0.0;
    for (int i = length - 1; i >= 0; --i) {
        derivative = derivative * x + value;
        value = value * x + coefficients[static_cast<std::size_t>(i)];
    }
}

} // namespace

OcamModel loadModel(const std::string& filename)
{
    std::ifstream input(filename);
    if (!input.is_open()) {
        throw std::runtime_error("Could not open OCamCalib file '" + filename + "'.");
    }

    OcamModel model;

    std::stringstream direct_polynomial(nextDataLine(input));
    direct_polynomial >> model.length_pol;
    if (model.length_pol < 0 || model.length_pol > max_polynomial_length) {
        throw std::runtime_error("Invalid direct polynomial length in '" + filename + "'.");
    }
    for (int i = 0; i < model.length_pol; ++i) {
        direct_polynomial >> model.pol[i];
    }

    std::stringstream inverse_polynomial(nextDataLine(input));
    inverse_polynomial >> model.length_invpol;
    if (model.length_invpol < 0 || model.length_invpol > max_polynomial_length) {
        throw std::runtime_error("Invalid inverse polynomial length in '" + filename + "'.");
    }
    for (int i = 0; i < model.length_invpol; ++i) {
        inverse_polynomial >> model.invpol[i];
    }

    std::stringstream center(nextDataLine(input));
    center >> model.xc >> model.yc;

    std::stringstream affine(nextDataLine(input));
    affine >> model.c >> model.d >> model.e;

    std::stringstream dimensions(nextDataLine(input));
    dimensions >> model.height >> model.width;

    if (!input.good() && !input.eof()) {
        throw std::runtime_error("Failed while parsing OCamCalib file '" + filename + "'.");
    }
    return model;
}

Eigen::Vector3d cam2world(const Eigen::Vector2d& point_2d, const OcamModel& model)
{
    // Invert the OCamCalib affine image transform before evaluating z = pol(r).
    const double invdet = 1.0 / (model.c - model.d * model.e);
    const double xp = invdet * ((point_2d.x() - model.xc) - model.d * (point_2d.y() - model.yc));
    const double yp = invdet * (-model.e * (point_2d.x() - model.xc) + model.c * (point_2d.y() - model.yc));

    const double r = std::sqrt(xp * xp + yp * yp);
    const double zp = evalPolynomial(model.pol, model.length_pol, r);

    const double invnorm = 1.0 / std::sqrt(xp * xp + yp * yp + zp * zp);
    return {invnorm * xp, invnorm * yp, invnorm * zp};
}

Eigen::Vector2d world2cam(const Eigen::Vector3d& point_3d, const OcamModel& model)
{
    const double norm = std::sqrt(point_3d.x() * point_3d.x() + point_3d.y() * point_3d.y());
    if (norm == 0.0) {
        return {model.xc, model.yc};
    }

    // Inverse polynomial maps elevation angle to image-plane radius rho.
    const double theta = std::atan(point_3d.z() / norm);
    const double rho = evalPolynomial(model.invpol, model.length_invpol, theta);

    const double invnorm = 1.0 / norm;
    const double x = point_3d.x() * invnorm * rho;
    const double y = point_3d.y() * invnorm * rho;
    return {x * model.c + y * model.d + model.xc, x * model.e + y + model.yc};
}

Eigen::Vector3d OcamModel::backProject(const Eigen::Vector2d& image_point) const
{
    const Eigen::Vector3d raw = cam2world(Eigen::Vector2d(image_point.y(), image_point.x()), *this);
    return Eigen::Vector3d(raw.y(), raw.x(), -raw.z()).normalized();
}

Eigen::Vector2d OcamModel::project(const Eigen::Vector3d& camera_point) const
{
    const Eigen::Vector2d projected = world2cam(Eigen::Vector3d(camera_point.y(), camera_point.x(), -camera_point.z()), *this);
    return {projected.y(), projected.x()};
}

Eigen::Matrix<double, 2, 3> OcamModel::projectJacobian(const Eigen::Vector3d& camera_point) const
{
    // Work in native OCamCalib axis order, then swap back to public x,y pixels.
    const Eigen::Vector3d raw_point(camera_point.y(), camera_point.x(), -camera_point.z());
    const double x = raw_point.x();
    const double y = raw_point.y();
    const double z = raw_point.z();
    const double r = std::hypot(x, y);
    if (r < epsilon || length_invpol <= 0) {
        Eigen::Matrix<double, 2, 3> jacobian = Eigen::Matrix<double, 2, 3>::Zero();
        const double z_safe = std::abs(camera_point.z()) < epsilon ? epsilon : camera_point.z();
        jacobian(0, 0) = c / z_safe;
        jacobian(1, 1) = 1.0 / z_safe;
        return jacobian;
    }

    // Chain rule: point -> theta -> rho(theta) -> affine pixel.
    //
    // theta must be the same quantity world2cam() feeds to invpol, namely the
    // elevation angle atan(z / r) measured from the image plane. Using the
    // polar angle atan2(r, z) instead evaluates the 10th-order inverse
    // polynomial at pi/2 - theta -- a different argument entirely -- and flips
    // the sign of both chain-rule terms, which is what made this Jacobian
    // disagree with finite differences of project() by two to four orders of
    // magnitude.
    const double theta = std::atan(z / r);
    double rho = 0.0;
    double drho_dtheta = 0.0;
    evalPolynomialAndDerivative(invpol, length_invpol, theta, rho, drho_dtheta);

    const double denom = z * z + r * r;
    const double dtheta_dr = -z / denom;
    const double dtheta_dz = r / denom;
    const double dr_dx = x / r;
    const double dr_dy = y / r;
    const double drho_dx = drho_dtheta * dtheta_dr * dr_dx;
    const double drho_dy = drho_dtheta * dtheta_dr * dr_dy;
    const double drho_dz = drho_dtheta * dtheta_dz;

    const double inv_r = 1.0 / r;
    const double inv_r3 = inv_r * inv_r * inv_r;
    const double ax = x * inv_r;
    const double ay = y * inv_r;
    const double dax_dx = y * y * inv_r3;
    const double dax_dy = -x * y * inv_r3;
    const double day_dx = -x * y * inv_r3;
    const double day_dy = x * x * inv_r3;

    Eigen::Matrix<double, 2, 3> dv_draw;
    dv_draw << dax_dx * rho + ax * drho_dx, dax_dy * rho + ax * drho_dy, ax * drho_dz,
        day_dx * rho + ay * drho_dx, day_dy * rho + ay * drho_dy, ay * drho_dz;

    Eigen::Matrix2d affine;
    affine << c, d,
        e, 1.0;
    const Eigen::Matrix<double, 2, 3> draw_uv_draw_point = affine * dv_draw;

    Eigen::Matrix<double, 3, 3> draw_point_dcamera = Eigen::Matrix3d::Zero();
    draw_point_dcamera(0, 1) = 1.0;
    draw_point_dcamera(1, 0) = 1.0;
    draw_point_dcamera(2, 2) = -1.0;
    Eigen::Matrix2d public_swap;
    public_swap << 0.0, 1.0,
        1.0, 0.0;
    return public_swap * draw_uv_draw_point * draw_point_dcamera;
}

Eigen::Matrix<double, 3, 2> OcamModel::backProjectJacobian(const Eigen::Vector2d& image_point) const
{
    // Public pixels are x,y; OCamCalib stores rows,columns.
    const Eigen::Vector2d raw_image(image_point.y(), image_point.x());
    const double invdet = 1.0 / (c - d * e);
    Eigen::Matrix2d affine_inverse;
    affine_inverse << invdet, -d * invdet,
        -e * invdet, c * invdet;

    const Eigen::Vector2d centered(raw_image.x() - xc, raw_image.y() - yc);
    const Eigen::Vector2d xy = affine_inverse * centered;
    const double x = xy.x();
    const double y = xy.y();
    const double r = std::hypot(x, y);
    if (length_pol <= 0) {
        return Eigen::Matrix<double, 3, 2>::Zero();
    }

    // Direct polynomial gives z(r), then the ray is normalized.
    double z = 0.0;
    double dz_dr = 0.0;
    evalPolynomialAndDerivative(pol, length_pol, std::max(r, epsilon), z, dz_dr);
    const double r_safe = std::max(r, epsilon);
    const double dz_dx = dz_dr * x / r_safe;
    const double dz_dy = dz_dr * y / r_safe;

    Eigen::Matrix<double, 3, 2> draw_dxy;
    draw_dxy << 1.0, 0.0,
        0.0, 1.0,
        dz_dx, dz_dy;
    const Eigen::Vector3d raw_vector(x, y, z);
    const Eigen::Matrix<double, 3, 2> draw_draw_image =
        pose_estimation::normalizedVectorJacobian(raw_vector, epsilon) * draw_dxy * affine_inverse;

    Eigen::Matrix3d public_axis = Eigen::Matrix3d::Zero();
    public_axis(0, 1) = 1.0;
    public_axis(1, 0) = 1.0;
    public_axis(2, 2) = -1.0;
    Eigen::Matrix2d raw_image_dpublic;
    raw_image_dpublic << 0.0, 1.0,
        1.0, 0.0;
    return public_axis * draw_draw_image * raw_image_dpublic;
}

} // namespace uvdar_core::calibration::fisheye
