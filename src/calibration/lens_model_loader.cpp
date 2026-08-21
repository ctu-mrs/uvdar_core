#include "uvdar_core/calibration/lens_model_loader.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "uvdar_core/calibration/fisheye/equidistant_model.hpp"
#include "uvdar_core/calibration/fisheye/ocam_model.hpp"
#include "uvdar_core/calibration/fisheye/radial_model.hpp"
#include "uvdar_core/calibration/pinhole/pinhole_model.hpp"
#include "uvdar_core/helpers/yaml.hpp"

namespace uvdar_core::calibration {

namespace yaml = uvdar_core::helpers::yaml;

namespace {

YAML::Node loadCameraConfigFile(const YAML::Node& input_node, const std::filesystem::path& config_path)
{
    const std::string calibration_file = yaml::optionalScalar<std::string>(input_node, "calib_file", std::string {});
    if (calibration_file.empty()) {
        return {};
    }

    const std::string resolved_path = yaml::resolvePath(config_path, calibration_file);
    const std::filesystem::path path(resolved_path);
    if (path.extension() != ".yaml" && path.extension() != ".yml") {
        return {};
    }
    return yaml::loadFile(resolved_path);
}

fisheye::OcamModel loadOcamYamlModel(const YAML::Node& camera_node)
{
    fisheye::OcamModel model;
    const std::vector<double> direct = yaml::optionalSequence<double>(camera_node, "direct_polynomial");
    const std::vector<double> inverse = yaml::optionalSequence<double>(camera_node, "inverse_polynomial");
    const std::vector<double> center = yaml::optionalSequence<double>(camera_node, "center");
    const std::vector<double> affine = yaml::optionalSequence<double>(camera_node, "affine");
    const std::vector<double> image_size = yaml::optionalSequence<double>(camera_node, "image_size");

    if (direct.empty() || direct.size() > static_cast<std::size_t>(fisheye::max_polynomial_length)) {
        throw std::runtime_error("OCam YAML requires direct_polynomial with 1..64 coefficients.");
    }
    if (inverse.empty() || inverse.size() > static_cast<std::size_t>(fisheye::max_polynomial_length)) {
        throw std::runtime_error("OCam YAML requires inverse_polynomial with 1..64 coefficients.");
    }
    if (center.size() != 2U) {
        throw std::runtime_error("OCam YAML requires center: [row, column].");
    }
    if (affine.size() != 3U) {
        throw std::runtime_error("OCam YAML requires affine: [c, d, e].");
    }
    if (image_size.size() != 2U) {
        throw std::runtime_error("OCam YAML requires image_size: [height, width].");
    }

    model.length_pol = static_cast<int>(direct.size());
    std::copy(direct.begin(), direct.end(), model.pol.begin());
    model.length_invpol = static_cast<int>(inverse.size());
    std::copy(inverse.begin(), inverse.end(), model.invpol.begin());
    model.xc = center[0];
    model.yc = center[1];
    model.c = affine[0];
    model.d = affine[1];
    model.e = affine[2];
    model.height = static_cast<int>(std::llround(image_size[0]));
    model.width = static_cast<int>(std::llround(image_size[1]));
    return model;
}

} // namespace

LensModelPtr loadLensModel(const YAML::Node& input_node, const std::filesystem::path& config_path)
{
    const YAML::Node camera_node = loadCameraConfigFile(input_node, config_path);
    const std::string model_type = yaml::optionalScalarAny<std::string>(camera_node, input_node, "calibration_model", "ocamcalib");
    if (model_type == "ocamcalib") {
        if (camera_node) {
            return std::make_shared<fisheye::OcamModel>(loadOcamYamlModel(camera_node));
        }
        const std::string calibration_file = yaml::resolvePath(
            config_path,
            yaml::requireScalar<std::string>(input_node, "calib_file", "camera configuration"));
        return std::make_shared<fisheye::OcamModel>(fisheye::loadModel(calibration_file));
    }

    const std::vector<double> intrinsics = yaml::optionalSequenceAny<double>(camera_node, input_node, "intrinsics");
    const std::vector<double> distortion = yaml::optionalSequenceAny<double>(camera_node, input_node, "distortion");
    const int width = yaml::optionalScalarAny<int>(camera_node, input_node, "image_width", 0);
    const int height = yaml::optionalScalarAny<int>(camera_node, input_node, "image_height", 0);
    if (intrinsics.size() < 4U) {
        throw std::runtime_error("Calibration model '" + model_type + "' requires intrinsics: [fx, fy, cx, cy].");
    }

    if (model_type == "pinhole") {
        pinhole::PinholeModel::Parameters parameters;
        parameters.fx = intrinsics[0];
        parameters.fy = intrinsics[1];
        parameters.cx = intrinsics[2];
        parameters.cy = intrinsics[3];
        parameters.width = width;
        parameters.height = height;
        if (distortion.size() > 0U) parameters.k1 = distortion[0];
        if (distortion.size() > 1U) parameters.k2 = distortion[1];
        if (distortion.size() > 2U) parameters.p1 = distortion[2];
        if (distortion.size() > 3U) parameters.p2 = distortion[3];
        if (distortion.size() > 4U) parameters.k3 = distortion[4];
        return std::make_shared<pinhole::PinholeModel>(parameters);
    }

    if (model_type == "fisheye_equidistant" || model_type == "equidistant") {
        fisheye::EquidistantModel::Parameters parameters;
        parameters.fx = intrinsics[0];
        parameters.fy = intrinsics[1];
        parameters.cx = intrinsics[2];
        parameters.cy = intrinsics[3];
        parameters.width = width;
        parameters.height = height;
        if (distortion.size() > 0U) parameters.k1 = distortion[0];
        if (distortion.size() > 1U) parameters.k2 = distortion[1];
        if (distortion.size() > 2U) parameters.k3 = distortion[2];
        if (distortion.size() > 3U) parameters.k4 = distortion[3];
        return std::make_shared<fisheye::EquidistantModel>(parameters);
    }

    if (model_type == "fisheye_equisolid" || model_type == "equisolid" || model_type == "equisolid_angle"
        || model_type == "fisheye_stereographic" || model_type == "stereographic"
        || model_type == "fisheye_orthographic" || model_type == "orthographic") {
        fisheye::RadialModel::Parameters parameters;
        if (model_type == "fisheye_stereographic" || model_type == "stereographic") {
            parameters.projection = fisheye::RadialModel::Projection::Stereographic;
        } else if (model_type == "fisheye_orthographic" || model_type == "orthographic") {
            parameters.projection = fisheye::RadialModel::Projection::Orthographic;
        } else {
            parameters.projection = fisheye::RadialModel::Projection::EquisolidAngle;
        }
        parameters.fx = intrinsics[0];
        parameters.fy = intrinsics[1];
        parameters.cx = intrinsics[2];
        parameters.cy = intrinsics[3];
        parameters.width = width;
        parameters.height = height;
        if (distortion.size() > 0U) parameters.k1 = distortion[0];
        if (distortion.size() > 1U) parameters.k2 = distortion[1];
        if (distortion.size() > 2U) parameters.k3 = distortion[2];
        if (distortion.size() > 3U) parameters.k4 = distortion[3];
        return std::make_shared<fisheye::RadialModel>(parameters);
    }

    throw std::runtime_error("Unsupported calibration_model '" + model_type + "'.");
}

} // namespace uvdar_core::calibration
