#include <uvdar_ros/blink_processor_ros.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>

namespace uvdar::blink_processor {

/* BlinkProcessorComponent //{ */
BlinkProcessorComponent::BlinkProcessorComponent(rclcpp::NodeOptions options)
    : mrs_lib::Node("BlinkProcessor", options) {
  node_   = this_node_ptr();
  logger_ = std::make_shared<RosLogger>(node_->get_logger());

  if (!loadParams_()) {
    throw std::runtime_error("Failed to load parameters from the config file!");
  }

  if (!checkLoadedParams_()) {
    throw std::runtime_error("Some loaded parameters are not valid!");
  }

  if (!loadPatternsFile_()) {
    throw std::runtime_error("Failed to load blinking patterns!");
  }

  if (!initBlinkProcessor_()) {
    throw std::runtime_error("Failed to initialize blink processor!");
  }

  if (!initRosCommunication_()) {
    throw std::runtime_error("Failed to initialize ROS communication!");
  }

  RCLCPP_INFO(node_->get_logger(), "BlinkProcessor node initialized successfully.");
}
//}

/* loadParams_ //{ */
bool BlinkProcessorComponent::loadParams_() {
  param_loader_ = std::make_shared<mrs_lib::ParamLoader>(node_, node_->get_name());

  std::vector<std::string> config_files;
  param_loader_->loadParam("config_files", config_files);
  for (auto config_file : config_files) {
    RCLCPP_INFO(node_->get_logger(), "Loading config file '%s'", config_file.c_str());
    param_loader_->addYamlFile(config_file);
  }

  try {
    loadRosParams_();
    loadBlinkProcessorParams_();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load parameters: %s", e.what());
    return false;
  }

  if (!param_loader_->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Some compulsory parameters were not loaded successfully!");
    return false;
  }

  return true;
}
//}

/* loadRosParams_ //{ */
void BlinkProcessorComponent::loadRosParams_() {
  param_loader_->loadParam("uv_led_detector/detected_points_topics", _detected_raw_points_topics_);
  param_loader_->loadParam("blink_processor/detected_markers_topics", _detected_markers_topics_);
}
//}

/* loadBlinkProcessorParams_ //{ */
void BlinkProcessorComponent::loadBlinkProcessorParams_() {
  param_loader_->loadParam("blink_processor/patterns_file", _patterns_file_path_);

  param_loader_->loadParam("blink_processor/allowed_BER_per_sequence", _cfg_.allowed_BER_per_seq);
  param_loader_->loadParam("blink_processor/polynomial_degree", _cfg_.poly_order);
  param_loader_->loadParam("blink_processor/decay_factor", _cfg_.poly_decay_factor);
  param_loader_->loadParam("blink_processor/stored_seq_len_factor", _cfg_.seq.stored_seq_len_factor);
  param_loader_->loadParam("blink_processor/confidence_probability_percentage", _cfg_.conf_prob_percentage);
  param_loader_->loadParam("blink_processor/max_buffer_length", _cfg_.max_buffer_length);
  param_loader_->loadParam("blink_processor/max_consecutive_zeros", _cfg_.max_consecutive_zeros);
  param_loader_->loadParam("blink_processor/min_prediction_tol_px", _cfg_.min_prediction_tol_px);
}
//}

/* checkLoadedParams_ //{ */
bool BlinkProcessorComponent::checkLoadedParams_() {
  if (!checkRosTopics_()) {
    RCLCPP_ERROR(node_->get_logger(), "Some ROS topics are not defined!");
    return false;
  }

  if (!checkPatternsFile_()) {
    RCLCPP_ERROR(node_->get_logger(), "Blinking patterns file is not valid!");
    return false;
  }

  if (!checkBlinkProcessorConfig_()) {
    RCLCPP_ERROR(node_->get_logger(), "Blink processor configuration parameters are not valid!");
    return false;
  }
  return true;
}
//}

/* checkRosTopics_ //{ */
bool BlinkProcessorComponent::checkRosTopics_() const {
  if (_detected_raw_points_topics_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No detected points topics specified!");
    return false;
  }

  if (_detected_markers_topics_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No detected markers topics specified!");
    return false;
  }

  return true;
}
//}

/* checkPatternsFile_ //{ */
bool BlinkProcessorComponent::checkPatternsFile_() {
  if (_patterns_file_path_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No blinking patterns file specified!");
    return false;
  }

  std::filesystem::path file_path(_patterns_file_path_);
  if (!file_path.is_absolute()) {
    std::string package_share_dir;
    try {
      package_share_dir = ament_index_cpp::get_package_share_directory("uvdar_ros");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to find package 'uvdar_ros': %s", e.what());
      return false;
    }
    file_path            = std::filesystem::path(package_share_dir) / "config" / _patterns_file_path_;
    _patterns_file_path_ = file_path.string();
  }

  std::ifstream f(_patterns_file_path_);
  if (!f.good()) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Patterns file '%s' does not exist or is not readable! If it starts with /, it is treated as an absolute path; "
        "otherwise, it is treated as relative to the 'config' directory of the 'uvdar_ros' package.",
        _patterns_file_path_.c_str());
    return false;
  }

  return true;
}
//}

/* checkBlinkProcessorConfig_ //{ */
bool BlinkProcessorComponent::checkBlinkProcessorConfig_() const {
  if (_cfg_.poly_order < 0 || _cfg_.poly_order > 4) {
    RCLCPP_ERROR(node_->get_logger(), "Polynomial degree must be in the range [0, 4]!");
    return false;
  }

  if (_cfg_.allowed_BER_per_seq < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Allowed BER per sequence cannot be negative!");
    return false;
  }

  if (_cfg_.seq.stored_seq_len_factor <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "Stored sequence length factor must be positive!");
    return false;
  }

  if (_cfg_.conf_prob_percentage <= 0 || _cfg_.conf_prob_percentage >= 100) {
    RCLCPP_ERROR(node_->get_logger(), "Confidence probability percentage must be in the range (0, 100)!");
    return false;
  }

  if (_cfg_.min_prediction_tol_px < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Minimum prediction tolerance in pixels cannot be negative!");
    return false;
  }

  if (_cfg_.max_consecutive_zeros < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Maximum consecutive zeros cannot be negative!");
    return false;
  }

  if (_cfg_.max_buffer_length < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Maximum buffer length cannot be negative!");
    return false;
  }

  if (_cfg_.poly_decay_factor < 0.0 || _cfg_.poly_decay_factor > 100.0) {
    RCLCPP_ERROR(node_->get_logger(), "Polynomial decay factor must be in the range [0.0, 100.0]!");
    return false;
  }

  return true;
}
//}

/* loadPatternsFile_ //{ */
bool BlinkProcessorComponent::loadPatternsFile_() {
  std::ifstream file(_patterns_file_path_);
  if (!file.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open patterns file '%s'!", _patterns_file_path_.c_str());
    return false;
  }

  _blinking_patterns_.clear();

  std::string line;
  int line_num = 0;
  while (std::getline(file, line)) {
    ++line_num;

    if (line.empty() || line[0] == '#') {
      continue;
    }

    Sequence seq;
    std::istringstream iss(line);
    std::string token;
    while (std::getline(iss, token, ',')) {
      try {
        int val = std::stoi(token);
        if (val != 0 && val != 1) {
          RCLCPP_ERROR(node_->get_logger(), "Invalid value '%s' on line %d — expected 0 or 1", token.c_str(), line_num);
          return false;
        }
        seq.push_back(val != 0);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid value '%s' on line %d of patterns file: %s", token.c_str(), line_num,
                     e.what());
        return false;
      }
    }
    _blinking_patterns_.push_back(seq);
  }

  if (_blinking_patterns_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No patterns found in file '%s'!", _patterns_file_path_.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Loaded %zu blinking patterns from '%s'", _blinking_patterns_.size(),
              _patterns_file_path_.c_str());
  return true;
}
//}

/* initBlinkProcessor_ //{ */
bool BlinkProcessorComponent::initBlinkProcessor_() {
  if (_blinking_patterns_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot initialize blink processor: no blinking patterns loaded!");
    return false;
  }

  blink_processor_ = std::make_unique<BlinkProcessor>(_cfg_, *logger_);
  if (!blink_processor_->setBlinkingPatterns(_blinking_patterns_)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to set blinking patterns in blink processor!");
    return false;
  }

  return true;
}
//}

bool BlinkProcessorComponent::initRosCommunication_() {

  return true;
}
//}

} // namespace uvdar::blink_processor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uvdar::blink_processor::BlinkProcessorComponent)
