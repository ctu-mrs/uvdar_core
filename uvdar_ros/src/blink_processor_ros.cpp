#include <uvdar_ros/blink_processor_ros.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>

namespace uvdar::blink_processor {

using namespace std::literals::chrono_literals;

/* BlinkProcessorComponent //{ */
BlinkProcessorComponent::BlinkProcessorComponent(rclcpp::NodeOptions options)
    : mrs_lib::Node("BlinkProcessor", options) {
  node_   = this_node_ptr();
  logger_ = std::make_shared<RosLogger>(node_->get_logger());

  loadParams_();

  checkLoadedParams_();

  loadPatternsFile_();

  checkCameraCalibrationFiles_();

  initBlinkProcessor_();

  initRosCommunication_();

  RCLCPP_INFO(node_->get_logger(), "BlinkProcessor node initialized successfully.");
}
//}

/* loadParams_ //{ */
void BlinkProcessorComponent::loadParams_() {
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
    throw std::runtime_error("Failed to load parameters!");
  }

  if (!param_loader_->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Some compulsory parameters were not loaded successfully!");
    throw std::runtime_error("Failed to load parameters!");
  }
}
//}

/* loadRosParams_ //{ */
void BlinkProcessorComponent::loadRosParams_() {
  param_loader_->loadParam("uv_led_detector/detected_points_topics", _detected_raw_points_topics_);
  param_loader_->loadParam("blink_processor/detected_markers_topics", _detected_markers_topics_);
  param_loader_->loadParam("blink_processor/rviz_frame_id", _rviz_frame_id_);
}
//}

/* loadBlinkProcessorParams_ //{ */
void BlinkProcessorComponent::loadBlinkProcessorParams_() {
  param_loader_->loadParam("blink_processor/debug", _debug_mode_);
  param_loader_->loadParam("blink_processor/patterns_file", _patterns_file_path_);
  param_loader_->loadParam("blink_processor/camera_calibration_file", _camera_calib_files_);
  param_loader_->loadParam("blink_processor/allowed_BER_per_sequence", _cfg_.allowed_BER_per_seq);
  param_loader_->loadParam("blink_processor/polynomial_degree", _cfg_.poly_order);
  param_loader_->loadParam("blink_processor/decay_factor", _cfg_.poly_decay_factor);
  param_loader_->loadParam("blink_processor/stored_seq_len_factor", _cfg_.seq.stored_seq_len_factor);
  param_loader_->loadParam("blink_processor/confidence_probability_percentage", _cfg_.conf_prob_percentage);
  param_loader_->loadParam("blink_processor/max_buffer_length", _cfg_.max_buffer_length);
  param_loader_->loadParam("blink_processor/max_consecutive_zeros", _cfg_.max_consecutive_zeros);
  param_loader_->loadParam("blink_processor/min_prediction_tol_px", _cfg_.min_prediction_tol_px);
  param_loader_->loadParam("blink_processor/max_prediction_interval_px", _cfg_.max_predict_interval_px);

  int max_px_shift{0};
  param_loader_->loadParam("blink_processor/max_px_shift", max_px_shift);
  _cfg_.max_px_shift = cv::Point2d(max_px_shift, max_px_shift);
}
//}

/* checkLoadedParams_ //{ */
void BlinkProcessorComponent::checkLoadedParams_() {
  if (!checkRosTopics_()) {
    RCLCPP_ERROR(node_->get_logger(), "Some ROS topics are not defined!");
    throw std::runtime_error("Some ROS topics are not defined!");
  }

  if (!checkPatternsFile_()) {
    RCLCPP_ERROR(node_->get_logger(), "Blinking patterns file is not valid!");
    throw std::runtime_error("Blinking patterns file is not valid!");
  }

  if (!checkBlinkProcessorConfig_()) {
    RCLCPP_ERROR(node_->get_logger(), "Blink processor configuration parameters are not valid!");
    throw std::runtime_error("Blink processor configuration parameters are not valid!");
  }
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

  if (_detected_raw_points_topics_.size() != _detected_markers_topics_.size()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "The number of detected points topics must match the number of detected markers topics!");
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

/* checkCameraCalibrationFiles_ */
void BlinkProcessorComponent::checkCameraCalibrationFiles_() {
  if (_camera_calib_files_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No camera calibration files specified!");
    throw std::runtime_error("No camera calibration files specified!");
  }

  if (_camera_calib_files_.size() != _detected_raw_points_topics_.size()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "The camera calibration files count must match the detected points topics count!");
    throw std::runtime_error("Camera calibration files count does not match detected points topics count!");
  }

  for (auto& calib_file : _camera_calib_files_) {
    std::filesystem::path file_path(calib_file);
    if (!file_path.is_absolute()) {
      std::string package_share_dir;
      try {
        package_share_dir = ament_index_cpp::get_package_share_directory("uvdar_ros");
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to find package 'uvdar_ros': %s", e.what());
        throw std::runtime_error("Failed to find package 'uvdar_ros': " + std::string(e.what()));
      }
      file_path = std::filesystem::path(package_share_dir) / "config" / calib_file;
    }

    std::ifstream f(file_path);
    if (!f.good()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Camera calibration file '%s' does not exist or is not readable! If it starts with /, it is treated "
                   "as an absolute path; "
                   "otherwise, it is treated as relative to the 'config' directory of the 'uvdar_ros' package.",
                   calib_file.c_str());
      throw std::runtime_error("Camera calibration file '" + calib_file + "' does not exist or is not readable!");
    }

    calib_file = file_path.string();
  }
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

  if (_cfg_.max_px_shift.x < 0.0 || _cfg_.max_px_shift.y < 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "Max pixel shift cannot be negative!");
    return false;
  }

  if (_cfg_.max_predict_interval_px < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Maximum prediction interval for the extended search cannot be negative!");
    return false;
  }

  return true;
}
//}

/* loadPatternsFile_ //{ */
void BlinkProcessorComponent::loadPatternsFile_() {
  std::ifstream file(_patterns_file_path_);
  if (!file.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open patterns file '%s'!", _patterns_file_path_.c_str());
    throw std::runtime_error("Failed to open patterns file: '" + _patterns_file_path_ + "'!");
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
          throw std::runtime_error("Invalid value '" + token + "' on line " + std::to_string(line_num) +
                                   " — expected 0 or 1");
        }
        seq.push_back(val != 0);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid value '%s' on line %d of patterns file: %s", token.c_str(), line_num,
                     e.what());
        throw std::runtime_error("Invalid value '" + token + "' on line " + std::to_string(line_num) +
                                 " of patterns file: " + e.what());
      }
    }
    _blinking_patterns_.push_back(seq);
  }

  if (_blinking_patterns_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No patterns found in file '%s'!", _patterns_file_path_.c_str());
    throw std::runtime_error("No patterns found in file '" + _patterns_file_path_ + "'!");
  }

  RCLCPP_INFO(node_->get_logger(), "Loaded %zu blinking patterns from '%s'", _blinking_patterns_.size(),
              _patterns_file_path_.c_str());
}
//}

/* initBlinkProcessor_ //{ */
void BlinkProcessorComponent::initBlinkProcessor_() {
  if (_blinking_patterns_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot initialize blink processor: no blinking patterns loaded!");
    throw std::runtime_error("Cannot initialize blink processor: no blinking patterns loaded!");
  }

  camera_count_ = _detected_raw_points_topics_.size();
  trackers_.clear();
  trackers_.resize(camera_count_);
  for (size_t i = 0; i < camera_count_; ++i) {
    trackers_[i].raw_points_topic       = _detected_raw_points_topics_[i];
    trackers_[i].detected_markers_topic = _detected_markers_topics_[i];
    trackers_[i].blink_processor        = std::make_unique<BlinkProcessor>(_cfg_, *logger_);
    trackers_[i].camera_calib =
        std::make_unique<CameraCalibration>(CameraCalibration::loadCalibration(_camera_calib_files_[i]));

    if (!trackers_[i].blink_processor->setBlinkingPatterns(_blinking_patterns_)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to set blinking patterns in blink processor!");
      throw std::runtime_error("Failed to set blinking patterns in blink processor!");
    }
  }
}
//}

/* initRosCommunication_ //{ */
void BlinkProcessorComponent::initRosCommunication_() {
  receiving_callback_group_  = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  processing_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  for (size_t i = 0; i < camera_count_; ++i) {
    initRosSubscribers_(i);
    initRosPublishers_(i);
  }
}
//}

/* initRosSubscribers_ //{ */
void BlinkProcessorComponent::initRosSubscribers_(const size_t tracker_idx) {
  auto& tracker = trackers_.at(tracker_idx);

  mrs_lib::TimerHandlerOptions timer_opts_start;
  timer_opts_start.node      = node_;
  timer_opts_start.autostart = true;

  tracker.timer = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(std::chrono::milliseconds(1)),
                                              [this, tracker_idx]() { processRawPoints_(tracker_idx); });
  tracker.timer->stop();

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = node_;
  shopts.node_name                           = node_->get_name();
  shopts.no_message_timeout                  = rclcpp::Duration::from_seconds(5.0);
  shopts.subscription_options.callback_group = receiving_callback_group_;

  tracker.sub_raw_points = mrs_lib::SubscriberHandler<RawImagePointArrayMsg>(
      shopts, tracker.raw_points_topic,
      [camera_idx = tracker_idx, this](const RawImagePointArrayMsg::ConstSharedPtr& points_msg) {
        auto& tracker = trackers_[camera_idx];
        {
          std::lock_guard<std::mutex> lk(tracker.mtx);
          tracker.last_msg = points_msg;
        }
        tracker.timer->start();
      });
}
//}

/* initRosPublishers_ //{ */
void BlinkProcessorComponent::initRosPublishers_(const size_t tracker_idx) {
  auto& tracker = trackers_.at(tracker_idx);

  mrs_lib::PublisherHandlerOptions pubopts;
  pubopts.node = node_;
  pubopts.qos  = rclcpp::QoS(1);
  tracker.pub_detected_markers =
      mrs_lib::PublisherHandler<MarkerPointArrayMsg>(pubopts, tracker.detected_markers_topic);

  if (_debug_mode_) {
    tracker.rviz_markers_topic = tracker.detected_markers_topic + "/rviz";

    tracker.pub_rviz_markers =
        mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray>(pubopts, tracker.rviz_markers_topic);

    tracker.pub_debug_image =
        mrs_lib::PublisherHandler<sensor_msgs::msg::Image>(pubopts, tracker.detected_markers_topic + "/debug_image");
  }
}
//}

/* processRawPoints_ //{ */
void BlinkProcessorComponent::processRawPoints_(const int camera_idx) {
  auto& tracker = trackers_[camera_idx];
  RawImagePointArrayMsg::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lk(tracker.mtx);
    msg              = tracker.last_msg;
    tracker.last_msg = nullptr;
  }

  if (!msg) {
    tracker.timer->stop();
    return;
  }

  std::vector<PointState> unassigned_points;
  unassigned_points.reserve(msg->points.size());
  for (auto point_time_stamp : msg->points) {
    PointState p;
    p.point       = cv::Point2d(point_time_stamp.x, point_time_stamp.y);
    p.led_state   = true;
    p.insert_time = rosTimeToTimePoint_(msg->stamp);
    unassigned_points.push_back(std::move(p));
  }
  tracker.blink_processor->processBuffer(unassigned_points);

  std::vector<TrackedMarker> results = tracker.blink_processor->getResults();

  publishDetectedMarkers_(results, tracker, msg->stamp);

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Number of detected markers: %ld",
                       results.size());

  if (_debug_mode_) {
    publishDebugImage_(results, tracker, msg->stamp);
    publishRvizMarkers_(results, tracker);
  }

  tracker.timer->stop();
}
//}

/* rosTimeToTimePoint_ //{ */
TimePoint BlinkProcessorComponent::rosTimeToTimePoint_(const builtin_interfaces::msg::Time& ros_time) {
  rclcpp::Time rcl_time(ros_time);

  return TimePoint(std::chrono::nanoseconds(rcl_time.nanoseconds()));
}
//}

/* publishDetectedMarkers_ //{ */
void BlinkProcessorComponent::publishDetectedMarkers_(const std::vector<TrackedMarker>& markers,
                                                      TrackerContext& tracker,
                                                      const builtin_interfaces::msg::Time& time_stamp) {
  MarkerPointArrayMsg msg;
  msg.stamp = time_stamp;
  msg.points.reserve(markers.size());
  for (const auto& marker : markers) {
    // auto id = static_cast<double>(marker.id);
    // if (id < 0) {
    //   continue;
    // }

    auto world_point = tracker.camera_calib->camToWorld(marker.last_point.point);
    MarkerPointMsg point;
    point.x  = world_point.x;
    point.y  = world_point.y;
    point.z  = world_point.z;
    point.id = marker.id;
    msg.points.push_back(point);
  }

  tracker.pub_detected_markers.publish(msg);
}
//}

/* publishRvizMarkers_ //{ */
void BlinkProcessorComponent::publishRvizMarkers_(const std::vector<TrackedMarker>& markers, TrackerContext& tracker) {
  if (!_debug_mode_) {
    return;
  }
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id    = _rviz_frame_id_;
  marker.header.stamp       = node_->now();
  marker.ns                 = "tracked_markers";
  marker.id                 = 0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  marker.points.reserve(markers.size());
  for (const auto& tracked_marker : markers) {
    if (tracked_marker.id < 0) {
      continue;
    }

    auto world_point = tracker.camera_calib->camToWorld(tracked_marker.last_point.point);

    geometry_msgs::msg::Point point;
    point.x = world_point.x;
    point.y = world_point.y;
    point.z = world_point.z;
    marker.points.push_back(point);
  }

  if (marker.points.empty()) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker.type   = visualization_msgs::msg::Marker::SPHERE_LIST;
  } else {
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type   = visualization_msgs::msg::Marker::SPHERE_LIST;
  }

  marker_array.markers.push_back(marker);
  tracker.pub_rviz_markers.publish(marker_array);
}
//}

/* publishDebugImage_ //{ */
void BlinkProcessorComponent::publishDebugImage_(const std::vector<TrackedMarker>& markers, TrackerContext& tracker,
                                                 const builtin_interfaces::msg::Time& time_stamp) {
  if (!_debug_mode_) {
    return;
  }
  const int kWidth  = tracker.camera_calib->model.width;
  const int kHeight = tracker.camera_calib->model.height;

  cv::Mat frame = cv::Mat::zeros(kHeight, kWidth, CV_8UC3);

  for (const auto& m : markers) {
    const auto& ps = m.last_point;

    cv::Point pt;
    if (ps.x_stats.poly_reg_computed && ps.y_stats.poly_reg_computed) {
      pt = cv::Point(static_cast<int>(ps.x_stats.predicted_coordinate),
                     static_cast<int>(ps.y_stats.predicted_coordinate));
    } else {
      pt = cv::Point(static_cast<int>(ps.point.x), static_cast<int>(ps.point.y));
    }

    cv::Scalar color = (m.id < 0) ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);

    cv::circle(frame, pt, 5, color, cv::FILLED);

    std::string label = "id=" + std::to_string(m.id);
    cv::putText(frame, label, cv::Point(pt.x + 6, pt.y - 4), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
  }

  auto img_msg             = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
  img_msg->header.stamp    = time_stamp;
  img_msg->header.frame_id = _rviz_frame_id_;
  tracker.pub_debug_image.publish(*img_msg);
}
//}

} // namespace uvdar::blink_processor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uvdar::blink_processor::BlinkProcessorComponent)
