#include <uvdar_ros_wrapper/uv_led_detector_ros.h>

namespace uvdar {

/* UvLedDetectorComponent constructor //{ */
UvLedDetectorComponent::UvLedDetectorComponent(rclcpp::NodeOptions options) : Node("PCLFiltration", options) {
  timer_init_ = this->create_wall_timer(std::chrono::duration<double>(0.1s),
                                        std::bind(&UvLedDetectorComponent::initialize_, this));
}
//}

/* initialize_ //{ */
void UvLedDetectorComponent::initialize_() {
  node_   = this->shared_from_this();
  logger_ = std::make_shared<RosLogger>(node_->get_logger());

  loadParams_();

  initDetector_();

  initRosInterface_();

  timer_init_->cancel();
  initialized_ = true;
  RCLCPP_INFO(node_->get_logger(), "[UVDARDetector]: Initialized.");
}
//}

/* loadParams_ //{ */
void UvLedDetectorComponent::loadParams_() {
  param_loader_ = std::make_shared<mrs_lib::ParamLoader>(node_, node_->get_name());

  std::vector<std::string> config_files;
  param_loader_->loadParam("config_files", config_files);

  for (auto config_file : config_files) {
    RCLCPP_INFO(node_->get_logger(), "loading config file '%s'", config_file.c_str());
    param_loader_->addYamlFile(config_file);
  }

  loadRosParams_();
  loadUvLedDetectParams_();

  if (!param_loader_->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[UVDARDetector]: Some compulsory parameters were not loaded successfully, ending the node!");
    rclcpp::shutdown();
  }
}
//}

/* loadRosParams_ //{ */
void UvLedDetectorComponent::loadRosParams_() {
  param_loader_->loadParam("uav_name", uav_name_, std::string("uav1"));
  param_loader_->loadParam("initial_delay", initial_delay_, 5.0);

  param_loader_->loadParam("camera_input_topics", camera_topics_, std::vector<std::string>{"camera_in"});
  param_loader_->loadParam("detected_points_topics", detected_points_topics_,
                           std::vector<std::string>{"detected_points_topics"});
  param_loader_->loadParam("publish_sun_points", publish_sun_points_, false);
}
//}

/* loadUvLedDetectParams_ //{ */
void UvLedDetectorComponent::loadUvLedDetectParams_() {
  param_loader_->loadParam("uv_led_detector/use_gpu", detect_cfg_.gpu, false);
  param_loader_->loadParam("uv_led_detector/gui", detect_cfg_.gui, false);
  param_loader_->loadParam("uv_led_detector/use_masks", detect_cfg_.use_masks, false);
  param_loader_->loadParam("uv_led_detector/threshold", detect_cfg_.threshold, 200);
  param_loader_->loadParam("uv_led_detector/threshold_diff", detect_cfg_.threshold_diff, 100);
  param_loader_->loadParam("uv_led_detector/threshold_sun", detect_cfg_.threshold_sun, 150);
  param_loader_->loadParam("uv_led_detector/threshold_sun_dist", detect_cfg_.threshold_sun_dist, 25);
  param_loader_->loadParam("uv_led_detector/threshold_sun_merge", detect_cfg_.threshold_sun_merge, 20);
}
//}

/* initDetector_ //{ */
void UvLedDetectorComponent::initDetector_() {
  checkCameraInputTopics_();

  checkDetectedPointsTopics_();

  // TODO:
  if (detect_cfg_.use_masks) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "[UVDARDetector]: Masks has not been implemented yet. Shutting down!");
    return;
  }

  cameras_.clear();
  cameras_.resize(camera_count_);
  for (size_t i = 0; i < camera_count_; ++i) {
    cameras_[i].uv_detector           = std::make_unique<UvLedDetector>(*logger_, detect_cfg_);
    cameras_[i].camera_topic          = camera_topics_[i];
    cameras_[i].detected_points_topic = detected_points_topics_[i];
  }
}
//}

/* checkCameraInputTopics_ //{ */
void UvLedDetectorComponent::checkCameraInputTopics_() {
  if (camera_topics_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[UVDARDetector]:  No camera topics were supplied, ending the node!");
    rclcpp::shutdown();
  }
  camera_count_ = camera_topics_.size();
}
//}

/* checkDetectedPointsTopics_ //{ */
void UvLedDetectorComponent::checkDetectedPointsTopics_() {
  if (detected_points_topics_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[UVDARDetector]:  No detected_points topics were supplied, ending the node!");
    rclcpp::shutdown();
  }

  if (detected_points_topics_.size() != camera_count_) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "[UVDARDetector] The number of detected_points topics ("
                                                 << detected_points_topics_.size()
                                                 << ") does not match the number of cameras (" << camera_count_
                                                 << ")!");
    rclcpp::shutdown();
  }
}
//}

/* initRosInterface_ //{ */
void UvLedDetectorComponent::initRosInterface_() {
  image_callback_group_      = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  processing_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  initRosProcessImgSubs_();
  initRosPublishers_();
}
//}

/* initRosProcessImgSubs_ //{ */
void UvLedDetectorComponent::initRosProcessImgSubs_() {

  for (size_t i = 0; i < camera_count_; ++i) {
    auto& cam = cameras_.at(i);

    cam.timer = this->create_wall_timer(
        std::chrono::milliseconds(1), [this, i]() { processImage_(i); }, processing_callback_group_);
    cam.timer->cancel();

    mrs_lib::SubscriberHandlerOptions shopts;
    shopts.node                                = node_;
    shopts.node_name                           = node_->get_name();
    shopts.no_message_timeout                  = rclcpp::Duration::from_seconds(5.0);
    shopts.subscription_options.callback_group = image_callback_group_;

    // clang-format off
    cam.sub = mrs_lib::SubscriberHandler<sensor_msgs::msg::Image>(
        shopts, cam.camera_topic, [image_idx = i, this](const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
          auto& cam = cameras_[image_idx];
          {
            std::lock_guard<std::mutex> lk(cam.mtx);
            cam.last_msg = image_msg;
          }
          cameras_[image_idx].timer->reset();
        });
    // clang-format on
  }
}
//}

/* initRosPublishers_ //{ */
void UvLedDetectorComponent::initRosPublishers_() {
  mrs_lib::PublisherHandlerOptions pubopts;
  pubopts.node = node_;
  pubopts.qos  = rclcpp::QoS(1);

  for (size_t i = 0; i < camera_count_; ++i) {
    auto& cam = cameras_.at(i);

    cam.pub_detected_points = mrs_lib::PublisherHandler<uvdar_ros_interfaces::msg::ImagePointsWithFloatStamped>(
        pubopts, cam.detected_points_topic);

    if (publish_sun_points_) {
      cam.pub_sun_points = mrs_lib::PublisherHandler<uvdar_ros_interfaces::msg::ImagePointsWithFloatStamped>(
          pubopts, cam.detected_points_topic + "/sun");
    }

    cam.pub_debug_dp_image =
        mrs_lib::PublisherHandler<sensor_msgs::msg::Image>(pubopts, cam.detected_points_topic + "/raw_image");

    cam.pub_debug_sp_image =
        mrs_lib::PublisherHandler<sensor_msgs::msg::Image>(pubopts, cam.detected_points_topic + "/sun/raw_image");
#ifdef DEBUG
#endif
  }
}
//}

/* areAllCamerasDetected_ //{ */
bool UvLedDetectorComponent::areAllCamerasDetected_() {
  if (all_cameras_detected_) {
    return true;
  }

  size_t counter{0};
  for (const auto& cam : cameras_) {
    if ((cam.current_image.cols > 0) && (cam.current_image.rows > 0)) {
      counter++;
    }
  }

  if (counter == cameras_.size()) {
    all_cameras_detected_ = true;
  }

  return all_cameras_detected_;
}
//}

/* isInitDelayDone_ //{ */
bool UvLedDetectorComponent::isInitDelayDone_() {
  /*
   This delay is necessary to avoid strange segmentation faults with software
   rendering backend for OpenGL used in the buildfarm testing.
  */
  if (initial_delay_done_flag_.load(std::memory_order_acquire)) {
    return true;
  }

  std::lock_guard<std::mutex> lk(initial_delay_mtx_);
  // re-check
  if (initial_delay_done_flag_.load(std::memory_order_relaxed)) {
    return true;
  }

  if (!initial_delay_started_flag_) {
    initial_delay_started_flag_ = true;
    initial_delay_start_        = node_->get_clock()->now();
  }

  const auto now       = node_->get_clock()->now();
  const auto diff_time = (now - initial_delay_start_).seconds();

  if (diff_time < initial_delay_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "[UVDARDetector]: Ignoring message for %.1fs...", initial_delay_ - diff_time);
    return false;
  }

  initial_delay_done_flag_.store(true, std::memory_order_release);
  return true;
}
//}

/* processImage_ //{ */
void UvLedDetectorComponent::processImage_(const int image_index) {

  auto& cam = cameras_[image_index];
  sensor_msgs::msg::Image::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lk(cam.mtx);
    msg = cam.last_msg;
  }

  if (!msg) {
    cam.timer->cancel();
    return;
  }

  auto cv_ptr       = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  cam.current_image = cv_ptr->image.clone();
  cam.image_size    = cv_ptr->image.size();

  if (!areAllCamerasDetected_()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "[UVDARDetector]: Not all cameras have produced input, waiting...");
    return;
  }

  if (!isInitDelayDone_()) {
    return;
  }

  if (!cam.uv_detector->detect(cam.current_image, cam.detected_points, cam.sun_points)) {
    RCLCPP_ERROR(node_->get_logger(), "[UVDARDetector]: Failed to detect UV LEDs from camera:%d", image_index);
  }

  publishDetectedPoints_(*cv_ptr, cam);
  if (publish_sun_points_) {
    publishSunPoints_(*cv_ptr, cam);
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Number of detected points: %ld",
                       cam.detected_points.size());
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Number of sun points: %ld",
                       cam.sun_points.size());
  publishDetectedPointsImage_(*cv_ptr, cam);

  publishSunPointsImage_(*cv_ptr, cam);
#ifdef DEBUG

#endif

  cam.timer->cancel();
}
//}

/* publishDetectedPoints_ //{ */
void UvLedDetectorComponent::publishDetectedPoints_(const cv_bridge::CvImage& image, CameraContext& camera) {
  uvdar_ros_interfaces::msg::ImagePointsWithFloatStamped msg_detected;
  msg_detected.stamp        = image.header.stamp;
  msg_detected.image_width  = image.image.cols;
  msg_detected.image_height = image.image.rows;
  for (const auto& detected_point : camera.detected_points) {
    uvdar_ros_interfaces::msg::Point2DWithFloat point;
    point.x = detected_point.x;
    point.y = detected_point.y;
    msg_detected.points.push_back(point);
  }
  camera.pub_detected_points.publish(msg_detected);
}
//}

/* publishSunPoints_ //{ */
void UvLedDetectorComponent::publishSunPoints_(const cv_bridge::CvImage& image, CameraContext& camera) {
  uvdar_ros_interfaces::msg::ImagePointsWithFloatStamped msg_detected;
  msg_detected.stamp        = image.header.stamp;
  msg_detected.image_width  = image.image.cols;
  msg_detected.image_height = image.image.rows;
  for (const auto& sun_point : camera.sun_points) {
    uvdar_ros_interfaces::msg::Point2DWithFloat point;
    point.x = sun_point.x;
    point.y = sun_point.y;
    msg_detected.points.push_back(point);
  }
  camera.pub_sun_points.publish(msg_detected);
}
//}

/* publishDetectedPointsImage_ //{ */
void UvLedDetectorComponent::publishDetectedPointsImage_(const cv_bridge::CvImage& image, CameraContext& camera) {
  sensor_msgs::msg::Image msg;
  msg.header.stamp    = image.header.stamp;
  msg.header.frame_id = "camera";
  msg.height          = image.image.rows;
  msg.width           = image.image.cols;
  msg.encoding        = "mono8";
  msg.step            = msg.width;
  msg.data.assign(msg.height * msg.step, 0);

  for (const auto& detected_point : camera.detected_points) {
    int x = detected_point.x;
    int y = detected_point.y;

    if (x < 0 || x >= static_cast<int>(msg.width) || y < 0 || y >= static_cast<int>(msg.height)) {
      continue;
    }

    msg.data[y * msg.step + x] = 255;
  }
  camera.pub_debug_dp_image.publish(msg);
}
//}

/* publishSunPointsImage_ //{ */
void UvLedDetectorComponent::publishSunPointsImage_(const cv_bridge::CvImage& image, CameraContext& camera) {
  sensor_msgs::msg::Image msg;
  msg.header.stamp    = image.header.stamp;
  msg.header.frame_id = "camera";
  msg.height          = image.image.rows;
  msg.width           = image.image.cols;
  msg.encoding        = "mono8";
  msg.step            = msg.width;
  msg.data.assign(msg.height * msg.step, 0);

  for (const auto& sun_point : camera.sun_points) {
    int x = sun_point.x;
    int y = sun_point.y;

    if (x < 0 || x >= static_cast<int>(msg.width) || y < 0 || y >= static_cast<int>(msg.height)) {
      continue;
    }

    msg.data[y * msg.step + x] = 255;
  }
  camera.pub_debug_sp_image.publish(msg);
}
//}

} // namespace uvdar

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uvdar::UvLedDetectorComponent)
