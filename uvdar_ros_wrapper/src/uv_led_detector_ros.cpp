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
  param_loader_->loadParam("publish_visualization", publish_visualization_flag_, false);
  param_loader_->loadParam("initial_delay", initial_delay_, 5.0);

  param_loader_->loadParam("camera_topics", camera_topics_, std::vector<std::string>{"camera_in"});
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
  if (camera_topics_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[UVDARDetector]:  No camera topics were supplied, ending the node!");
    rclcpp::shutdown();
  }
  camera_count_ = camera_topics_.size();

  if (detect_cfg_.use_masks) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "[UVDARDetector]: Masks has not been implemented yet. Shutting down!");
    return;
  }

  uv_detector_ = std::make_unique<UvLedDetector>(*logger_, detect_cfg_);
}
//}

/* initRosInterface_ //{ */
void UvLedDetectorComponent::initRosInterface_() {
  cameras_.clear();
  cameras_.resize(camera_count_);

  image_callback_group_      = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  processing_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  for (size_t i = 0; i < camera_count_; ++i) {
    auto& cam = cameras_.at(i);
    cam.topic = camera_topics_.at(i);

    cam.timer = this->create_wall_timer(
        std::chrono::milliseconds(0), [this, i]() { processImage_(i); }, processing_callback_group_);
    cam.timer->cancel();

    mrs_lib::SubscriberHandlerOptions shopts;
    shopts.node                                = node_;
    shopts.node_name                           = node_->get_name();
    shopts.no_message_timeout                  = rclcpp::Duration::from_seconds(5.0);
    shopts.subscription_options.callback_group = image_callback_group_;

    // clang-format off
    cam.sub = mrs_lib::SubscriberHandler<sensor_msgs::msg::Image>(
        shopts, cam.topic, [image_idx = i, this](const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
          auto& cam = cameras_[image_idx];
          {
            std::lock_guard<std::mutex> lk(cam.mtx);
            cam.last_msg = std::move(image_msg);
          }
          cameras_[image_idx].timer->reset();
        });
    // clang-format on
  }

  mrs_lib::PublisherHandlerOptions pubopts;
  pubopts.node = node_;
  pubopts.qos  = rclcpp::QoS(1);

  pub_detected_points = mrs_lib::PublisherHandler<uvdar_ros_interfaces::msg::ImagePointsWithFloatStamped>(
      pubopts, "~/detected_points_out");

  debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/debug_image", rclcpp::SensorDataQoS());
}
//}

/* callbackImage_ //{ */
void UvLedDetectorComponent::processImage_(const int image_index) {

  auto& camera = cameras_[image_index];
  sensor_msgs::msg::Image::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lk(camera.mtx);
    msg = camera.last_msg;

    if (!msg) {
      camera.timer->cancel();
      return;
    }

    auto cv_ptr          = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    camera.current_image = cv_ptr->image.clone();
    camera.image_size    = cv_ptr->image.size();

    if (!uv_detector_->detect(camera.current_image, camera.detected_points, camera.sun_points)) {
      RCLCPP_ERROR(node_->get_logger(), "[UVDARDetector]: Failed to detect UV LEDs from camera:%d", image_index);
    }

    uvdar_ros_interfaces::msg::ImagePointsWithFloatStamped msg_detected;
    msg_detected.stamp        = cv_ptr->header.stamp;
    msg_detected.image_width  = cv_ptr->image.cols;
    msg_detected.image_height = cv_ptr->image.rows;
    for (auto& detected_point : camera.detected_points) {
      uvdar_ros_interfaces::msg::Point2DWithFloat point;
      point.x = detected_point.x;
      point.y = detected_point.y;
      msg_detected.points.push_back(point);
    }
    pub_detected_points.publish(msg_detected);

    // ============================
    // TODO: remove
    sensor_msgs::msg::Image msg;
    msg.header.stamp    = this->now();
    msg.header.frame_id = "camera";
    msg.height          = cv_ptr->image.rows;
    msg.width           = cv_ptr->image.cols;
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
    debug_pub_->publish(msg);
  }
  camera.timer->cancel();
}
//}

} // namespace uvdar

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uvdar::UvLedDetectorComponent)