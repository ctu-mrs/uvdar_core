#pragma once

#include <rclcpp/rclcpp.hpp>

#include <deque>
#include <mutex>

#include <uvdar_ros/utils/ros_logger.h>
#include <uvdar/blink_processor/blink_processor.h>
#include <uvdar_ros_msgs/msg/image_points_with_float_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mrs_lib/node.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

namespace uvdar::blink_processor {

using namespace std::literals::chrono_literals;
using image_point_publisher_t = mrs_lib::PublisherHandler<uvdar_ros_msgs::msg::ImagePointsWithFloatStamped>;
using MarkerPointMsg          = uvdar_ros_msgs::msg::ImagePointsWithFloatStamped;

struct TrackerContext {
  std::string raw_points_topic;
  std::string detected_markers_topic;
  std::string rviz_markers_topic;

  mrs_lib::SubscriberHandler<MarkerPointMsg> sub_raw_points;
  mrs_lib::PublisherHandler<MarkerPointMsg> pub_detected_markers;
  mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray> pub_rviz_markers;
  MarkerPointMsg::ConstSharedPtr last_msg;

  std::unique_ptr<BlinkProcessor> blink_processor;

  std::shared_ptr<TimerType> timer;
  std::mutex mtx;
};

class BlinkProcessorComponent : public mrs_lib::Node {
 public:
  BlinkProcessorComponent(rclcpp::NodeOptions options);

 private:
  [[nodiscard]] bool loadParams_();
  void loadRosParams_();
  void loadBlinkProcessorParams_();

  [[nodiscard]] bool checkLoadedParams_();
  [[nodiscard]] bool checkRosTopics_() const;
  [[nodiscard]] bool checkPatternsFile_();
  [[nodiscard]] bool checkBlinkProcessorConfig_() const;

  [[nodiscard]] bool loadPatternsFile_();
  [[nodiscard]] bool initBlinkProcessor_();

  [[nodiscard]] bool initRosCommunication_();

  void processRawPoints_(const int camera_idx);

  TimePoint rosTimeToTimePoint_(const builtin_interfaces::msg::Time& ros_time);

  void publishDetectedMarkers_(const std::vector<TrackedMarker>& markers, TrackerContext& tracker,
                               const builtin_interfaces::msg::Time& time_stamp);
  void publishRvizMarkers_(const std::vector<TrackedMarker>& markers, TrackerContext& tracker);

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr receiving_callback_group_;
  rclcpp::CallbackGroup::SharedPtr processing_callback_group_;

  std::shared_ptr<RosLogger> logger_;

  std::shared_ptr<mrs_lib::ParamLoader> param_loader_;
  BlinkProcessorConfig _cfg_;
  std::string _patterns_file_path_;
  std::vector<std::string> _detected_raw_points_topics_;
  std::vector<std::string> _detected_markers_topics_;
  std::string _rviz_frame_id_;

  std::vector<Sequence> _blinking_patterns_;

  size_t camera_count_{0};
  std::deque<TrackerContext> trackers_;
};

} // namespace uvdar::blink_processor