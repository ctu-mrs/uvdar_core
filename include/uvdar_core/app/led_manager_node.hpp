#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <mrs_modules_msgs/msg/baca_protocol.hpp>
#include <mrs_msgs/srv/float64_srv.hpp>
#include <mrs_msgs/srv/set_int.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "uvdar_core/srv/set_ints.hpp"
#include "uvdar_core/srv/set_led_message.hpp"

namespace uvdar_core::app {

/**
 * @brief Node that drives the UVDAR LED driver board over the Baca serial protocol.
 *
 * Ported from the ROS1 uvdar_core `led_manager` nodelet. Exposes the blinking
 * sequence, frequency and message-transmission control services and, when
 * running in simulation, mirrors the same requests to the Gazebo UVDAR LED
 * plugin services.
 */
class LedManagerNode : public rclcpp::Node {
public:
    explicit LedManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    static constexpr bool kUvdarClassic = false;
    static constexpr int kGazeboLedCount = 8;

    bool loadSequenceFile(const std::string& sequence_file);

    void callbackSetActive(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response> res);
    void callbackSetFrequency(
        const std::shared_ptr<mrs_msgs::srv::Float64Srv::Request> req,
        std::shared_ptr<mrs_msgs::srv::Float64Srv::Response> res);
    void callbackLoadSequences(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void callbackLoadSingleSequence(
        const std::shared_ptr<mrs_msgs::srv::SetInt::Request> req,
        std::shared_ptr<mrs_msgs::srv::SetInt::Response> res);
    void callbackSelectSingleSequence(
        const std::shared_ptr<mrs_msgs::srv::SetInt::Request> req,
        std::shared_ptr<mrs_msgs::srv::SetInt::Response> res);
    void callbackSelectSequences(
        const std::shared_ptr<uvdar_core::srv::SetInts::Request> req,
        std::shared_ptr<uvdar_core::srv::SetInts::Response> res);
    void callbackQuickStart(
        const std::shared_ptr<mrs_msgs::srv::SetInt::Request> req,
        std::shared_ptr<mrs_msgs::srv::SetInt::Response> res);
    void callbackSetMode(
        const std::shared_ptr<mrs_msgs::srv::SetInt::Request> req,
        std::shared_ptr<mrs_msgs::srv::SetInt::Response> res);
    void callbackSetMessage(
        const std::shared_ptr<uvdar_core::srv::SetLedMessage::Request> req,
        std::shared_ptr<uvdar_core::srv::SetLedMessage::Response> res);

    std::string uav_name_;
    std::string sequence_file_;
    std::vector<std::vector<bool>> sequences_;
    bool initialized_ = false;
    int mode_ = 0;

    rclcpp::Publisher<mrs_modules_msgs::msg::BacaProtocol>::SharedPtr baca_protocol_publisher_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr serv_set_active_;
    rclcpp::Service<mrs_msgs::srv::Float64Srv>::SharedPtr serv_frequency_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr serv_load_sequences_;
    rclcpp::Service<mrs_msgs::srv::SetInt>::SharedPtr serv_load_single_sequence_;
    rclcpp::Service<mrs_msgs::srv::SetInt>::SharedPtr serv_select_single_sequence_;
    rclcpp::Service<uvdar_core::srv::SetInts>::SharedPtr serv_select_sequences_;
    rclcpp::Service<mrs_msgs::srv::SetInt>::SharedPtr serv_quick_start_;
    rclcpp::Service<mrs_msgs::srv::SetInt>::SharedPtr serv_set_mode_;
    rclcpp::Service<uvdar_core::srv::SetLedMessage>::SharedPtr serv_set_message_;

    // Mirrors of the above commands, sent to the per-LED Gazebo plugin services when simulating.
    std::vector<rclcpp::Client<mrs_msgs::srv::SetInt>::SharedPtr> clients_set_sq_gz_;
    std::vector<rclcpp::Client<mrs_msgs::srv::Float64Srv>::SharedPtr> clients_set_fr_gz_;
    std::vector<rclcpp::Client<mrs_msgs::srv::SetInt>::SharedPtr> clients_set_md_gz_;
    std::vector<rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr> clients_set_ac_gz_;
    // NOTE: the Gazebo plugin's ledMessageSender service uses uvdar_gazebo_plugin's own
    // SetLedMessage type, so it is intentionally not mirrored here to avoid a reverse
    // (uvdar_core -> uvdar_gazebo_plugin) package dependency.
};

} // namespace uvdar_core::app
