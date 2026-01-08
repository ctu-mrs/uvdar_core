#include <ros/ros.h>
#include <ros/package.h>
#include <mrs_lib/param_loader.h>
#include <mrs_msgs/SetInt.h>
#include <string>
#include <cmath>
#include <mrs_modules_msgs/BacaProtocol.h>
#include <uvdar_core/SetLedState.h>

namespace fr_setter
{

class FrSetter {
private:
  std::string                     uav_name;
  ros::ServiceServer              service_set_fr;
  ros::Publisher                  baca_protocol_publisher;
  std::vector<ros::ServiceClient> clients_set_fr;
  std::vector<int>                data_frame;
  bool                            initialized = false;

public:
  FrSetter(ros::NodeHandle& nh) {
    mrs_lib::ParamLoader param_loader(nh, "FrSetter");

    param_loader.loadParam("uav_name", uav_name);
    service_set_fr          = nh.advertiseService("set_frequency", &FrSetter::FrequencySetterCallback, this);
    baca_protocol_publisher = nh.advertise<mrs_modules_msgs::BacaProtocol>("baca_protocol_out", 1);

    for (int i = 0; i < 8; i++) {
      clients_set_fr.push_back(nh.serviceClient<mrs_msgs::SetInt>("/gazebo/ledFrequencySetter/" + uav_name + "_uvled_" + std::to_string(i + 1) + "_lens_link"));
    }

    initialized = true;
  }

private:
  bool FrequencySetterCallback(uvdar_core::SetLedState::Request& req, uvdar_core::SetLedState::Response& res) {
    mrs_modules_msgs::BacaProtocol serial_msg;
    serial_msg.stamp = ros::Time::now();
        
    mrs_msgs::SetInt led_state;
    if (req.data) {
      if (req.start_of_message) {
        data_frame = req.data_frame;
        serial_msg.payload.push_back(0x93);
        for (auto& bit : data_frame) {
          serial_msg.payload.push_back(bit);
        }
        baca_protocol_publisher.publish(serial_msg);
        // send through baca protokol

        ROS_INFO("New data frame received");
        for (auto& bits : data_frame) {
          // ROS_INFO("Led on");
          std::cout << bits;
        }
        std::cout << std::endl;
      }
      if (req.bit_value) {
        // set max freq
        led_state.request.value = 0;
        for (auto& client : clients_set_fr)
          client.call(led_state);
        // ROS_INFO("Led on");
      } else {
        // Turn lights off
        led_state.request.value = INT_MAX;
        for (auto& client : clients_set_fr)
          client.call(led_state);
        // ROS_INFO("Led off");
      }
    } else {
      serial_msg.payload.push_back(0x92);
      if (req.multi_frequency) {
        for (int i = 0; i < 8; i++) {
          led_state.request.value = req.frequencies[i];
          serial_msg.payload.push_back(led_state.request.value);
          clients_set_fr[i].call(led_state);
        }
        // ROS_INFO("Blinking multi");
        // send through baca protokol

      } else {
        // set the same frequency
        led_state.request.value = req.frequency;
        for (auto& client : clients_set_fr) {
          client.call(led_state);
          serial_msg.payload.push_back(led_state.request.value);
        }
        // ROS_INFO("Blinking one");
        // send through baca protokol
      }
      baca_protocol_publisher.publish(serial_msg);
    }

    res.success = true;
    return true;
  }
};
}  // namespace fr_setter

int main(int argc, char** argv) {
  ros::init(argc, argv, "FrSetter");
  ros::NodeHandle     nh("~");
  fr_setter::FrSetter fr(nh);
  ROS_INFO("[UVDAR FrSetter]: frequency setter node initiated");
  ros::spin();
  return 0;
}
