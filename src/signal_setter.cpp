#include <ros/ros.h>
#include <ros/package.h>
#include <mrs_lib/param_loader.h>
#include <mrs_msgs/SetInt.h>
#include <string>
#include <cmath>
#include <mrs_msgs/BacaProtocol.h>
#include <uvdar_core/SetLedState.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
/* #include <uvdar_core/SignalSetter.h> */

std::string                     uav_name;
ros::ServiceServer              service_set_fr;
ros::Publisher                  baca_protocol_publisher;
std::vector<ros::ServiceClient> clients_set_fr;
std::vector<int>                data_frame;
bool                            initialized = false;
bool simulation = true;

namespace signal_setter
{

class SignalSetter : public nodelet::Nodelet {
public:
  void onInit() {
    ros::NodeHandle nh("~");
    
    mrs_lib::ParamLoader param_loader(nh, "~");
    param_loader.loadParam("uav_name", uav_name);
    param_loader.loadParam("simulation", simulation);
    service_set_fr          = nh.advertiseService("set_frequency", &SignalSetter::FrequencySetterCallback, this);
    baca_protocol_publisher = nh.advertise<mrs_msgs::BacaProtocol>("baca_protocol_out", 1);

    for (int i = 0; i < 8; i++) {
      clients_set_fr.push_back(nh.serviceClient<mrs_msgs::SetInt>("/gazebo/ledSignalSetter/" + uav_name + "_uvled_" + std::to_string(i + 1) + "_lens_link"));
    }

    initialized = true;

    ROS_INFO("[UVDAR SignalSetter]: signal setter node v2 initiated %s, %d", uav_name.c_str(), simulation);
  }

private:

  bool FrequencySetterCallback(uvdar_core::SetLedState::Request& req, uvdar_core::SetLedState::Response& res) {
    mrs_msgs::BacaProtocol serial_msg;
    serial_msg.stamp = ros::Time::now();

    mrs_msgs::SetInt led_state;

    if (req.data) {
      if (req.start_of_message) {
        data_frame = req.data_frame;
        serial_msg.payload.push_back(0x93);
        
        ROS_INFO("Sended data frame ");
        
        for (auto& bit : data_frame) {
          for(int i = 0; i < 3; i++){
            serial_msg.payload.push_back(bit);
            std::cout << bit;
          }
        }
        std::cout << std::endl;
        
        baca_protocol_publisher.publish(serial_msg);
        // send through baca protokol

        /* for (auto& bits : data_frame) { */
        /*   // ROS_INFO("Led on"); */
        /*   std::cout << bits; */
        /* } */
        /* std::cout << std::endl; */
      }
      if (req.bit_value && simulation) {
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
    } 
    /* else { */
    /*   serial_msg.payload.push_back(0x92); */
    /*   if (req.multi_frequency) { */
    /*     for (int i = 0; i < 8; i++) { */
    /*       led_state.request.value = req.frequencies[i]; */
    /*       serial_msg.payload.push_back(led_state.request.value); */
    /*       clients_set_fr[i].call(led_state); */
    /*     } */
    /*     // ROS_INFO("Blinking multi"); */
    /*     // send through baca protokol */

    /*   } else { */
    /*     // set the same frequency */
    /*     led_state.request.value = req.frequency; */
    /*     for (auto& client : clients_set_fr) { */
    /*       client.call(led_state); */
    /*       serial_msg.payload.push_back(led_state.request.value); */
    /*     } */
    /*     // ROS_INFO("Blinking one"); */
    /*     // send through baca protokol */
    /*   } */
    /*   baca_protocol_publisher.publish(serial_msg); */
    /* } */

    res.success = true;
    return true;
  }

};
}  // namespace signal_setter

PLUGINLIB_EXPORT_CLASS(signal_setter::SignalSetter, nodelet::Nodelet);
