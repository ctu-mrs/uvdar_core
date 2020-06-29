#include <ros/ros.h>
#include <ros/package.h>
#include <mrs_msgs/BacaProtocol.h>
#include <uvdar_core/FrequencySet.h>

namespace uvdar {
  class FrequencySetter {
    private: 
      bool initialized = false;
      ros::Subscriber frequency_subscriber;
      ros::Publisher baca_protocol_publisher;
    public:
      FrequencySetter(ros::NodeHandle nh){
        frequency_subscriber = nh.subscribe("frequencies_in", 1, &FrequencySetter::frequencyCallback, this);
        baca_protocol_publisher = nh.advertise<mrs_msgs::BacaProtocol>("baca_protocol_out", 1);

        initialized = true;
      }

      ~FrequencySetter(){
      }


    private:
      void frequencyCallback(const uvdar::FrequencySetConstPtr& msg){
        mrs_msgs::BacaProtocol serial_msg;
        serial_msg.stamp = ros::Time::now();
        serial_msg.payload.push_back(0x92);

        serial_msg.payload.push_back(msg->f1);
        serial_msg.payload.push_back(msg->f2);
        serial_msg.payload.push_back(msg->f3);
        serial_msg.payload.push_back(msg->f4);
        serial_msg.payload.push_back(msg->f5);
        serial_msg.payload.push_back(msg->f6);
        serial_msg.payload.push_back(msg->f7);
        serial_msg.payload.push_back(msg->f8);

        baca_protocol_publisher.publish(serial_msg);
      }

  };
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "uvdar_frequency_setter");
  ros::NodeHandle nodeA("~");
  uvdar::FrequencySetter        kl(nodeA);

  ROS_INFO("[UVDAR FrequencySetter]: frequency setter node initiated");

  ros::spin();

  return 0;
}
