#include <ros/ros.h>
#include <ros/package.h>
#include <uvdar/FrequencySet.h>

namespace uvdar {
  class FrequencySetter {
    private: 
      bool initialized = false;
      ros::Subscriber frequency_subscriber;
    public:
      FrequencySetter(ros::NodeHandle nh){
      frequency_subscriber = nh.subscribe("freqnuencies_in" + std::to_string(i+1), 1, &UvdarKalman::measurementCallback, this);


      initialized = true;
      }

      ~FrequencySetter(){
      }


  };
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "uvdar_frequency_setter");
  ros::NodeHandle nodeA;
  uvdar::FrequencySetter        kl(nodeA);

  ROS_INFO("[UVDAR FrequencySetter]: frequency setter node initiated");

  ros::spin();

  return 0;
}
