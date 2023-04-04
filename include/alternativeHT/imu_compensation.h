#include <ros/ros.h>
#include <iostream>
#include <mrs_msgs/ImagePointsWithFloatStamped.h>
#include "nav_msgs/Odometry.h"

// priliminary 


namespace uvdar{
    

class IMU_COMPENSATION{
    
    
    private:
        using imu_data_callback_t = boost::function<void (const nav_msgs::Odometry::ConstPtr&)>;
        std::string imuTopicName_;
        std::string pubTopic_extension_; 
        void odomCallback(const nav_msgs::Odometry::ConstPtr&);

    public:
        IMU_COMPENSATION(const std::string);
        ~IMU_COMPENSATION();
        ros::Subscriber subscribeOdomTopic(ros::NodeHandle &);
        void compensateByIMUMovement(const mrs_msgs::ImagePointsWithFloatStampedConstPtr &, const size_t &,  std::vector<ros::Publisher> & );
};


} //uvdar