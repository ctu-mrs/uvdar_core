#include "imu_compensation.h"

using namespace uvdar;

IMU_COMPENSATION::IMU_COMPENSATION(std::string imuTopicName){
    imuTopicName_ = imuTopicName;
}

ros::Subscriber IMU_COMPENSATION::subscribeOdomTopic(ros::NodeHandle & nodeHandle){

    imu_data_callback_t callback = [this](const nav_msgs::Odometry::ConstPtr& msg){
      odomCallback(msg);
    };

    return nodeHandle.subscribe(imuTopicName_, 1, callback);
}

void IMU_COMPENSATION::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
   ROS_INFO("In IMU Compensator: [angular_velocity (x,y,z)]: [%f, %f, %f]", 
             msg->twist.twist.angular.x, 
             msg->twist.twist.angular.y, 
             msg->twist.twist.angular.z);
}




void IMU_COMPENSATION::compensateByIMUMovement(const mrs_msgs::ImagePointsWithFloatStampedConstPtr & ptsMsg, const size_t & image_index, std::vector<ros::Publisher> & pub_points_seen_compensated_imu_){

    mrs_msgs::ImagePointsWithFloatStamped compensatedMsg;
    for(auto point : ptsMsg->points){
        mrs_msgs::Point2DWithFloat p;
        p.x = -point.x;
        p.y = -point.y;
        compensatedMsg.points.push_back(p);
    }
    compensatedMsg.stamp = ptsMsg->stamp;
    
    pub_points_seen_compensated_imu_[image_index].publish(compensatedMsg);

}



IMU_COMPENSATION::~IMU_COMPENSATION()
{
}
