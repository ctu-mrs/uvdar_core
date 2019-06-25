#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <mutex>

#define filterCount 2
#define freq 3


class Reprojector{
  public:
  Reprojector(ros::NodeHandle nh){
    gotImage = false;
    gotOdom = false;
    measSubscriber[0] = nh.subscribe("filteredPose1", 1, &Reprojector::odomCallback, this);
    measSubscriber[1] = nh.subscribe("filteredPose2", 1, &Reprojector::odomCallback, this);
    ImageSubscriber = nh.subscribe("camera", 1, &Reprojector::imageCallback, this);

    timer = nh.createTimer(ros::Duration(1.0/fmax(freq,1.0)), &Reprojector::spin, this);
  }

  ~Reprojector(){
  }
  private:
  //methods
  void imageCallback(const sensor_msgs::ImageConstPtr& msg_Image){
    gotImage = true;
    std::scoped_lock lock(mtx_image);
    if (msg_Image == NULL) 
      return;
    cv_bridge::CvImageConstPtr image;
    if (sensor_msgs::image_encodings::isColor(msg_Image->encoding))
      image = cv_bridge::toCvShare(msg_Image, sensor_msgs::image_encodings::BGR8);
    else
      image = cv_bridge::toCvShare(msg_Image, sensor_msgs::image_encodings::MONO8);
    currImage = image->image;
  }

  void odomCallback(nav_msgs::OdometryPtr msg_odom){
    gotOdom = true;
    std::scoped_lock(mtx_odom);
    currOdom = *msg_odom;
  }

  void spin([[ maybe_unused ]] const ros::TimerEvent& unused){
    if (!gotOdom){
      ROS_INFO("Odometry not yet obtained, waiting...");
      return;
    }
    if (!gotImage){
      ROS_INFO("Image not yet obtained, waiting...");
      return;
    }
    drawAndShow();
  }

  void drawAndShow(){
    drawImage();
    cv::imshow("ocv_marked",viewImage);
  }

  void drawImage(){
    std::scoped_lock lock(mtx_image,mtx_odom);
    if (currImage.channels() == 1){
      cv::cvtColor(currImage,viewImage,cv::COLOR_GRAY2BGR);
    }
    else {
      viewImage = currImage;
    }
    cv::circle(viewImage,getImPos(currOdom),getProjSize(currOdom),cv::Scalar(0,0,255));
  }

  cv::Point2i getImPos(nav_msgs::Odometry currOdom){
    return cv::Point2i(200,200);
  }

  int getProjSize(nav_msgs::Odometry currOdom){
    return 10;
  }

  
  //variables
  nav_msgs::Odometry currOdom;
  cv::Mat currImage, viewImage;

  std::mutex mtx_image, mtx_odom;
  
  ros::Subscriber measSubscriber[filterCount];
  ros::Subscriber ImageSubscriber;

  ros::Timer timer;

  bool gotImage,gotOdom;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "uvdar_reprojector");
  ros::NodeHandle nh;
  Reprojector r(nh);


  ros::spin();


  return 0;
}
