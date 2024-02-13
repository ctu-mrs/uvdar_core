#define camera_delay 0.50
#define MAX_POINTS_PER_IMAGE 100

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <image_transport/image_transport.h>
#include <iostream>

namespace uvdar {
class UVDARVideoReader: public nodelet::Nodelet{
public:
    UVDARVideoReader(){};
    
    ~UVDARVideoReader(){
}
/* onInit() //{ */

  /**
   * @brief Initializer - loads parameters and initializes necessary structures
   */
private:
    void onInit() {


    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();
    cv::VideoCapture cap("/home/tim/records_two_signals/water/first_test.mp4");

    image_callback_t callback = [this] (sensor_msgs::Image& image_msg) { 
        callbackImage(image_msg );
      };
    nh_.advertise<sensor_msgs::Image>("camera/image", 1);
     

    
 }
  //}

  

  ros::Publisher pub_image;
  using image_callback_t= boost::function<void (const sensor_msgs::Image&)>;

  void UVDARVideoReader::callbackImage(sensor_msgs::Image& image_msg){
  
  }
  
  };


} //namespace uvdar

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uvdar::UVDARVideoReader, nodelet::Nodelet)
