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

#include <iostream>

namespace uvdar {
class UVDARVideoReader: public nodelet::Nodelet{
public:

/* onInit() //{ */

  /**
   * @brief Initializer - loads parameters and initializes necessary structures
   */
  void onInit() {

    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();
    cv::VideoCapture cap("/home/tim/records_two_signals/water/first_test.mp4");
    
     
  while(1){
 
    cv:: Mat frame;
    
    // Capture frame-by-frame
    cap >> frame;

    // If the frame is empty, break immediately
    if (frame.empty()){
        ROS_WARN("Frame Empty");
    }
    // Write the frame into the file 'outcpp.avi'
    
    // Display the resulting frame    
    cv::imshow( "Frame", frame );
  
    // Press  ESC on keyboard to  exit
    char c = (char)cv::waitKey(1);
    if( c == 27 ) 
      break;
  }
 
  // When everything done, release the video capture and write object
  cap.release();
 
  // Closes all the fra
  cv::destroyAllWindows();  
       

    
 }
  //}
~UVDARVideoReader(){
}
  
private:

};


} //namespace uvdar

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uvdar::UVDARVideoReader, nodelet::Nodelet)
