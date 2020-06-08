#define camera_delay 0.50

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/MultiArrayDimension.h>
#include <mrs_msgs/Int32MultiArrayStamped.h>
#include <stdint.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <functional>
#include <boost/filesystem/operations.hpp>

#include "detect/uv_led_detect_fast.h"

namespace enc = sensor_msgs::image_encodings;

namespace uvdar {
class UVDARDetector : public nodelet::Nodelet{
public:

/* onInit() //{ */
  void onInit() {

    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    nh_.param("debug", _debug_, bool(false));
    nh_.param("gui", _gui_, bool(false));

    nh_.param("threshold", _threshold_, 200);

    /* subscribe to cameras //{ */

    std::vector<std::string> _camera_topics;
    nh_.param("camera_topics", _camera_topics);
    if (_camera_topics.empty()) {
      ROS_ERROR("[UVDARDetector]: No camera topics were supplied!");
      return;
    }
    _camera_count_ = (unsigned int)(_camera_topics.size());

    // Create callbacks for each camera
    for (unsigned int i = 0; i < _camera_count_; ++i) {
      image_callback_t callback = [image_index=i,this] (const sensor_msgs::ImageConstPtr& image_msg) { 
        processRaw(image_msg, image_index);
      };
      cals_image_.push_back(callback);
    }
    // Subscribe to corresponding topics
    for (size_t i = 0; i < _camera_topics.size(); ++i) {
      sub_images_.push_back(nh_.subscribe(_camera_topics[i], 1, &image_callback_t::operator(), &cals_image_[i]));
    }

    //}

    /* prepare masks if necessary //{ */
    nh_.param("use_masks", _use_masks_, bool(false));
    if (_use_masks_){
      nh_.param("mask_file_names", _mask_file_names_);

      if (_mask_file_names_.size() != _camera_count_){
        ROS_ERROR_STREAM("[UVDARDetector]: Masks are enabled, but the number of mask filenames provided does not match the number of camera topics (" << _camera_count_ << ")!");
        return;
      }

      nh_.param("masks_mrs_named", _masks_mrs_named_, bool(false));
      if (_masks_mrs_named_){
        nh_.param("body_name", _body_name_);
      }

      if (!loadMasks()){
        ROS_ERROR("[UVDARDetector]: Masks are enabled, but the mask files could not be loaded!");
        return;
      }
    }
    //}
    
    /* create pubslishers //{ */
    nh_.param("publish_sun_points", _publish_sun_points_, bool(false));

    std::vector<std::string> _points_seen_topics;
      nh_.param("points_seen_topics", _points_seen_topics);
      if (_points_seen_topics.size() != _camera_count_) {
        ROS_ERROR_STREAM("[UVDARDetector] The number of output topics (" << _points_seen_topics.size()  << ") does not match the number of cameras (" << _camera_count_ << ")!");
        return;
      }

      // Create the publishers
      for (size_t i = 0; i < _points_seen_topics.size(); ++i) {
        pub_candidate_points_.push_back(nh_.advertise<mrs_msgs::Int32MultiArrayStamped>(_points_seen_topics[i], 1));
        
        if (_publish_sun_points_){
          pub_sun_points_.push_back(nh_.advertise<mrs_msgs::Int32MultiArrayStamped>(_points_seen_topics[i]+"/sun", 1));
        }
      }
    //}

    ROS_INFO("[UVDARDetector]: Initializing FAST-based marker detection...");
    uvdf_ = std::make_unique<uvLedDetect_fast>();
    if (!uvdf_){
      ROS_ERROR("[UVDARDetector]: Failed to initialize FAST-based marker detection!");
      return;
    }

    ROS_INFO("[UVDARDetector]: Waiting for time...");
    ros::Time::waitForValid();

    initialized_ = true;
    ROS_INFO("[UVDARDetector]: Initialized.");
  }
  //}

  ~UVDARDetector() {
  }

private:

    /* loadMasks //{ */
    bool loadMasks(){
      std::string file_name;
      for (unsigned int i=0; i<_camera_count_; i++){

        if (_masks_mrs_named_){
          file_name = ros::package::getPath("uvdar")+"/masks/"+_body_name_+"_"+_mask_file_names_[i]+".bmp";
        } else {
          file_name = _mask_file_names_[i];
        }

        ROS_INFO_STREAM("[UVDARDetector]: Loading mask file [" << file_name << "]");
        if (!(boost::filesystem::exists(file_name))){
          ROS_ERROR_STREAM("[UVDARDetector]: Mask [" << file_name << "] does not exist!");
          return false;
        }

        _masks_.push_back(cv::imread(file_name, cv::IMREAD_GRAYSCALE));
        if (!(_masks_.back().data)){
          ROS_ERROR_STREAM("[UVDARDetector]: Mask [" << file_name << "] could not be loaded!");
          return false;
        }

        uvdf_->addMask(_masks_[i]);
      }
      return true;
    }
    //}

  void ProcessCompressed(const sensor_msgs::CompressedImageConstPtr& image_msg, int image_index) {

    cv_bridge::CvImagePtr image;
    if (image_msg != NULL)
      image = cv_bridge::toCvCopy(image_msg);
    ProcessSingleImage(image, image_index);
  }

  void processRaw(const sensor_msgs::ImageConstPtr& image_msg, int image_index) {
  /* clock_t begin1, begin2, end1, end2; */
  /* double  elapsedTime; */
  /* begin1                             = std::clock(); */
    cv_bridge::CvImageConstPtr image;
    image = cv_bridge::toCvShare(image_msg, enc::MONO8);
    ProcessSingleImage(image, image_index);
  }


  void ProcessSingleImage(const cv_bridge::CvImageConstPtr image, int image_index) {
    if (!initialized_) return;
  /* clock_t begin1, begin2, end1, end2; */
  /* double  elapsedTime; */
  /* begin1                             = std::clock(); */

    int key = -1;

    /* imshow("wtf", localImg_raw); */
  /* end         = std::clock(); */
  /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
  /* std::cout << "3: " << elapsedTime << " s" << "f: " << 1.0/elapsedTime << std::endl; */
  /* begin                             = std::clock(); */


    /* localImg_raw.copyTo(localImg, mask); */



    /* end         = std::clock(); */
    /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
    /* std::cout << "4: " << elapsedTime << " s" << "f: " << 1.0/elapsedTime << std::endl; */
    /* begin2                             = std::clock(); */
    std::vector<cv::Point2i> sun_points;
    std::vector<cv::Point2i> outvec = uvdf_->processImage(&(image->image), &(image->image), sun_points, _gui_, _debug_, _threshold_, _masks_mrs_named_?image_index:-1);
    /* end2         = std::clock(); */
    /* elapsedTime = double(end2 - begin2) / CLOCKS_PER_SEC; */
    /* std::cout << "5: " << elapsedTime << " s" << "f: " << 1.0/elapsedTime << std::endl; */

    std::vector< int > convert;

    mrs_msgs::Int32MultiArrayStamped msg_sun;
    msg_sun.stamp = image->header.stamp;
    msg_sun.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_sun.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_sun.layout.dim[0].size   = sun_points.size();
    msg_sun.layout.dim[0].label  = "count";
    msg_sun.layout.dim[0].stride = sun_points.size() * 3;
    msg_sun.layout.dim[1].size   = 3;
    msg_sun.layout.dim[1].label  = "value";
    msg_sun.layout.dim[1].stride = 3;
    for (int i = 0; i < (int)(sun_points.size()); i++) {
      convert.push_back(sun_points[i].x);
      convert.push_back(sun_points[i].y);
      convert.push_back(0);
    }
    msg_sun.data = convert;
    pub_sun_points_[image_index].publish(msg_sun);


    if (outvec.size()>30){
      ROS_INFO("Over 30 points received. Skipping noisy image");
      return;
    }
    else {
      mrs_msgs::Int32MultiArrayStamped msg;
      msg.stamp = image->header.stamp;
      msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
      msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
      msg.layout.dim[0].size   = outvec.size();
      msg.layout.dim[0].label  = "count";
      msg.layout.dim[0].stride = outvec.size() * 3;
      msg.layout.dim[1].size   = 3;
      msg.layout.dim[1].label  = "value";
      msg.layout.dim[1].stride = 3;
      convert.clear();
      for (int i = 0; i < (int)(outvec.size()); i++) {
        convert.push_back(outvec[i].x);
        convert.push_back(outvec[i].y);
        convert.push_back(0);
      }
      msg.data = convert;
      pub_candidate_points_[image_index].publish(msg);
    }

    if (_gui_){
      cv::waitKey(10);
    }

  /* end1         = std::clock(); */
  /* elapsedTime = double(end1 - begin1) / CLOCKS_PER_SEC; */
  /* std::cout << "6: " << elapsedTime << " s" << "f: " << 1.0/elapsedTime << std::endl; */

  }

private:
  bool initialized_ = false;

  std::vector<ros::Subscriber> sub_images_;
  unsigned int _camera_count_;
  using image_callback_t = std::function<void (const sensor_msgs::ImageConstPtr&)>;
  std::vector<image_callback_t> cals_image_;


  bool _publish_sun_points_ = false;
  std::vector<ros::Publisher> pub_sun_points_;

  std::vector<ros::Publisher> pub_candidate_points_;

  bool _debug_;
  bool _gui_;

  int  _threshold_;

  bool _use_masks_;
  bool _masks_mrs_named_;
  std::string _body_name_;
  std::vector<std::string> _mask_file_names_;
  std::vector<cv::Mat> _masks_;

  std::unique_ptr<uvLedDetect_fast> uvdf_;

};


} //namespace uvdar

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uvdar::UVDARDetector, nodelet::Nodelet)
