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
#include <uvdar/Int32MultiArrayStamped.h>
#include <stdint.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <functional>

#include "detect/uvLedDetect_fast.h"

namespace enc = sensor_msgs::image_encodings;

namespace uvdar {
class UVDetector : public nodelet::Nodelet{
public:

  void onInit() {

    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    nh_.param("uav_name", uav_name, std::string());

    /* nh_.param("justReport", justReport, false); */
    nh_.param("threshold", threshVal, 200);
    /* if (justReport) */
    /*   ROS_INFO("Thresh: %d", threshVal); */

    nh_.param("FromBag", FromBag, bool(true));
    nh_.param("FromCamera", FromCamera, bool(false));
    nh_.param("Flip", Flip, bool(false));

    nh_.param("GPU", useGpu, bool(false));

    nh_.param("DEBUG", DEBUG, bool(false));


    nh_.param("gui", gui, bool(false));

    nh_.param("useOdom", useOdom, bool(false));


    ROS_INFO("UseOdom? %s", useOdom ? "true" : "false");
    if (useOdom) {

      imu_register.clear();
      yawRate   = 0.0;
      pitchRate = 0.0;
      rollRate  = 0.0;
      /* listener = new tf::TransformListener(); */
      TiltSubscriber = nh_.subscribe("imu", 1, &UVDetector::TiltCallback, this, ros::TransportHints().tcpNoDelay());
    }

    bool ImgCompressed;
    nh_.param("CameraImageCompressed", ImgCompressed, bool(false));


    nh_.param("silentDebug", silent_debug, bool(false));


    nh_.param("cameraRotated", cameraRotated, bool(false));
    // nh_.getParam("camera_rotation_matrix/data", camRot);
    nh_.getParam("alpha", gamma);


    gotCamInfo = false;


    if (FromBag) {
      ros::Time::waitForValid();
      begin = ros::Time::now();
    }

    nh_.param("lines", lines, bool(false));
    nh_.param("accumLength", accumLength, int(5));

    ROS_INFO("Initializing FAST-based marker detection");
    uvdf = new uvLedDetect_fast();

    /* subscribe to cameras //{ */

    std::vector<std::string> cameraTopics;
    nh_.param("cameraTopics", cameraTopics, cameraTopics);
    if (cameraTopics.empty()) {
      ROS_ERROR("[UVDetector]: No topics of cameras were supplied");
    }

    nh_.param("useMasksNamed", _use_masks_named_, bool(false));
    if (_use_masks_named_){
      nh_.param("nato_name", _nato_name_, _nato_name_);
      nh_.param("maskFileNames", _mask_file_names_, _mask_file_names_);
      LoadMasks((int)(cameraTopics.size()));
    }

    stopped = false;
    // Create callbacks for each camera
    for (size_t i = 0; i < cameraTopics.size(); ++i) {
      image_callback_t callback = [imageIndex=i,this] (const sensor_msgs::ImageConstPtr& image_msg) { 
        ProcessRaw(image_msg, imageIndex);
      };
      imageCallbacks.push_back(callback);
    }
    // Subscribe to corresponding topics
    for (size_t i = 0; i < cameraTopics.size(); ++i) {
      imageSubscribers.push_back(nh_.subscribe(cameraTopics[i], 1, &image_callback_t::operator(), &imageCallbacks[i]));
    }


    //}
    
    
    /* create pubslishers //{ */

    std::vector<std::string> pointsSeenTopics;
    if (true) {
      nh_.param("pointsSeenTopics", pointsSeenTopics, pointsSeenTopics);
      // if the number of subscribed topics doesn't match the numbe of published
      if ((FromBag || FromCamera) && pointsSeenTopics.size() != cameraTopics.size()) {
        ROS_ERROR_STREAM("[UVDetector] The number of cameraTopics (" << cameraTopics.size() 
            << ") is not matching the number of pointsSeenTopics (" << pointsSeenTopics.size() << ")!");
      }

      // Create the publishers
      for (size_t i = 0; i < pointsSeenTopics.size(); ++i) {
        sunPointsPublishers.push_back(nh_.advertise<uvdar::Int32MultiArrayStamped>(pointsSeenTopics[i]+"/sun", 1));
        pointsPublishers.push_back(nh_.advertise<uvdar::Int32MultiArrayStamped>(pointsSeenTopics[i], 1));
      }
    }

    //}

    initialized = true;
    ROS_INFO("[UVDetector]: initialized");
  }

  ~UVDetector() {
  }

private:

    /* LoadMasks //{ */
    void LoadMasks(int size){
      for (int i=0; i<size; i++){
        ROS_INFO_STREAM("Loading mask file [" << ros::package::getPath("uvdar")+"/masks/"+_nato_name_+"_"+_mask_file_names_[i]+".bmp" << "]");
        masks.push_back(cv::imread( ros::package::getPath("uvdar")+"/masks/"+_nato_name_+"_"+_mask_file_names_[i]+".bmp", cv::IMREAD_GRAYSCALE));
        uvdf->addMask(masks[i]);
      }
    }
    //}
    //
  void TiltCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
    imu_register.insert(imu_register.begin(), *imu_msg);
    bool caughtUp   = false;
    bool reachedEnd = false;
    while (!reachedEnd) {

      ros::Time::waitForValid();

      ros::Time     tcurr    = ros::Time::now();
      ros::Time     tlast    = imu_register.back().header.stamp;
      ros::Duration difftime = (tcurr - tlast);
      /* ROS_INFO("sec: %f",tcurr.toSec()); */
      /* ROS_INFO("sec: %f",tlast.toSec()); */
      /* ROS_INFO("sec: %f",difftime.toSec()); */
      if (difftime > ros::Duration(100.0 * camera_delay)) {
        imu_register.pop_back();
        return;
      }
      if (difftime > ros::Duration(camera_delay)) {
        caughtUp = true;
        if (imu_register.size() > 0)
          imu_register.pop_back();
        /* ROS_INFO("size:%d", (int)(imu_register.size())); */
        for (int i = 0; i < (int)(imu_register.size()); i++) {
          /* ROS_INFO("%d:%f", i,imu_register[i].angular_velocity.z); */
        }
      } else {
        /* ROS_INFO("size:%d", imu_register.size()); */
        reachedEnd = true;
      }
    }

    ros::Duration dur = (ros::Time::now() - imu_register.back().header.stamp);

    if (!caughtUp)
      return;

    mutex_imu.lock();
    {
      yawRate   = imu_register.back().angular_velocity.z;
      pitchRate = imu_register.back().angular_velocity.y;
      rollRate  = imu_register.back().angular_velocity.x;
    }
    mutex_imu.unlock();

    /* ROS_INFO( "Y:%f, P:%f, R:%f, B:%d", yawRate, pitchRate, rollRate, (int)(imu_register.size())); */
  }

  void ProcessCompressed(const sensor_msgs::CompressedImageConstPtr& image_msg, size_t imageIndex) {

    cv_bridge::CvImagePtr image;
    if (image_msg != NULL)
      image = cv_bridge::toCvCopy(image_msg);
    ProcessSingleImage(image, imageIndex);
  }

  void ProcessRaw(const sensor_msgs::ImageConstPtr& image_msg, size_t imageIndex) {
  /* clock_t begin1, begin2, end1, end2; */
  /* double  elapsedTime; */
  /* begin1                             = std::clock(); */
    cv_bridge::CvImageConstPtr image;
    image = cv_bridge::toCvShare(image_msg, enc::MONO8);
    ProcessSingleImage(image, imageIndex);
  }


  void ProcessSingleImage(const cv_bridge::CvImageConstPtr image, size_t imageIndex) {
    if (!initialized) return;
  /* clock_t begin1, begin2, end1, end2; */
  /* double  elapsedTime; */
  /* begin1                             = std::clock(); */

    if (stopped)
      return;
    int key = -1;

    // First things first
    if (first) {
      if (DEBUG) {
        ROS_INFO("Source img: %dx%d", image->image.cols, image->image.rows);
      }
      /* cv::namedWindow("cv_Main", CV_GUI_NORMAL|CV_WINDOW_AUTOSIZE); */
      first = false;
    }

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
    uvdf_mutex.lock();
    std::vector<cv::Point2i> outvec = uvdf->processImage(&(image->image), &(image->image), sun_points, gui, DEBUG, threshVal, _use_masks_named_?imageIndex:-1);
    uvdf_mutex.unlock();
    /* end2         = std::clock(); */
    /* elapsedTime = double(end2 - begin2) / CLOCKS_PER_SEC; */
    /* std::cout << "5: " << elapsedTime << " s" << "f: " << 1.0/elapsedTime << std::endl; */


    /* std::cout <<  outvec << std::endl; */

    if (outvec.size()>30){
      ROS_INFO("Over 30 points received. Skipping noisy image");
      return;
    }
    else {
      std::vector< int > convert;
      /* if ((int)(sun_points.size()) > 0) { */
        uvdar::Int32MultiArrayStamped msg_sun;
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
        sunPointsPublishers[imageIndex].publish(msg_sun);
      /* } */

      uvdar::Int32MultiArrayStamped msg;
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
      pointsPublishers[imageIndex].publish(msg);
    }
    if (gui)
      key = cv::waitKey(10);

    if (key == 13)
      stopped = true;

  /* end1         = std::clock(); */
  /* elapsedTime = double(end1 - begin1) / CLOCKS_PER_SEC; */
  /* std::cout << "6: " << elapsedTime << " s" << "f: " << 1.0/elapsedTime << std::endl; */

  }

private:
  bool initialized = false;

  cv::VideoCapture  vc;
  cv::Mat mask;
  bool              FromBag;
  bool              FromCamera;

  bool first;
  bool stopped;

  bool Flip;

  ros::Time RangeRecTime;

  ros::Subscriber CamInfoSubscriber;
  ros::Subscriber TiltSubscriber;
  std::vector<ros::Subscriber> imageSubscribers;
  using image_callback_t = std::function<void (const sensor_msgs::ImageConstPtr&)>;
  std::vector<image_callback_t> imageCallbacks;

  std::vector<cv::Mat> masks;

  std::vector<ros::Publisher> sunPointsPublishers;
  std::vector<ros::Publisher> pointsPublishers;

  tf::TransformListener* listener;

  cv::Mat imOrigScaled;
  cv::Mat imCurr;
  cv::Mat imPrev;

  double vxm, vym, vam;

  int         imCenterX, imCenterY;  // center of original image
  int         xi, xf, yi, yf;        // frame corner coordinates
  cv::Point2i midPoint;
  bool        coordsAcquired;
  cv::Rect    frameRect;


  ros::Time begin;

  // Input arguments
  bool DEBUG;
  int  threshVal;
  bool silent_debug;
  bool AccelerationBounding;
  // std::vector<double> camRot;
  double gamma;  // rotation of camera in the helicopter frame (positive)



  double cx, cy, fx, fy, s;
  double k1, k2, p1, p2, k3;
  bool   gotCamInfo;

  bool gui, useOdom, _use_masks_named_;

  int numberOfBins;

  bool cameraRotated;

  bool lines;
  int  accumLength;

  int   RansacNumOfChosen;
  int   RansacNumOfIter;
  float RansacThresholdRadSq;
  bool  Allsac;

  double     rollRate, pitchRate, yawRate;
  std::mutex mutex_imu;

  double max_px_speed_t;
  float  maxSpeed;
  float  maxAccel;
  bool   checkAccel;

  std::string uav_name;
  std::string _nato_name_;

  ros::Time odomSpeedTime;
  float     speed_noise;

  int    lastSpeedsSize;
  double analyseDuration;


  bool              useGpu;
  uvLedDetect_fast* uvdf;
  std::mutex  uvdf_mutex;
  /* uvLedDetect_gpu *uvdg; */

  // thread
  std::thread main_thread;

  std::vector< sensor_msgs::Imu > imu_register;

  std::vector<std::string> _mask_file_names_;
};

/* int main(int argc, char** argv) { */
/*   ros::init(argc, argv, "uv_marker_detector"); */
/*   ros::NodeHandle nodeA; */
/*   UVDetector   uvd(nodeA); */

/*   ROS_INFO("UV LED marker detector node initiated"); */

/*   ros::spin(); */

/*   return 0; */
/* } */

} //namespace uvdar
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uvdar::UVDetector, nodelet::Nodelet)
