#define camera_delay 0.50

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt32MultiArray.h>
#include <stdint.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>

#include "detect/uvLedDetect_fast.h"

namespace enc = sensor_msgs::image_encodings;

namespace uvdar {
class UVDetector : public nodelet::Nodelet{
public:

  void onInit() {

    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    nh_.param("uav_name", uav_name, std::string());

    nh_.param("justReport", justReport, false);
    nh_.param("threshold", threshVal, 200);
    if (justReport)
      ROS_INFO("Thresh: %d", threshVal);


    nh_.param("FromBag", FromBag, bool(true));
    nh_.param("FromCamera", FromCamera, bool(false));
    nh_.param("Flip", Flip, bool(false));

    nh_.param("camNum", camNum, int(0));

    nh_.param("GPU", useGpu, bool(false));

    nh_.param("cellSize", cellSize, int(32));
    nh_.param("cellOverlay", cellOverlay, int(8));
    nh_.param("surroundRadius", surroundRadius, int(4));

    nh_.param("DEBUG", DEBUG, bool(false));


    nh_.param("SamplePointSize", samplePointSize, int(8));


    nh_.param("gui", gui, bool(false));
    nh_.param("publish", publish, bool(true));

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


    nh_.param("storeVideo", storeVideo, bool(false));


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

    if (true) {
      ROS_INFO("Initializing FAST-based marker detection");
      uvdf = new uvLedDetect_fast();
    }


    if (FromBag || FromCamera) {
      ROS_INFO("Ros type data");
      stopped = false;
      if (ImgCompressed) {
        ROS_INFO("Source is COMPRESSEd");
        ImageSubscriber = nh_.subscribe("camera", 1, &UVDetector::ProcessCompressed, this);
      } else {
        ROS_INFO("Source is RAW");
        ImageSubscriber = nh_.subscribe("camera", 1, &UVDetector::ProcessRaw, this);
      }
    }

    if (justReport) {
      PointsPublisher = nh_.advertise< std_msgs::UInt32MultiArray >("pointsSeen", 1);
    }


  }

  ~UVDetector() {
  }

private:

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
        for (int i = 0; i < imu_register.size(); i++) {
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

  void ProcessCompressed(const sensor_msgs::CompressedImageConstPtr& image_msg) {
    cv_bridge::CvImagePtr image;
    if (image_msg != NULL)
      image = cv_bridge::toCvCopy(image_msg);
    ProcessSingleImage(image);
  }

  void ProcessRaw(const sensor_msgs::ImageConstPtr& image_msg) {
  /* clock_t begin1, begin2, end1, end2; */
  /* double  elapsedTime; */
  /* begin1                             = std::clock(); */
    cv_bridge::CvImageConstPtr image;
    image = cv_bridge::toCvShare(image_msg, enc::MONO8);
    ProcessSingleImage(image);
  }


  void ProcessSingleImage(const cv_bridge::CvImageConstPtr image) {
  /* clock_t begin1, begin2, end1, end2; */
  /* double  elapsedTime; */
  /* begin1                             = std::clock(); */
    double yaw_local;
    double pitch_local;
    double roll_local;

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
    std::vector< cv::Point2i > outvec = uvdf->processImage(image->image, localImg_raw, false, DEBUG, threshVal);
    /* end2         = std::clock(); */
    /* elapsedTime = double(end2 - begin2) / CLOCKS_PER_SEC; */
    /* std::cout << "5: " << elapsedTime << " s" << "f: " << 1.0/elapsedTime << std::endl; */


    /* std::cout <<  outvec << std::endl; */

    if (outvec.size()>30){
      ROS_INFO("Over 30 points received. Skipping noisy image");
      return;
    }
    else if (justReport) {
      std_msgs::UInt32MultiArray msg;
      msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
      msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
      msg.layout.dim[0].size   = outvec.size();
      msg.layout.dim[0].label  = "count";
      msg.layout.dim[0].stride = outvec.size() * 3;
      msg.layout.dim[1].size   = 3;
      msg.layout.dim[1].label  = "value";
      msg.layout.dim[1].stride = 3;
      std::vector< unsigned int > convert;
      for (int i = 0; i < outvec.size(); i++) {
        convert.push_back(outvec[i].x);
        convert.push_back(outvec[i].y);
        convert.push_back(0);
      }
      msg.data = convert;
      PointsPublisher.publish(msg);
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
  cv::VideoCapture  vc;
  cv::Mat mask;
  cv::Mat localImg_raw, localImg;
  bool              FromBag;
  bool              FromCamera;
  int               camNum;

  bool first;
  bool stopped;

  bool Flip;

  ros::Time RangeRecTime;

  ros::Publisher PointsPublisher;

  ros::Subscriber CamInfoSubscriber;
  ros::Subscriber TiltSubscriber;
  ros::Subscriber ImageSubscriber;

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
  bool justReport;
  int  threshVal;
  bool silent_debug;
  bool storeVideo;
  bool AccelerationBounding;
  // std::vector<double> camRot;
  double gamma;  // rotation of camera in the helicopter frame (positive)


  int samplePointSize;

  int cellSize;
  int cellOverlay;
  int surroundRadius;

  double cx, cy, fx, fy, s;
  double k1, k2, p1, p2, k3;
  bool   gotCamInfo;

  bool gui, publish, useOdom;

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

  ros::Time odomSpeedTime;
  float     speed_noise;

  int    lastSpeedsSize;
  double analyseDuration;


  bool              useGpu;
  uvLedDetect_fast* uvdf;
  /* uvLedDetect_gpu *uvdg; */

  // thread
  std::thread main_thread;

  std::vector< sensor_msgs::Imu > imu_register;
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
