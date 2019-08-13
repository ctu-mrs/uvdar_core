#define camera_delay 0.50
#define armLength 0.2775
/* #define followDistance 6.0 */
#define farDistance 5.0
/* #define trajcoeff 0.5 */
/* #define yawcoeff 0.1 */
#define maxSpeed 2.0

#define maxTrajSteps 20

#define maxFollowDistDeviation 2.0

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <mav_manager/Vec4.h>
#include <mrs_msgs/TrackerDiagnostics.h>
#include <mrs_msgs/TrackerTrajectorySrv.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_srvs/SetBool.h>
#include <stdint.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <undistortFunctions/ocam_functions.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include "slider/slider.h"

/* #include "UVDD/uvLedDetect_cpu.h" */
/* #include "UVDD/uvLedDetect_gpu.h" */

namespace enc = sensor_msgs::image_encodings;

class Follower {
public:
  Follower(ros::NodeHandle& node) {
    reachedTarget = false;
    ros::NodeHandle private_node_handle("~");
    private_node_handle.param("uav_name", uav_name, std::string());

    private_node_handle.param("trajectoryControl", trajectoryControl, bool(false));

    private_node_handle.param("followDistance", followDistance, double(6.0));
    private_node_handle.param("trajCoeff", trajcoeff, double(0.5));
    private_node_handle.param("yawCoeff", yawcoeff, double(0.1));

    private_node_handle.param("controlYaw", controlYaw, bool(true));

    private_node_handle.param("frequencyCount", IDcount, int(4));
    private_node_handle.param("myID", myID, int(0));

    private_node_handle.param("filterDistLength", filterDistLength, int(3));

    private_node_handle.param("DEBUG", DEBUG, bool(false));

    private_node_handle.param("gui", gui, bool(false));
    private_node_handle.param("publish", publish, bool(true));

    private_node_handle.param("useOdom", useOdom, bool(false));

    ROS_INFO("FOL: UseOdom? %s", useOdom ? "true" : "false");

    gotCamInfo = false;

    private_node_handle.param("accumLength", accumLength, int(5));

    char calib_path[100];

    sprintf(calib_path, "%s/config/calib_results.txt", ros::package::getPath("uvdd").c_str());

    get_ocam_model(&oc_model, calib_path);

    targetInCamPub  = node.advertise< geometry_msgs::Pose >("targetInCam", 1);
    targetInBasePub = node.advertise< geometry_msgs::Pose >("targetInBase", 1);
    yawdiffPub      = node.advertise< std_msgs::Float32 >("yawDifference", 1);
    yawodomPub      = node.advertise< std_msgs::Float32 >("yawOdom", 1);
    setpointPub     = node.advertise< geometry_msgs::Pose >("relativeSetpoint", 1);
    setyawPub       = node.advertise< std_msgs::Float32 >("relativeSetyaw", 1);

    measuredDist = node.advertise< std_msgs::Float32 >("measuredDist", 1);
    filteredDist = node.advertise< std_msgs::Float32 >("filteredDist", 1);

    pointsSubscriber = node.subscribe("blinkersSeen", 1, &Follower::ProcessPoints, this);


    for (int i = 0; i < IDcount; i++) {
      distanceSlider.push_back(slider());
      foundTarget.push_back(false);
      first.push_back(true);
      centerEstimInBase.push_back(Eigen::Vector3d(0, 0, 0));
      separatedPoints.push_back(std::vector< cv::Point3i >());
    }


    OdomSubscriber = node.subscribe("odometry", 1, &Follower::odomAngleCallback, this);
    DiagSubscriber = node.subscribe("diagnostics", 1, &Follower::diagnosticsCallback, this);

    tf_thread = std::thread(&Follower::TfThread, this);

    yawRelativePub = node.advertise< std_msgs::Float64 >("yaw_relative_out", 1);
    report_thread  = std::thread(&Follower::ReportThread, this);
    neighborsPub   = node.advertise< sensor_msgs::PointCloud >((std::string("/") + std::string(uav_name) + std::string("/uv_estimated_neighbors")).c_str(), 1);
  }

  ~Follower() {
  }


  bool toggleReady(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {

    bool response = req.data;

    res.success = true;
    res.message = (response ? "Following enabled" : "Following disabled");

    if (response) {

      ROS_INFO("Following enabled.");

    } else {

      ROS_INFO("Following disabled");
    }

    return true;
  }

  double YawError() {
    double result;
    // CW
    int leftID   = myID - 1;
    int rightID  = myID + 1;
    int middleID = myID + 2;

    if (leftID == -1)
      leftID = IDcount - 1;
    if (rightID == IDcount)
      rightID = 0;
    if (middleID >= IDcount)
      middleID = middleID - 4;

    if ((foundTarget[leftID]) && (foundTarget[rightID])) {
      Eigen::Vector3d centerVec = (centerEstimInBase[leftID] + centerEstimInBase[rightID]) / 2.0;
      result                    = atan2(centerVec.y(), centerVec.x());
      return result;
    }

    if ((foundTarget[leftID])) {
      result = atan2(centerEstimInBase[leftID].y(), centerEstimInBase[leftID].x()) - (CV_PI / 4.0);
      return result;
    }
    if ((foundTarget[rightID])) {
      result = (-CV_PI / 4.0) - atan2(centerEstimInBase[rightID].y(), centerEstimInBase[rightID].x()) - (-CV_PI / 4.0);
      return result;
    }
    if ((foundTarget[middleID])) {
      result = atan2(centerEstimInBase[middleID].y(), centerEstimInBase[middleID].x()) - (0.0);
      return result;
    }

    return 0.0;
  }

  void ReportThread() {
    ros::Rate                   targetRate(20);
    sensor_msgs::PointCloud     ptcl;
    geometry_msgs::Point32      currPt;
    sensor_msgs::ChannelFloat32 currChannel;
    currChannel.name = "ID";
    while (true) {
      targetRate.sleep();
      ptcl.points.clear();
      ptcl.channels.clear();
      currChannel.values.clear();
      ROS_INFO("my ID: %d, count: %d", myID, IDcount);
      for (int ID = 0; ID < IDcount; ID++) {
        /* if ((!foundTarget[ID]) && ((ros::Time::now() - lastSeen).toSec() > 2.0)) { */
        if (!foundTarget[ID]) {
          /* if (true) { */
          ROS_INFO("Target with ID %d is NOT seen", ID);
        } else if (myID != ID) {
          currChannel.values.push_back((float)ID);
          currPt.x = centerEstimInBase[ID].x();
          currPt.y = centerEstimInBase[ID].y();
          currPt.z = centerEstimInBase[ID].z();
          ptcl.points.push_back(currPt);
        }
      }
      ptcl.channels.push_back(currChannel);

      neighborsPub.publish(ptcl);

      if (controlYaw) {
        std_msgs::Float64 yawCorrection;
        yawCorrection.data = YawError() * 0.1;
        ROS_INFO("yawdiff: %4.2f", yawCorrection.data);
        yawRelativePub.publish(yawCorrection);
      }
    }
  }

  void TfThread() {
    ros::Rate transformRate(1.0);
    while (true) {
      try {
        listener.waitForTransform("fcu_" + uav_name, "uvcam"+uav_name, ros::Time::now(), ros::Duration(1.0));
        mutex_tf.lock();
        listener.lookupTransform("fcu_" + uav_name, "uvcam"+uav_name, ros::Time(0), transformCam2Base);
        mutex_tf.unlock();
      }
      catch (tf::TransformException ex) {
        ROS_ERROR("TF: %s", ex.what());
        mutex_tf.unlock();
        ros::Duration(1.0).sleep();
        continue;
      }
      try {
        listener.waitForTransform("local_origin", "fcu_" + uav_name, ros::Time::now(), ros::Duration(1.0));
        mutex_tf.lock();
        listener.lookupTransform("local_origin", "fcu_" + uav_name, ros::Time(0), transformBase2World);
        mutex_tf.unlock();
      }
      catch (tf::TransformException ex) {
        ROS_ERROR("TF: %s", ex.what());
        mutex_tf.unlock();
        ros::Duration(1.0).sleep();
        continue;
      }
      transformRate.sleep();
      /* ROS_INFO("TF next"); */
    }
  }

  void odomAngleCallback(const nav_msgs::Odometry odom_msg) {
    // roll_old = roll; pitch_old = pitch; yaw_old = yaw;
    // ypr_old_time = ypr_time;

    tf::Quaternion bt;
    tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, bt);
    tf::Matrix3x3(bt).getRPY(roll, pitch, yaw);
    /* ROS_INFO("Yaw: %4.2f", yaw); */
  }

  void diagnosticsCallback(const mrs_msgs::TrackerDiagnostics diag_msg) {
    if (!(diag_msg.tracking_trajectory))
      reachedTarget = true;
    /* ROS_INFO("Reached target"); */
  }


  void ProcessPoints(const std_msgs::Int32MultiArrayConstPtr& msg) {
    int                        countSeen;
    std::vector< cv::Point3i > points;
    countSeen = (int)((msg)->layout.dim[0].size);
    if (DEBUG)
      ROS_INFO("Received points: %d", countSeen);
    if (countSeen < 1) {
      for (int i = 0; i < IDcount; i++) {
        foundTarget[i] = false;
      }
      return;
    }


    for (int i = 0; i < countSeen; i++) {
      if (msg->data[(i * 3) + 2] <= 200) {
        points.push_back(cv::Point3i(msg->data[(i * 3)], msg->data[(i * 3) + 1], msg->data[(i * 3) + 2]));
      }
    }

    for (int i = 0; i < IDcount; i++) {
      separatedPoints[i].clear();
    }

    if (points.size() > 1) {

      for (int i = 0; i < points.size(); i++) {
        if ((points[i].z >= 0) && (points[i].z != myID)) {
          separatedPoints[points[i].z].push_back(points[i]);
        }
      }

      for (int i = 0; i < IDcount; i++) {
        extractSingleRelative(separatedPoints[i], i);
      }
    }
  }

  void extractSingleRelative(std::vector< cv::Point3i > input, int ID) {
    Eigen::Vector3d centerEstimInCam;
    double          maxDist = 100.0;

    while (input.size() > 3) {
      for (int i = 0; i < input.size(); i++) {
        bool viable = false;
        for (int j = 0; j < input.size(); j++) {
          if (i == j)
            continue;

          if ((cv::norm(input[i] - input[j]) < maxDist) && (abs(input[i].y - input[j].y) < abs(input[i].x - input[j].x))) {
            viable = true;
            break;
          }
        }
        if (!viable) {
          input.erase(input.begin() + i);
          i--;
        }
      }
      maxDist = 0.5 * maxDist;
    }


    if (input.size() == 3) {
      cv::Point3i tmp;
      cv::Point3i a = input[0];
      cv::Point3i b = input[1];
      cv::Point3i c = input[2];

      if ((input[2].x) < (input[0].x)) {
        a = input[2];
        c = input[0];
      }
      if ((input[1].x) < (a.x)) {
        b = a;
        a = input[1];
      }
      if ((b.x) > (c.x)) {
        tmp = c;
        c   = b;
        b   = tmp;
      }
      std::cout << "central led: " << b << std::endl;


      double v1[3], v2[3], v3[3];
      double va[2] = {(double)(a.y), (double)(a.x)};
      double vb[2] = {(double)(b.y), (double)(b.x)};
      double vc[2] = {(double)(c.y), (double)(c.x)};

      cam2world(v1, va, &oc_model);
      cam2world(v2, vb, &oc_model);
      cam2world(v3, vc, &oc_model);

      Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
      Eigen::Vector3d V2(v2[1], v2[0], -v2[2]);
      Eigen::Vector3d V3(v3[1], v3[0], -v3[2]);

      double alpha = acos(V1.dot(V2));
      double beta  = acos(V2.dot(V3));

      double A = 1.0 / tan(alpha);
      double B = 1.0 / tan(beta);
      std::cout << "alpha: " << alpha << " beta: " << beta << std::endl;
      std::cout << "A: " << A << " B: " << B << std::endl;

      double longOperand = (A * A - A * B + sqrt(3.0) * A + B * B + sqrt(3.0) * B + 3.0);
      std::cout << "long operand: " << longOperand << std::endl;
      double delta = 2.0 * atan(((B * (2.0 * sqrt(longOperand / (B * B + 2.0 * sqrt(3.0) + 3.0)) - 1.0)) +
                                 (6.0 * sqrt(longOperand / ((sqrt(3.0) * B + 3.0) * (sqrt(3.0) * B + 3.0)))) + (2.0 * A + sqrt(3.0))) /
                                (sqrt(3.0) * B + 3.0));

      double gamma      = CV_PI - (delta + alpha);
      double distMiddle = sin(gamma) * armLength / sin(alpha);


      double distance = sqrt(fmax(0.1, distMiddle * distMiddle + armLength * armLength - 2 * distMiddle * armLength * cos(delta + (CV_PI / 3.0))));
      if (first[ID]) {
        distanceSlider[ID].filterInit(distance, filterDistLength);
        first[ID] = false;
      }
      distanceSlider[ID].filterPush(distance);
      double distanceFiltered = distanceSlider[ID].filterSlide();

      double phi = asin(sin(delta + (CV_PI / 3.0)) * (armLength / distanceFiltered));
      std::cout << "delta: " << delta << std::endl;
      std::cout << "Estimated distance: " << distance << std::endl;
      std::cout << "Filtered distance: " << distanceFiltered << std::endl;
      std_msgs::Float32 dM, fdM;
      dM.data  = distance;
      fdM.data = distanceFiltered;
      measuredDist.publish(dM);
      filteredDist.publish(fdM);

      std::cout << "Estimated angle from mid. LED: " << phi * (180.0 / CV_PI) << std::endl;


      Eigen::Vector3d Pv = V3.cross(V1).normalized();
      Eigen::Transform< double, 3, Eigen::Affine > Rp(Eigen::AngleAxis< double >(phi, Pv));
      centerEstimInCam = distanceFiltered * (V2);


      std::cout << "Estimated center in CAM: " << centerEstimInCam << std::endl;

      foundTarget[ID] = true;
      lastSeen        = ros::Time::now();

    } else if (input.size() == 2) {
      cv::Point3i a;
      cv::Point3i b;

      if ((input[0].x) < (input[1].x)) {
        a = input[0];
        b = input[1];
      } else {
        a = input[1];
        b = input[0];
      }


      std::cout << "right led: " << b << std::endl;
      cv::Point3d central = (input[0] + input[1]) / 2.0;
      double      v1[3], v2[3];
      double      va[2] = {double(a.y), double(a.x)};
      double      vb[2] = {double(b.y), double(b.x)};
      ;
      cam2world(v1, va, &oc_model);
      cam2world(v2, vb, &oc_model);
      double vc[3];
      double pc[2] = {central.y, central.x};
      cam2world(vc, pc, &oc_model);

      Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
      Eigen::Vector3d V2(v2[1], v2[0], -v2[2]);
      Eigen::Vector3d Vc(vc[1], vc[0], -vc[2]);

      double alpha = acos(V1.dot(V2));

      double vd = sqrt(0.75 * armLength);

      double distance = (armLength / 2.0) / tan(alpha / 2.0) + vd;
      if (first[ID]) {
        distanceSlider[ID].filterInit(distance, filterDistLength);
        first[ID] = false;
      }
      distanceSlider[ID].filterPush(distance);
      double distanceFiltered = distanceSlider[ID].filterSlide();

      std::cout << "Estimated distance: " << distance << std::endl;
      std::cout << "Filtered distance: " << distanceFiltered << std::endl;
      std_msgs::Float32 dM, fdM;
      dM.data  = distance;
      fdM.data = distanceFiltered;
      measuredDist.publish(dM);
      filteredDist.publish(fdM);


      centerEstimInCam = distanceFiltered * Vc;

      std::cout << "Estimated center in CAM: " << centerEstimInCam << std::endl;
      geometry_msgs::Pose p;
      p.position.x = centerEstimInCam.x();
      p.position.y = centerEstimInCam.y();
      p.position.z = centerEstimInCam.z();
      targetInCamPub.publish(p);
      foundTarget[ID] = true;
      lastSeen        = ros::Time::now();

      /* } else if (input.size() == 1) { */
      /*   std::cout << "Only single point visible - no distance information" << std::endl; */
      /*   std::cout << "led: " << input[0] << std::endl; */
      /*   double v1[3]; */
      /*   double va[2] = {double(input[0].y), double(input[0].x)}; */
      /*   ; */
      /*   cam2world(v1, va, &oc_model); */

      /*   Eigen::Vector3d V1(v1[1], v1[0], -v1[2]); */

      /*   if (first[ID]) { */
      /*     distanceSlider[ID].filterInit(farDistance, filterDistLength); */
      /*     first[ID] = false; */
      /*   } */

      /*   double distanceFiltered = distanceSlider[ID].filterSlide(); */
      /*   centerEstimInCam        = distanceFiltered * V1; */
      /*   std::cout << "Estimated center in CAM: " << centerEstimInCam << std::endl; */
      /*   geometry_msgs::Pose p; */
      /*   p.position.x = centerEstimInCam.x(); */
      /*   p.position.y = centerEstimInCam.y(); */
      /*   p.position.z = centerEstimInCam.z(); */
      /*   targetInCamPub.publish(p); */
      /*   foundTarget[ID] = true; */
      /*   lastSeen        = ros::Time::now(); */
    } else {
      std::cout << "No valid points seen. Waiting" << std::endl;
      centerEstimInCam.x()      = 0;
      centerEstimInCam.y()      = 0;
      centerEstimInCam.z()      = 0;
      centerEstimInBase[ID].x() = 0;
      centerEstimInBase[ID].y() = 0;
      centerEstimInBase[ID].z() = 0;
      foundTarget[ID]           = false;
      return;
    }


    Eigen::Affine3d Cam2Base;
    tf::transformTFToEigen(transformCam2Base, Cam2Base);
    mutex_tf.lock();

    centerEstimInBase[ID] = (Cam2Base * centerEstimInCam);
    mutex_tf.unlock();


    std::cout << "Center " << ID << " in BASE: " << centerEstimInBase[ID] << std::endl;
    geometry_msgs::Pose p;
    p.position.x = centerEstimInBase[ID].x();
    p.position.y = centerEstimInBase[ID].y();
    p.position.z = centerEstimInBase[ID].z();
    targetInBasePub.publish(p);
  }

  template < typename T >
  int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }

private:
  std::stringstream VideoPath;

  std::stringstream MaskPath;
  std::string       MaskPathHard;
  int               VideoNumber;
  bool              FromVideo;
  bool              FromBag;
  bool              FromCamera;
  int               camNum;

  std::vector< bool > first;
  bool                stopped;

  bool Flip;

  ros::Time RangeRecTime;

  ros::Subscriber pointsSubscriber;

  tf::TransformListener listener;
  tf::StampedTransform  transformCam2Base;
  tf::StampedTransform  transformBase2World;

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

  double yaw, pitch, roll;


  bool gui, publish, useOdom;

  int numberOfBins;

  bool cameraRotated;

  int accumLength;

  int   RansacNumOfChosen;
  int   RansacNumOfIter;
  float RansacThresholdRadSq;
  bool  Allsac;

  double     rollRate, pitchRate, yawRate;
  std::mutex mutex_imu;
  std::mutex mutex_tf;

  double max_px_speed_t;
  float  maxAccel;
  bool   checkAccel;

  std::string uav_name;

  ros::Time odomSpeedTime;
  float     speed_noise;

  int    lastSpeedsSize;
  double analyseDuration;


  bool trajectoryControl;

  double followDistance;
  double trajcoeff;
  double yawcoeff;

  bool controlYaw;

  /* uvLedDetect_gpu *uvdg; */

  // thread
  std::thread report_thread;
  std::thread tf_thread;

  std::vector< sensor_msgs::Imu > imu_register;

  struct ocam_model oc_model;

  ros::ServiceClient             yawClient;
  ros::Publisher                 yawRelativePub;
  mav_manager::Vec4              tpnt;
  mrs_msgs::TrackerTrajectorySrv tts;
  mrs_msgs::TrackerTrajectory    ttt;
  std::vector< bool >            foundTarget;
  std::vector< Eigen::Vector3d > centerEstimInBase;

  /* bool   toRight;    // direction in which we should go to reach the tail */


  ros::Subscriber OdomSubscriber;
  ros::Subscriber DiagSubscriber;
  bool            reachedTarget;
  ros::Time       lastSeen;


  ros::Publisher neighborsPub;

  ros::Publisher targetInCamPub;
  ros::Publisher targetInBasePub;
  ros::Publisher yawdiffPub;
  ros::Publisher setyawPub;
  ros::Publisher yawodomPub;
  ros::Publisher setpointPub;
  ros::Publisher measuredDist;
  ros::Publisher filteredDist;

  int IDcount;
  int myID;

  std::vector< slider >                     distanceSlider;
  std::vector< std::vector< cv::Point3i > > separatedPoints;
  int                                       filterDistLength;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "UVDD");
  ros::NodeHandle nodeA;
  Follower        dd(nodeA);

  ROS_INFO("Oriented follower node initiated");

  ros::spin();

  return 0;
}
