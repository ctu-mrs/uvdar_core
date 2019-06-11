#define leftID 0
#define rightID 1

#define camera_delay 0.50
#define armLength 0.2775
/* #define followDistance 6.0 */
#define farDistance 5.0
/* #define tailingCoeff 1.0 */
/* #define trajcoeff 0.5 */
/* #define yawcoeff 0.1 */
#define maxSpeed 2.0

#define maxTrajSteps 20

#define maxFollowDistDeviation 2.0

/* #include <std_srvs/Trigger.h> */
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/TrackerDiagnostics.h>
/* #include <mrs_msgs/TrackerTrajectorySrv.h> */
#include <mrs_msgs/Vec1.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_srvs/SetBool.h>
#include <stdint.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <OCamCalib/ocam_functions.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include "unscented/unscented.h"



namespace enc = sensor_msgs::image_encodings;

class PoseReporter {
public:
  PoseReporter(ros::NodeHandle& node) {
    reachedTarget   = false;
    followTriggered = false;
    ros::NodeHandle private_node_handle("~");
    private_node_handle.param("uav_name", uav_name, std::string());

    private_node_handle.param("followDistance", followDistance, double(6.0));
    private_node_handle.param("trajCoeff", trajcoeff, double(0.5));
    private_node_handle.param("yawCoeff", yawcoeff, double(0.1));
    private_node_handle.param("tailingCoeff", tailingCoeff, double(1.0));

    private_node_handle.param("filterDistLength", filterDistLength, int(3));
    private_node_handle.param("filterOrientationLength", filterOrientationLength, int(3));

    private_node_handle.param("DEBUG", DEBUG, bool(false));

    private_node_handle.param("gui", gui, bool(false));
    private_node_handle.param("publish", publish, bool(true));

    private_node_handle.param("useOdom", useOdom, bool(false));

    ROS_INFO("FOL: UseOdom? %s", useOdom ? "true" : "false");

    gotCamInfo = false;

    private_node_handle.param("accumLength", accumLength, int(5));

    char calib_path[100];

    sprintf(calib_path, "%s/include/OCamCalib/config/calib_results.txt", ros::package::getPath("uvdar").c_str());

    get_ocam_model(&oc_model, calib_path);

    targetInCamPub    = node.advertise< geometry_msgs::Pose >("targetInCam", 1);
    targetInBasePub   = node.advertise< geometry_msgs::Pose >("targetInBase", 1);
    goalposInWorldPub = node.advertise< geometry_msgs::Pose >("goalposInWorld", 1);
    goalposInBasePub  = node.advertise< geometry_msgs::Pose >("goalposInBase", 1);
    yawdiffPub        = node.advertise< std_msgs::Float32 >("yawDifference", 1);
    yawodomPub        = node.advertise< std_msgs::Float32 >("yawOdom", 1);
    setpointPub       = node.advertise< geometry_msgs::Pose >("relativeSetpoint", 1);
    setyawPub         = node.advertise< std_msgs::Float32 >("relativeSetyaw", 1);

    measuredDist = node.advertise< std_msgs::Float32 >("measuredDist", 1);
    filteredDist = node.advertise< std_msgs::Float32 >("filteredDist", 1);

    Px2 = Eigen::MatrixXd(9,9);
    Px3 = Eigen::MatrixXd(11,11);


    first            = true;
    pointsSubscriber = node.subscribe("blinkersSeen", 1, &PoseReporter::ProcessPoints, this);

    foundTarget = false;


    OdomSubscriber = node.subscribe("odometry", 1, &PoseReporter::odomAngleCallback, this);
    DiagSubscriber = node.subscribe("diagnostics", 1, &PoseReporter::diagnosticsCallback, this);

    tf_thread   = std::thread(&PoseReporter::TfThread, this);
    ser_trigger = private_node_handle.advertiseService("toggle_uv_follow", &PoseReporter::toggleReady, this);

  }

  ~PoseReporter() {
  }

  Eigen::VectorXd uvdarHexarotorPose3p(Eigen::VectorXd X, int ids[3]){
      cv::Point3i tmp;
      cv::Point3i a(X(0),X(1),ids[0]);
      cv::Point3i b(X(2),X(3),ids[1]);
      cv::Point3i c(X(4),X(5),ids[2]);

      if ((points[2].x) < (points[0].x)) {
        a = points[2];
        c = points[0];
      }
      if ((points[1].x) < (a.x)) {
        b = a;
        a = points[1];
      }
      if ((b.x) > (c.x)) {
        tmp = c;
        c   = b;
        b   = tmp;
      }
      std::cout << "central led: " << b << std::endl;


      if ((a.z == leftID) && (b.z == leftID) && (c.z == leftID)) {
        angleDist = CV_PI * (1.5 / 3.0);
      } else if ((a.z == leftID) && (b.z == leftID) && (c.z == rightID)) {
        angleDist = CV_PI * (0.5 / 3.0);
      } else if ((a.z == leftID) && (b.z == rightID) && (c.z == rightID)) {
        angleDist = -CV_PI * (0.5 / 3.0);
      } else if ((a.z == rightID) && (b.z == rightID) && (c.z == rightID)) {
        angleDist = -CV_PI * (1.5 / 3.0);
      } else if ((a.z == rightID) && (b.z == leftID) && (c.z == leftID)) {
        angleDist = CV_PI * (2.5 / 3.0);
      } else if ((a.z == rightID) && (b.z == rightID) && (c.z == leftID)) {
        angleDist = -CV_PI * (2.5 / 3.0);
      } else {
        angleDist = 0.0;
      }

      double pixDist = (cv::norm(b - a) + cv::norm(c - b)) * 0.5;
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
      if (first) {
        first = false;
      }

      double phi = asin(sin(delta + (CV_PI / 3.0)) * (armLength / distanceFiltered));
      std::cout << "delta: " << delta << std::endl;
      std::cout << "Estimated distance: " << distance << std::endl;
      std_msgs::Float32 dM, fdM;
      dM.data  = distance;
      fdM.data = distanceFiltered;
      measuredDist.publish(dM);
      filteredDist.publish(fdM);

      std::cout << "Estimated angle from mid. LED: " << phi * (180.0 / CV_PI) << std::endl;


      Eigen::Vector3d Pv = V3.cross(V1).normalized();
      Eigen::Transform< double, 3, Eigen::Affine > Rp(Eigen::AngleAxis< double >(phi, Pv));
      centerEstimInCam = distanceFiltered * (V2);
      /* goalInCam        = (distanceFiltered - followDistance) * (Rp * V2); */
      /* tf::Vector3 centerEstimInCamTF; */
      /* tf::vectorEigenToTF(centerEstimInCam, centerEstimInCamTF); */
      /* tf::Vector3 centerEstimInBaseTF = (transform * centerEstimInCamTF); */
      /* /1* std::cout << centerEstimInBaseTF << std::endl; *1/ */
      /* tf::vectorTFToEigen(centerEstimInBaseTF, centerEstimInBase); */


      std::cout << "Estimated center in CAM: " << centerEstimInCam << std::endl;
  }

  Eigen::VectorXd uvdarHexarotorPose2p(Eigen::VectorXd X, double fleft, double fright, double fcenter){

      cv::Point3i a;
      cv::Point3i b;

      if ((X(0)) < (X(2))) {
        a = cv::Point3i(X(0),X(1),-1);
        b = cv::Point3i(X(2),X(3),-1);
      } else {
        a = cv::Point3i(X(2),X(3),-1);
        b = cv::Point3i(X(0),X(1),-1);
      }

      if ((a.z == leftID) && (b.z == leftID)) {
        angleDist = CV_PI * (1.5 / 3.0);
      } else if ((a.z == leftID) && (b.z == rightID)) {
        angleDist = 0;
      } else if ((a.z == rightID) && (b.z == rightID)) {
        angleDist = -CV_PI * (1.5 / 3.0);
      } else if ((a.z == rightID) && (b.z == leftID)) {
        angleDist = CV_PI;
      } else {  // should never happen
        angleDist = 0.0;
      }


      std::cout << "right led: " << b << std::endl;
      cv::Point3d central = (points[0] + points[1]) / 2.0;
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
      if (first) {
        distanceSlider.filterInit(distance, filterDistLength);
        orientationSlider.filterInit(angleDist, filterOrientationLength);
        first = false;
      }
      distanceSlider.filterPush(distance);
      orientationSlider.filterPush(angleDist);
      double distanceFiltered    = distanceSlider.filterSlide();
      double orientationFiltered = orientationSlider.filterSlide();

      std::cout << "Estimated distance: " << distance << std::endl;
      std::cout << "Filtered distance: " << distanceFiltered << std::endl;
  }


  bool toggleReady(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {

    bool response = req.data;

    res.success = true;
    res.message = (response ? "Following enabled" : "Following disabled");

    if (response) {

      followTriggered = true;
      ROS_INFO("Following enabled.");

    } else {

      followTriggered = false;
      ROS_INFO("Following disabled");
    }

    return true;
  }




  void TfThread() {
    ros::Rate transformRate(1.0);
    while (true) {
      try {
        listener.waitForTransform("fcu_" + uav_name, "uvcam", ros::Time::now(), ros::Duration(1.0));
        mutex_tf.lock();
        listener.lookupTransform("fcu_" + uav_name, "uvcam", ros::Time(0), transformCam2Base);
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
      foundTarget = false;
      return;
    }

    Eigen::Vector3d centerEstimInCam;
    Eigen::Vector3d goalInCam;


    for (int i = 0; i < countSeen; i++) {
      if (msg->data[(i * 3) + 2] <= 200) {
        points.push_back(cv::Point3i(msg->data[(i * 3)], msg->data[(i * 3) + 1], msg->data[(i * 3) + 2]));
      }
    }

    if (points.size() > 1) {
      double maxDist = 100.0;

      for (int i = 0; i < points.size(); i++) {
        if (points[i].z < 0) {
          points.erase(points.begin() + i);
          i--;
          continue;
        }
      }
      while (points.size() > 3) {
        for (int i = 0; i < points.size(); i++) {
          bool viable = false;
          for (int j = 0; j < points.size(); j++) {
            if (i == j)
              continue;

            if ((cv::norm(points[i] - points[j]) < maxDist) && (abs(points[i].y - points[j].y) < abs(points[i].x - points[j].x))) {
              viable = true;
              break;
            }
          }
          if (!viable) {
            points.erase(points.begin() + i);
            i--;
          }
        }
        maxDist = 0.5 * maxDist;
        /* std::cout << "maxDist: " << maxDist << std::endl; */
      }
    }


      Eigen::VectorXd X,Y;
      unscented::measurement ms;

    if (points.size() == 3) {
      X <<
        points[0].x ,points[0].y, points[0].z,
        points[1].x ,points[1].y, points[1].z,
        points[2].x, points[2].y, points[2].z;
      perr=0.2/framerate;
      Pm3 <<
        0.25,0,0,0,0,0,0,0,0,0;...
        0,0.25,0,0,0,0,0,0,0,0;...
        0,0,perr^2,0,0,0,0,0,0,0;...
        0,0,0,0.25,0,0,0,0,0,0;...
        0,0,0,0,0.25,0,0,0,0,0;...
        0,0,0,0,0,perr^2,0,0,0,0;...
        0,0,0,0,0,0,0.25,0,0,0;...
        0,0,0,0,0,0,0,0.25,0,0;...
        0,0,0,0,0,0,0,0,perr^2,0;...
        0,0,0,0,0,0,0,0,0,(2*pi/3)^2;...
        ;
      ms = unscented::unscentedTransform(X,Px3,&uvdarHexarotorPose3p);
    }

    else if (points.size() == 2) {
      X << points[0].x ,points[0].y, points[1].x ,points[1].y;
    perr=0.2/framerate;
    Pm2 <<
      0.25,0,0,0,0,0,0,0,0;...
      0,0.25,0,0,0,0,0,0,0;...
      0,0,perr^2,0,0,0,0,0,0;...
      0,0,0,0.25,0,0,0,0,0;...
      0,0,0,0,0.25,0,0,0,0;...
      0,0,0,0,0,perr^2,0,0,0;...
      0,0,0,0,0,0,deg2rad(8)^2,0,0;...
      0,0,0,0,0,0,0,deg2rad(30)^2,0;...
      0,0,0,0,0,0,0,0,deg2rad(10)^2;...
      ;

      ms = unscented::unscentedTransform(X,Px,&uvdarHexarotorPose2p,{points[0].z,points[1].z,points[2].z});
    }

    else if (points.size() == 1) {
      std::cout << "Only single point visible - no distance information" << std::endl;
      angleDist = 0.0;
      std::cout << "led: " << points[0] << std::endl;
      double v1[3];
      double va[2] = {double(points[0].y), double(points[0].x)};
      ;
      cam2world(v1, va, &oc_model);

      Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);

      if (first) {
        distanceSlider.filterInit(farDistance, filterDistLength);
        orientationSlider.filterInit(0.0, filterOrientationLength);
        first = false;
      }
      orientationSlider.filterPush(0.0);

      double distanceFiltered = distanceSlider.filterSlide();
      centerEstimInCam        = distanceSlider.filterSlide() * V1;
      std::cout << "Estimated center in CAM: " << centerEstimInCam << std::endl;
      geometry_msgs::Pose p;
      p.position.x = centerEstimInCam.x();
      p.position.y = centerEstimInCam.y();
      p.position.z = centerEstimInCam.z();
      targetInCamPub.publish(p);
      foundTarget = true;
      lastSeen    = ros::Time::now();
    } else {
      std::cout << "No valid points seen. Waiting" << std::endl;
      centerEstimInCam.x()  = 0;
      centerEstimInCam.y()  = 0;
      centerEstimInCam.z()  = 0;
      centerEstimInBase.x() = 0;
      centerEstimInBase.y() = 0;
      centerEstimInBase.z() = 0;
      goalInBase.x()        = 0;
      goalInBase.y()        = 0;
      goalInBase.z()        = 0;
      tailingComponent      = 0;
      foundTarget           = false;
      return;
    }


      /* std::cout << "Estimated direction in CAM: " << (Rp*V2) << std::endl; */
      /* std::cout << "Central LED direction in CAM: " << (V2) << std::endl; */
      /* std::cout << "Rotation: " << Rp.matrix()   << std::endl; */

      foundTarget = true;
      lastSeen    = ros::Time::now();

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
      foundTarget = true;
      lastSeen    = ros::Time::now();

    }

    tf::Vector3 goalInCamTF, centerEstimInCamTF;
    tf::vectorEigenToTF(goalInCam, goalInCamTF);
    tf::vectorEigenToTF(centerEstimInCam, centerEstimInCamTF);
    mutex_tf.lock();
    tf::Vector3 goalInBaseTF        = (transformCam2Base * goalInCamTF);
    tf::Vector3 centerEstimInBaseTF = (transformCam2Base * centerEstimInCamTF);
    mutex_tf.unlock();
    Eigen::Affine3d eigenTF;
    /* ROS_INFO("TF parent: %s", transform.frame_id_.c_str()); */
    /* std::cout << "fcu_" + uav_name << std::endl; */
    /* tf::transformTFToEigen(transform, eigenTF); */
    /* std::cout << "TF mat: " << eigenTF.matrix() << std::endl; */
    tf::vectorTFToEigen(goalInBaseTF, goalInBase);
    tf::vectorTFToEigen(centerEstimInBaseTF, centerEstimInBase);

    Eigen::Vector3d CEBFlat(centerEstimInBase);
    double          flatLen = sqrt(CEBFlat.x() * CEBFlat.x() + CEBFlat.y() * CEBFlat.y());
    CEBFlat                 = CEBFlat / flatLen;
    goalInBase              = (flatLen - followDistance) * (CEBFlat);
    goalInBase.z()          = centerEstimInBase.z();

    tailingComponent = angleDist * flatLen * tailingCoeff;

    std::cout << "Tailing component: " << tailingComponent << std::endl;

    /* std::cout << "Goal in CAM: " << goalInCam << std::endl; */
    std::cout << "Goal in BASE: " << goalInBase << std::endl;
    geometry_msgs::Pose p;
    p.position.x = goalInBase.x();
    p.position.y = goalInBase.y();
    p.position.z = goalInBase.z();
    goalposInBasePub.publish(p);
    std::cout << "Center in BASE: " << centerEstimInBase << std::endl;
    p.position.x = centerEstimInBase.x();
    p.position.y = centerEstimInBase.y();
    p.position.z = centerEstimInBase.z();
    targetInBasePub.publish(p);
    /* if (reachedTarget) */
    /*   ROS_INFO("Reached target"); */
    /* tf::Vector3 centerEstimInCamTF; */
    /* tf::vectorEigenToTF(centerEstimInCam, centerEstimInCamTF); */
    /* tf::Vector3 centerEstimInBaseTF = (transform * centerEstimInCamTF); */
    /* tf::vectorTFToEigen(centerEstimInBaseTF, centerEstimInBase); */
    /* std::cout << "Estimated center in BASE: " << centerEstimInBase << std::endl; */
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

  bool first;
  bool stopped;

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

  double tailingComponent;

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



  double followDistance;
  double trajcoeff;
  double yawcoeff;
  double tailingCoeff;

  /* uvLedDetect_gpu *uvdg; */

  // thread
  std::thread target_thread;
  std::thread tf_thread;

  std::vector< sensor_msgs::Imu > imu_register;

  struct ocam_model oc_model;

  ros::ServiceClient             client;
  mrs_msgs::Vec4              tpnt;
  bool                           foundTarget;
  Eigen::Vector3d                centerEstimInBase;
  Eigen::Vector3d                goalInBase;

  /* bool   toRight;    // direction in which we should go to reach the tail */
  double angleDist;  // how large is the angle around the target between us and the tail

  Eigen::MatrixXd Px2,Px3;


  ros::Subscriber OdomSubscriber;
  ros::Subscriber DiagSubscriber;
  bool            reachedTarget;
  ros::Time       lastSeen;

  bool               followTriggered;
  ros::ServiceServer ser_trigger;

  ros::Publisher targetInCamPub;
  ros::Publisher targetInBasePub;
  ros::Publisher goalposInWorldPub;
  ros::Publisher goalposInBasePub;
  ros::Publisher yawdiffPub;
  ros::Publisher setyawPub;
  ros::Publisher yawodomPub;
  ros::Publisher setpointPub;
  ros::Publisher measuredDist;
  ros::Publisher filteredDist;


  int    filterDistLength, filterOrientationLength;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "uvdar_follower");
  ros::NodeHandle nodeA;
  PoseReporter        pr(nodeA);

  ROS_INFO("Directed follower node initiated");

  ros::spin();

  return 0;
}
