
#define camera_delay 0.50
#define armLength 0.2775
#define maxSpeed 2.0

#define min_frequency 4.8
#define max_frequency 36.0
#define boundary_ratio 0.5

/* #include <std_srvs/Trigger.h> */
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
/* #include <image_transport/image_transport.h> */
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/TrackerDiagnostics.h>
/* #include <mrs_msgs/TrackerTrajectorySrv.h> */
#include <mrs_msgs/Vec1.h>
/* #include <nav_msgs/Odometry.h> */
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
/* #include <mrs_lib/mrs_lib/Lkf.h> */

static double sqr(double a){
  return a*a;
}

static double cot(double input) {
  return cos(input) / sin(input);
}

static double deg2rad(double input) {
  return input*0.01745329251;
}

namespace enc = sensor_msgs::image_encodings;

class PoseReporter {
public:
  PoseReporter(ros::NodeHandle& node) {
    reachedTarget   = false;
    followTriggered = false;
    ros::NodeHandle private_node_handle("~");
    private_node_handle.param("uav_name", uav_name, std::string());
    /* ROS_INFO("UAV_NAME: %s",uav_name.c_str()); */

    private_node_handle.param("DEBUG", DEBUG, bool(false));

    private_node_handle.param("gui", gui, bool(false));

    private_node_handle.param("frequenciesPerTarget", frequenciesPerTarget, int(4));
    private_node_handle.param("targetCount", targetCount, int(4));
    int frequencyCount = targetCount*frequenciesPerTarget;

      
    int tempFreq;
    if (frequencySet.size() < frequencyCount) {
      node.param("frequency1", tempFreq, int(6));
      frequencySet.push_back(double(tempFreq));
    }
    if (frequencySet.size() < frequencyCount) {
      node.param("frequency2", tempFreq, int(10));
      frequencySet.push_back(double(tempFreq));
    }
    if (frequencySet.size() < frequencyCount) {
      node.param("frequency3", tempFreq, int(15));
      frequencySet.push_back(double(tempFreq));
    }
    if (frequencySet.size() < frequencyCount) {
      node.param("frequency4", tempFreq, int(30));
      frequencySet.push_back(double(tempFreq));
    }
    if (frequencySet.size() < frequencyCount) {
      node.param("frequency5", tempFreq, int(8));
      frequencySet.push_back(double(tempFreq));
    }
    if (frequencySet.size() < frequencyCount) {
      node.param("frequency6", tempFreq, int(12));
      frequencySet.push_back(double(tempFreq));
    }


    prepareFrequencyClassifiers();

    for (int i=0; i<2;i++){
      targetAcquired[i]=false;
    }



    gotCamInfo = false;

    char calib_path[100];

    sprintf(calib_path, "%s/include/OCamCalib/config/calib_results.txt", ros::package::getPath("uvdar").c_str());

    get_ocam_model(&oc_model, calib_path);

    targetInCamPub    = node.advertise< geometry_msgs::Pose >("targetInCam", 1);
    targetInBasePub   = node.advertise< geometry_msgs::Pose >("targetInBase", 1);
    yawdiffPub        = node.advertise< std_msgs::Float32 >("yawDifference", 1);
    yawodomPub        = node.advertise< std_msgs::Float32 >("yawOdom", 1);
    setpointPub       = node.advertise< geometry_msgs::Pose >("relativeSetpoint", 1);
    setyawPub         = node.advertise< std_msgs::Float32 >("relativeSetyaw", 1);

    measuredDist = node.advertise< std_msgs::Float32 >("measuredDist", 1);

    /* measuredPose = node.advertise<nav_msgs::Odometry>("measuredPose", 1); */
    for (int i=0;i<targetCount;i++){
      measuredPose[i] = node.advertise< geometry_msgs::PoseWithCovarianceStamped >(std::string("measuredPose")+std::to_string(i+1), 1);
    }

    X2 = Eigen::VectorXd(9,9);
    X3 = Eigen::VectorXd(10,10);
    Px2 = Eigen::MatrixXd(9,9);
    Px3 = Eigen::MatrixXd(10,10);


    first            = true;
    pointsSubscriber = node.subscribe("blinkersSeen", 1, &PoseReporter::ProcessPoints, this);


    framerateEstim = -1.0;
    framerateSubscriber = node.subscribe("estimatedFramerate", 1, &PoseReporter::GetFramerate, this);

    foundTarget = false;


    tf_thread   = std::thread(&PoseReporter::TfThread, this);

    for (int i = 0; i < targetCount; i++) {
      separatedPoints.push_back(std::vector< cv::Point3i >());
    }
  


  }

  ~PoseReporter() {
  }

  Eigen::VectorXd uvdarHexarotorPose3p(Eigen::VectorXd X, Eigen::VectorXd expFrequencies){
      cv::Point3d tmp;
      cv::Point3d a(X(0),X(1),X(2));
      cv::Point3d b(X(3),X(4),X(5));
      cv::Point3d c(X(6),X(7),X(8));
      double ambig = X(9);

      if ((c.x) < (a.x)) {
        tmp = a;
        a = c;
        c = tmp;
      }
      if ((b.x) < (a.x)) {
        tmp = b;
        b = a;
        a = tmp;
      }
      if ((b.x) > (c.x)) {
        tmp = c;
        c   = b;
        b   = tmp;
      }
      std::cout << "central led: " << b << std::endl;
      Eigen::Vector3i ids;
      ids << 0,1,2;
      Eigen::Vector3d expPeriods;
      expPeriods = expFrequencies.cwiseInverse(); 
      Eigen::Vector3d periods;
      periods << a.z,b.z,c.z;
      Eigen::Vector3d id;
      Eigen::MatrixXd::Index   minIndex;
      ((expPeriods.array()-(periods(0))).cwiseAbs()).minCoeff(&minIndex);
      id(0) = minIndex;
      ((expPeriods.array()-(periods(1))).cwiseAbs()).minCoeff(&minIndex);
      id(1) = minIndex;
      ((expPeriods.array()-(periods(2))).cwiseAbs()).minCoeff(&minIndex);
      id(2) = minIndex;


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

      Eigen::Vector3d norm13=V3.cross(V1);
      norm13=norm13/norm13.norm();
      double dist132=V2.dot(norm13);
      Eigen::Vector3d V2_c=V2-dist132*norm13;
      V2_c=V2_c/V2_c.norm();

      double Alpha = acos(V1.dot(V2_c));
      double Beta  = acos(V2_c.dot(V3));
  
      double A = 1.0 / tan(Alpha);
      double B = 1.0 / tan(Beta);
      std::cout << "alpha: " << Alpha << " beta: " << Beta << std::endl;
      std::cout << "A: " << A << " B: " << B << std::endl;

      double O = (A * A - A * B + sqrt(3.0) * A + B * B + sqrt(3.0) * B + 3.0);
      /* std::cout << "long operand: " << O << std::endl; */
      double delta = 2.0 * atan(((B * (2.0 * sqrt(O / (B * B + 2.0 * sqrt(3.0) + 3.0)) - 1.0)) +
                                 (6.0 * sqrt(O / ((sqrt(3.0) * B + 3.0) * (sqrt(3.0) * B + 3.0)))) + (2.0 * A + sqrt(3.0))) /
                                (sqrt(3.0) * B + 3.0));


      /* double gamma      = CV_PI - (delta + Alpha); */

      /* double distMiddle = sin(gamma) * armLength / sin(Alpha); */
      double distMiddle=(armLength*sin(M_PI-(delta+Alpha)))/(sin(Alpha));


      double l = sqrt(fmax(0.1, distMiddle * distMiddle + armLength * armLength - 2 * distMiddle * armLength * cos(delta + (CV_PI / 3.0))));

      double Epsilon=asin((armLength/l)*sin(delta+M_PI/3));
      /* phi=asin((b/l)*sin(delta+pi/3)); */

      /* double phi = asin(sin(delta + (CV_PI / 3.0)) * (armLength / l)); */
      double phi = asin(sin(delta + (CV_PI / 3.0)) * (distMiddle / l));
      std::cout << "delta: " << delta << std::endl;
      std::cout << "Estimated distance: " << l << std::endl;
      /* std_msgs::Float32 dM, fdM; */
      /* dM.data  = distance; */
      /* fdM.data = distanceFiltered; */
      /* measuredDist.publish(dM); */
      /* filteredDist.publish(fdM); */

      std::cout << "Estimated angle from mid. LED: " << phi * (180.0 / CV_PI) << std::endl;

      double C=acos(V2_c.dot(V2));
      Eigen::Vector3d V2_d=V2_c-V2;
      if (V2_d(1)<0)
        C=-C;
      double t=acos(V1.dot(V3));

      double Omega1=asin(fmax(-1.0,fmin(1.0,(C/t)*(2.0*sqrt(3.0)))));

      /* Eigen::Vector3d Pv = V2.cross(V1).normalized(); */
      /* Rc = makehgtform('axisrotate',norm13,epsilon); */
      Eigen::Transform< double, 3, Eigen::Affine > Rc(Eigen::AngleAxis< double >(Epsilon, norm13));
      /* vc=Rc(1:3,1:3)*v2_c; */
      Eigen::Vector3d Vc = Rc*V2_c;

      Eigen::VectorXd Yt=l*Vc;
      /* goalInCam        = (distanceFiltered - followDistance) * (Rp * V2); */
      /* tf::Vector3 centerEstimInCamTF; */
      /* tf::vectorEigenToTF(centerEstimInCam, centerEstimInCamTF); */
      /* tf::Vector3 centerEstimInBaseTF = (transform * centerEstimInCamTF); */
      /* /1* std::cout << centerEstimInBaseTF << std::endl; *1/ */
      /* tf::vectorTFToEigen(centerEstimInBaseTF, centerEstimInBase); */



      /* std::cout << "Estimated center in CAM: " << Yt << std::endl; */

      double tilt_perp=Omega1+atan2(Vc(1),sqrt(sqr(Vc(2))+sqr(Vc(0))));

      double relyaw;

      ROS_INFO_STREAM("leds: " << id);

      if (expFrequencies.size() == 2){
        if     ((id(0)==ids[0]) && (id(1)==ids[0]) && (id(2)==ids[0]))
          relyaw=(M_PI/2);
        else if ((id(0)==ids[1]) && (id(1)==ids[1]) && (id(2)==ids[1]))
        relyaw=(-M_PI/2);
        else if ((id(0)==ids[0]) && (id(1)==ids[0]) && (id(2)==ids[1]))
        relyaw=(M_PI/6);
        else if ((id(0)==ids[0]) && (id(1)==ids[1]) && (id(2)==ids[1]))
        relyaw=(-M_PI/6);
        else if ((id(0)==ids[1]) && (id(1)==ids[1]) && (id(2)==ids[0]))
        relyaw=(-5*M_PI/6);
        else if ((id(0)==ids[1]) && (id(1)==ids[0]) && (id(2)==ids[0]))
        relyaw=(5*M_PI/6);
        else
          if (id(0)==ids[0])
            relyaw=(M_PI/2)+ambig;
          else
            relyaw=(-M_PI/2)+ambig;
      }
      else {
        if     ((id(0)==ids[2]) && (id(1)==ids[0]) && (id(2)==ids[0]))
          relyaw=(M_PI/2);
        else if ((id(0)==ids[1]) && (id(1)==ids[1]) && (id(2)==ids[2]))
        relyaw=(-M_PI/2);
        else if ((id(0)==ids[0]) && (id(1)==ids[0]) && (id(2)==ids[1]))
        relyaw=(M_PI/6);
        else if ((id(0)==ids[0]) && (id(1)==ids[1]) && (id(2)==ids[1]))
        relyaw=(-M_PI/6);
        else if ((id(0)==ids[1]) && (id(1)==ids[2]) && (id(2)==ids[2]))
        relyaw=(-5*M_PI/6);
        else if ((id(0)==ids[2]) && (id(1)==ids[2]) && (id(2)==ids[0]))
          relyaw=(5*M_PI/6);
        else
          if (id(0)==ids[0])
            relyaw=(M_PI/2)+ambig;
          else
            relyaw=(-M_PI/2)+ambig;
      }

      double latnorm=sqrt(sqr(Yt(0))+sqr(Yt(2)));
      double Gamma=atan2(Yt(1),latnorm);
      Eigen::Vector3d obs_normal =V3.cross(V1); //not exact, but in practice changes very little
      obs_normal=obs_normal/(obs_normal.norm());
      Eigen::Vector3d latComp;
      latComp << Vc(0),0,Vc(2);
      latComp = latComp/(latComp.norm());
      if (Vc(1)<0) latComp = -latComp;

      /* Re = makehgtform('axisrotate',cross(vc,latComp),Gamma+pi/2); */
      Eigen::Transform< double, 3, Eigen::Affine > Re(Eigen::AngleAxis< double >( Gamma+(M_PI/2),(Vc.cross(latComp)).normalized()));
      Eigen::Vector3d exp_normal=Re*Vc;
      /* Ro = makehgtform('axisrotate',cross(vc,obs_normal),Gamma); */
      Eigen::Transform< double, 3, Eigen::Affine > Ro(Eigen::AngleAxis< double >( Gamma,(Vc.cross(obs_normal)).normalized()));
      obs_normal=Ro*obs_normal;
      /* obs_normal=Ro(1:3,1:3)*obs_normal; */
      /* exp_normal=Re(1:3,1:3)*vc; */
      double tilt_par=acos(obs_normal.dot(exp_normal));
      if (V1(1)<V2(1))
        tilt_par=-tilt_par;


      ROS_INFO_STREAM("latComp: " << latComp);
      ROS_INFO_STREAM("Vc: " << Vc);
      ROS_INFO_STREAM("Gamma: " << Gamma);
      ROS_INFO_STREAM("exp_normal: " << exp_normal);
      ROS_INFO_STREAM("obs_normal: " << obs_normal);
      ROS_INFO_STREAM("tilt_par: " << tilt_par);
      ROS_INFO_STREAM("tilt_perp: " << tilt_perp);

      double relyaw_view=relyaw+phi;
      relyaw=relyaw-atan2(Vc(0),Vc(2))+phi;

      double xl=(armLength/2)*cos(phi);
      double dist = Yt.norm();
      /* % X(1:3)=Xt(1:3) */
      Eigen::VectorXd Y(6);
      Yt = Yt*((dist-xl)/dist);
      latnorm=sqrt(sqr(Y(0))+sqr(Y(2)));
      double latang=atan2(Yt(0),Yt(2));
      Y(1)=Yt(1)-xl*sin(tilt_perp)*cos(tilt_par);
      Y(0)=Yt(0)-xl*sin(tilt_perp)*sin(tilt_par)*cos(latang)+xl*cos(tilt_perp)*sin(latang);
      Y(2)=Yt(2)+xl*sin(tilt_perp)*sin(tilt_par)*sin(latang)+xl*cos(tilt_perp)*cos(latang);


      double reltilt_abs=atan(sqrt(sqr(tan(tilt_perp))+sqr(tan(tilt_par))));
      double tiltdir=atan2(-tan(tilt_par),tan(tilt_perp));
      double tiltdir_adj=tiltdir-relyaw_view;
      double ta=cos(tiltdir_adj);
      double tb=-sin(tiltdir_adj);
      double tc=cot(reltilt_abs);
      double tpitch=atan2(ta,tc);
      double troll=atan2(tb,tc);

      Y << Yt.x(),Yt.y(),Yt.z(),troll,tpitch,relyaw;
      return Y;
  }

  static Eigen::Matrix3d rotate_covariance(const Eigen::Matrix3d& covariance, const Eigen::Matrix3d& rotation) {
    return rotation * covariance * rotation.transpose();  // rotate the covariance to point in direction of est. position
  }

  //Courtesy of Matous Vrba
  static Eigen::Matrix3d calc_position_covariance(const Eigen::Vector3d& position_sf, const double xy_covariance_coeff, const double z_covariance_coeff) {
    /* Calculates the corresponding covariance matrix of the estimated 3D position */
    Eigen::Matrix3d pos_cov = Eigen::Matrix3d::Identity();  // prepare the covariance matrix
    const double tol = 1e-9;
    pos_cov(0, 0) = pos_cov(1, 1) = xy_covariance_coeff;

    pos_cov(2, 2) = position_sf(2) * sqrt(position_sf(2)) * z_covariance_coeff;
    if (pos_cov(2, 2) < 0.33 * z_covariance_coeff)
      pos_cov(2, 2) = 0.33 * z_covariance_coeff;

    // Find the rotation matrix to rotate the covariance to point in the direction of the estimated position
    const Eigen::Vector3d a(0.0, 0.0, 1.0);
    const Eigen::Vector3d b = position_sf.normalized();
    const Eigen::Vector3d v = a.cross(b);
    const double sin_ab = v.norm();
    const double cos_ab = a.dot(b);
    Eigen::Matrix3d vec_rot = Eigen::Matrix3d::Identity();
    if (sin_ab < tol)  // unprobable, but possible - then it is identity or 180deg
    {
      if (cos_ab + 1.0 < tol)  // that would be 180deg
      {
        vec_rot << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
      }     // otherwise its identity
    } else  // otherwise just construct the matrix
    {
      Eigen::Matrix3d v_x;
      v_x << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
      vec_rot = Eigen::Matrix3d::Identity() + v_x + (1 - cos_ab) / (sin_ab * sin_ab) * (v_x * v_x);
    }
    pos_cov = rotate_covariance(pos_cov, vec_rot);  // rotate the covariance to point in direction of est. position
    return pos_cov;
  }

  unscented::measurement uvdarHexarotorPose1p_meas(Eigen::Vector2d X,double tubewidth, double meanDist){
    double v1[3];
    double x[2] = {X.y(),X.x()};
    cam2world(v1, x, &oc_model);
    Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
    V1 = V1*meanDist;

    unscented::measurement ms;
    ms.x = Eigen::VectorXd(6);
    ms.C = Eigen::MatrixXd(6,6);

    ms.x << V1.x(),V1.y(),V1.z(),0,0,0; 
    Eigen::MatrixXd temp;
    temp.setIdentity(6,6);
    ms.C = temp*666;//large covariance for angles in radians
    ms.C.topLeftCorner(3, 3) = calc_position_covariance(V1,tubewidth,meanDist/3);

    /* std::cout << "ms.C: " << ms.C << std::endl; */


    return ms;

  }

  Eigen::VectorXd uvdarHexarotorPose2p(Eigen::VectorXd X, Eigen::VectorXd expFrequencies){

    /* ROS_INFO_STREAM("X: " << X); */

      cv::Point3d a;
      cv::Point3d b;

      if ((X(0)) < (X(3))) {
        a = cv::Point3d(X(0),X(1),X(2));
        b = cv::Point3d(X(3),X(4),X(5));
      } else {
        a = cv::Point3d(X(3),X(4),X(5));
        b = cv::Point3d(X(0),X(1),X(2));
      }
      double ambig=X(7);
      double delta=X(6);

      /* std::cout << "right led: " << b << std::endl; */
      Eigen::Vector3i ids;
      ids << 0,1,2;
      Eigen::Vector3d expPeriods;
      expPeriods = expFrequencies.cwiseInverse(); 
      Eigen::Vector3d periods;
      periods << a.z,b.z;
      Eigen::Vector3d id;
      Eigen::MatrixXd::Index   minIndex;
      ((expPeriods.array()-(periods(0))).cwiseAbs()).minCoeff(&minIndex);
      id(0) = minIndex;
      ((expPeriods.array()-(periods(1))).cwiseAbs()).minCoeff(&minIndex);
      id(1) = minIndex;




      cv::Point3d central = (a+b) / 2.0;
      double      v1[3], v2[3];
      double      va[2] = {double(a.y), double(a.x)};
      double      vb[2] = {double(b.y), double(b.x)};
      ;
      cam2world(v1, va, &oc_model);
      cam2world(v2, vb, &oc_model);
      /* double vc[3]; */
      /* double pc[2] = {central.y, central.x}; */
      /* cam2world(vc, pc, &oc_model); */

      Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
      Eigen::Vector3d V2(v2[1], v2[0], -v2[2]);
      /* Eigen::Vector3d Vc(vc[1], vc[0], -vc[2]); */

      /* double alpha = acos(V1.dot(V2)); */

      /* double vd = sqrt(0.75 * armLength); */

      /* double distance = (armLength / 2.0) / tan(alpha / 2.0) + vd; */
      /* if (first) { */
      /*   distanceSlider.filterInit(distance, filterDistLength); */
      /*   orientationSlider.filterInit(angleDist, filterOrientationLength); */
      /*   first = false; */
      /* } */
      double d = armLength;
      double v=d*sqrt(3.0/4.0);
      double sqv=v*v;
      double sqd=d*d;
      double csAlpha = (V1.dot(V2));
      double Alpha=acos(csAlpha);
      double Alpha2=Alpha*Alpha;
      double snAlpha =sin(Alpha);
      double sndelta =sin(delta);
      double sn2delta =sin(2*delta);
      double csdelta =cos(delta);
      double cs2delta =cos(2*delta);

      /* ROS_INFO("Alpha: %f, v: %f, d: %f, delta: %f",Alpha, v, d, delta); */
      /* ROS_INFO_STREAM("V1:" << V1 << std::endl <<"V2: " << V2); */

      double l =
        (4*d*v*Alpha - 
         sqd*csAlpha - sqd*cos(Alpha - 2*delta) - 
         6*d*v*snAlpha - 2*d*v*sin(Alpha - 2*delta) + 
         sqd*Alpha*sn2delta + 4*sqv*Alpha*sn2delta - 
         sqrt(2)*sqrt(
           sqr(d*csdelta - 2*v*sndelta)*
           (
            sqd - sqd*Alpha2 - 4*sqv*Alpha2 - 4*d*v*Alpha*csAlpha + 
            4*d*v*Alpha*cos(Alpha - 2*delta) + 
            sqd*cos(2*(Alpha - delta)) - 
            sqd*Alpha2*cs2delta + 
            4*sqv*Alpha2*cs2delta + 
            2*sqd*Alpha*snAlpha + 
            2*sqd*Alpha*sin(Alpha - 2*delta) - 
            4*d*v*Alpha2*sn2delta)))
        /
        (4*d*csdelta*(Alpha - 2*snAlpha) + 8*v*Alpha*sndelta);

      /* distanceSlider.filterPush(distance); */
      /* orientationSlider.filterPush(angleDist); */

      /* std::cout << "Estimated distance: " << l << std::endl; */
      /* std::cout << "Filtered distance: " << distanceFiltered << std::endl; */

      /* std::cout << "Estimated direction in CAM: " << (Rp*V2) << std::endl; */
      /* std::cout << "Central LED direction in CAM: " << (V2) << std::endl; */
      /* std::cout << "Rotation: " << Rp.matrix()   << std::endl; */

      /* foundTarget = true; */
      /* lastSeen    = ros::Time::now(); */

      /* std_msgs::Float32 dM, fdM; */
      /* dM.data  = distance; */
      /* fdM.data = distanceFiltered; */
      /* measuredDist.publish(dM); */
      /* filteredDist.publish(fdM); */



      double kappa = M_PI/2-delta;
      double d1=(d/2)+v*tan(delta);
      double xl=v/cos(delta);
      double yl=l-xl;
      double alpha1=atan((d1*sin(kappa))/(yl-d1*cos(kappa)));
      Eigen::Vector3d Pv = V2.cross(V1).normalized();
      Eigen::Transform< double, 3, Eigen::Affine > Rp(Eigen::AngleAxis< double >(-alpha1, Pv));
      /* Rc = makehgtform('axisrotate',cross(v2,v1),-alpha1); */
      Eigen::Vector3d Vc=Rp*V1;
      Eigen::Vector3d Yt=l*Vc;

      /* std::cout << "Estimated center in CAM: " << Yt << std::endl; */
      /* geometry_msgs::Pose p; */
      /* p.position.x = centerEstimInCam.x(); */
      /* p.position.y = centerEstimInCam.y(); */
      /* p.position.z = centerEstimInCam.z(); */
      /* targetInCamPub.publish(p); */
      /* foundTarget = true; */
      /* lastSeen    = ros::Time::now(); */

      double relyaw;

      if (expFrequencies.size() == 2)
        if     ((id(0)==ids[0]) && (id(1)==ids[0]))
          relyaw=(M_PI/2)+ambig+delta;
        else if ((id(0)==ids[1]) && (id(1)==ids[1]))
          relyaw=(-M_PI/2)+ambig+delta;
        else if ((id(0)==ids[0]) && (id(1)==ids[1]))
          relyaw=0+delta;
        else
          relyaw=M_PI+delta;

        else 
          if     ((id(0)==ids[0]) && (id(1)==ids[0]))
            relyaw=(M_PI/3)+delta;
          else if ((id(0)==ids[1]) && (id(1)==ids[1]))
            relyaw=(-M_PI/3)+delta;
          else if ((id(0)==ids[0]) && (id(1)==ids[1]))
            relyaw=0+delta;
          else if ((id(0)==ids[1]) && (id(1)==ids[2]))
            relyaw=(-2*M_PI/3)+delta;
          else if ((id(0)==ids[2]) && (id(1)==ids[0]))
            relyaw=(2*M_PI/3)+delta;
          else if ((id(0)==ids[2]) && (id(1)==ids[2]))
            relyaw=(M_PI)+delta;
          else
            relyaw=ambig+delta;
        
      double latang=atan2(Vc(0),Vc(2));

      double relyaw_view=relyaw;

      /* ROS_INFO_STREAM("expFrequencies: " << expFrequencies); */
      /* ROS_INFO_STREAM("expPeriods: " << expPeriods); */
      /* ROS_INFO_STREAM("periods: " << periods); */
      /* ROS_INFO_STREAM("id: " << id); */
      /* ROS_INFO("relyaw_orig: %f",relyaw); */
      relyaw=relyaw-latang;
      /* ROS_INFO_STREAM("Vc: " << Vc); */
      /* ROS_INFO("latang: %f",latang); */

      double latnorm=sqrt(sqr(Yt(0))+sqr(Yt(2)));
      double Gamma=atan2(Yt(1),latnorm);
      double tilt_perp=X(8);
      Eigen::Vector3d obs_normal=V2.cross(V1);
      obs_normal=obs_normal/(obs_normal.norm());
      Eigen::Vector3d latComp;
      latComp << Vc(0),0,Vc(2);
      latComp = latComp/(latComp.norm());

      if (Vc(1)<0) latComp = -latComp;
      /* ROS_INFO_STREAM("cross: " << Vc.cross(latComp)); */

      Eigen::Transform< double, 3, Eigen::Affine > Re(Eigen::AngleAxis< double >( Gamma+(M_PI/2),(Vc.cross(latComp)).normalized()));
      Eigen::Vector3d exp_normal=Re*Vc;
      /* Ro = makehgtform('axisrotate',cross(vc,obs_normal),Gamma); */
      Eigen::Transform< double, 3, Eigen::Affine > Ro(Eigen::AngleAxis< double >( Gamma,(Vc.cross(obs_normal)).normalized()));
      obs_normal=Ro*obs_normal;
      /* Re = makehgtform('axisrotate',cross(vc,latComp),Gamma+pi/2); */
      double tilt_par=acos(obs_normal.dot(exp_normal));
      if (V1(1)<V2(1))
        tilt_par=-tilt_par;


/*       ROS_INFO_STREAM("exp_normal: " << exp_normal); */
/*       ROS_INFO_STREAM("obs_normal: " << obs_normal); */
/*       ROS_INFO_STREAM("tilt_par: " << tilt_par); */

      double dist = Yt.norm();
      Yt= Yt*((dist-xl)/dist);
      Yt(1)=Yt(1)-xl*sin(Gamma+tilt_perp)*cos(tilt_par);
      Yt(0)=Yt(0)-xl*sin(Gamma+tilt_perp)*sin(tilt_par)*cos(latang)+xl*cos(Gamma+tilt_perp)*sin(latang);
      Yt(2)=Yt(2)+xl*sin(Gamma+tilt_perp)*sin(tilt_par)*sin(latang)+xl*cos(Gamma+tilt_perp)*cos(latang);


      double reltilt_abs=atan(sqrt(sqr(tan(tilt_perp))+sqr(tan(tilt_par))));
      double tiltdir=atan2(-tan(tilt_par),tan(tilt_perp));
      double tiltdir_adj=tiltdir-relyaw_view;
      double ta=cos(tiltdir_adj);
      double tb=-sin(tiltdir_adj);
      double tc=cot(reltilt_abs);
      double tpitch=atan2(ta,tc);
      double troll=atan2(tb,tc);

      Eigen::VectorXd Y(6);
      Y << Yt.x(),Yt.y(),Yt.z(),troll,tpitch,relyaw;

      return Y;

       
    }



  void TfThread() {
    gotCam2Base = false;
    ros::Rate transformRate(1.0);
    while (true) {
      transformRate.sleep();
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
      break;;
      /* ROS_INFO("TF next"); */
    }
    gotCam2Base = true;
  }



  void GetFramerate(const std_msgs::Float32ConstPtr& msg) {
    framerateEstim = msg->data;
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
    if (framerateEstim < 0) {
      ROS_INFO("Framerate is not yet estimated. Waiting...");
      return;
    }
    if (!gotCam2Base) {
      ROS_INFO("Transformation to base is missing...");
      return;
    }

    if (first) {
      /* trackers[1]= new Lkf::Lkf(6, const int m, const int p, const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Q, */
      targetAcquired[1]=true;
      first = false;
      }

    for (int i = 0; i < countSeen; i++) {
      if (msg->data[(i * 3) + 2] <= 200) {
        points.push_back(cv::Point3i(msg->data[(i * 3)], msg->data[(i * 3) + 1], msg->data[(i * 3) + 2]));
      }
    }

    for (int i = 0; i < targetCount; i++) {
      separatedPoints[i].clear();
    }

    if (points.size() > 1) {

      for (int i = 0; i < points.size(); i++) {
        if (points[i].z >= 0) {
          separatedPoints[classifyMatch(findMatch(points[i].z))].push_back(points[i]);
        }
      }

      for (int i = 0; i < targetCount; i++) {
        ROS_INFO_STREAM("target [" << i << "]: ");
        ROS_INFO_STREAM("p: " << separatedPoints[i]);
        extractSingleRelative(separatedPoints[i], i);
      }
    }
  }

  void extractSingleRelative(std::vector< cv::Point3i > points, int target) {
    double leftF = frequencySet[target*2];
    double rightF = frequencySet[target*2+1];
    Eigen::Vector3d centerEstimInCam;
    double          maxDist = 100.0;

    int countSeen = (int)(points.size());


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


      unscented::measurement ms;

      ROS_INFO_STREAM("framerateEstim: " << framerateEstim);

      double perr=0.2/framerateEstim;

    if (points.size() == 3) {
      ROS_INFO_STREAM("ponts: " << points);
      X3 <<
        points[0].x ,points[0].y, 1.0/(double)(points[0].z),
        points[1].x ,points[1].y, 1.0/(double)(points[1].z),
        points[2].x, points[2].y, 1.0/(double)(points[2].z),
        0;  //to account for ambiguity
      Px3 <<
        1.0,0,0,0,0,0,0,0,0,0,
        0,1.0,0,0,0,0,0,0,0,0,
        0,0,sqr(perr),0,0,0,0,0,0,0,
        0,0,0,1.0,0,0,0,0,0,0,
        0,0,0,0,1.0,0,0,0,0,0,
        0,0,0,0,0,sqr(perr),0,0,0,0,
        0,0,0,0,0,0,1.0,0,0,0,
        0,0,0,0,0,0,0,1.0,0,0,
        0,0,0,0,0,0,0,0,sqr(perr),0,
        0,0,0,0,0,0,0,0,0,sqr(2*M_PI/3)
        ;
      boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd)> callback;
      callback=boost::bind(&PoseReporter::uvdarHexarotorPose3p,this,_1,_2);
      ms = unscented::unscentedTransform(X3,Px3,callback,leftF,rightF,-1);
    }
    else if (points.size() == 2) {
      ROS_INFO_STREAM("ponts: " << points);
      X2 <<
        (double)(points[0].x) ,(double)(points[0].y),1.0/(double)(points[0].z),
        (double)(points[1].x) ,(double)(points[1].y),1.0/(double)(points[1].z),
        0.0,0.0,0.0;
      Px2 <<
        1.0,0,0,0,0,0,0,0,0,
        0,1.0,0,0,0,0,0,0,0,
        0,0,sqr(perr),0,0,0,0,0,0,
        0,0,0,1.0,0,0,0,0,0,
        0,0,0,0,1.0,0,0,0,0,
        0,0,0,0,0,sqr(perr),0,0,0,
        0,0,0,0,0,0,sqr(deg2rad(8)),0,0,
        0,0,0,0,0,0,0,sqr(deg2rad(30)),0,
        0,0,0,0,0,0,0,0,sqr(deg2rad(10))
        ;

      ROS_INFO_STREAM("X2: " << X2);
      boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd)> callback;
      callback=boost::bind(&PoseReporter::uvdarHexarotorPose2p,this,_1,_2);
      ms = unscented::unscentedTransform(X2,Px2,callback,leftF,rightF,-1);
    }

    else if (points.size() == 1) {
      std::cout << "Only single point visible - no distance information" << std::endl;
      angleDist = 0.0;
      std::cout << "led: " << points[0] << std::endl;


      ms = uvdarHexarotorPose1p_meas(Eigen::Vector2d(points[0].x,points[0].y),armLength,10.0);


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
      tailingComponent      = 0;
      foundTarget           = false;
      return;
    }

    ROS_INFO_STREAM("Y: \n" << ms.x );
    /* std::cout << "Py: " << ms.C << std::endl; */
    ROS_INFO_STREAM("Py: \n" << ms.C );

    msgOdom = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();;
    /* msgOdom->twist.covariance = msgOdom->pose.covariance; */

    /* geometry_msgs::PoseWithCovarianceStampedPtr msgPose = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();; */
    /* geometry_msgs::PoseWithCovariancePtr msgPose = boost::make_shared<geometry_msgs::PoseWithCovariance>();; */
    /* msgPose->header.frame_id ="uvcam"; */
    /* msgPose->header.stamp = ros::Time::now(); */
    msgOdom->pose.pose.position.x = ms.x(0);
    msgOdom->pose.pose.position.y = ms.x(1);
    msgOdom->pose.pose.position.z = ms.x(2);
    tf::Quaternion qtemp;
    qtemp.setRPY(ms.x(3), ms.x(4), ms.x(5));
    qtemp=(transformCam2Base.getRotation().inverse())*qtemp;
    /* Eigen::Affine3d aTtemp; */
    /* tf::transformTFToEigen(transformCam2Base, aTtemp); */
    /* qtemp = aTtemp*qtemp; */
    msgOdom->pose.pose.orientation.x = qtemp.x();
    msgOdom->pose.pose.orientation.y = qtemp.y();
    msgOdom->pose.pose.orientation.z = qtemp.z();
    msgOdom->pose.pose.orientation.w = qtemp.w();
    for (int i=0; i<ms.C.cols(); i++){
      for (int j=0; j<ms.C.rows(); j++){
        msgOdom->pose.covariance[ms.C.cols()*j+i] =  ms.C(j,i);
      }
    }

    msgOdom->header.frame_id ="uvcam";
    msgOdom->header.stamp = ros::Time::now();
    /* msgOdom->pose = *(msgPose); */

    /* msgOdom->twist.twist.linear.x = 0.0; */
    /* msgOdom->twist.twist.linear.y = 0.0; */
    /* msgOdom->twist.twist.linear.z = 0.0; */
    /* msgOdom->twist.twist.angular.x = 0.0; */
    /* msgOdom->twist.twist.angular.y = 0.0; */
    /* msgOdom->twist.twist.angular.z = 0.0; */
    
    /* msgOdom->twist.covariance = { */
    /*   4,0,0,0,0,0, */
    /*   0,4,0,0,0,0, */
    /*   0,0,4,0,0,0, */
    /*   0,0,0,2,0,0, */
    /*   0,0,0,0,2,0, */
    /*   0,0,0,0,0,2}; */


    measuredPose[target].publish(msgOdom);





    tf::Vector3 goalInCamTF, centerEstimInCamTF;
    /* tf::vectorEigenToTF(goalInCam, goalInCamTF); */
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

    geometry_msgs::Pose p;
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

  int findMatch(double i_frequency) {
    double period = 1.0 / i_frequency;
    for (int i = 0; i < periodSet.size(); i++) {
      /* std::cout << period << " " <<  periodBoundsTop[i] << " " << periodBoundsBottom[i] << " " << periodSet[i] << std::endl; */
      if ((period > periodBoundsBottom[i]) && (period < periodBoundsTop[i])) {
        return i;
      }
    }
    return -1;
  }

  int classifyMatch(int ID) {
    return ID/frequenciesPerTarget;
  }

  void prepareFrequencyClassifiers() {
    for (int i = 0; i < frequencySet.size(); i++) {
      periodSet.push_back(1.0 / frequencySet[i]);
    }
    for (int i = 0; i < frequencySet.size() - 1; i++) {
      periodBoundsBottom.push_back((periodSet[i] * (1.0 - boundary_ratio) + periodSet[i + 1] * boundary_ratio));
    }
    periodBoundsBottom.push_back(1.0 / max_frequency);


    periodBoundsTop.push_back(1.0 / min_frequency);
    for (int i = 1; i < frequencySet.size(); i++) {
      periodBoundsTop.push_back((periodSet[i] * boundary_ratio + periodSet[i - 1] * (1.0 - boundary_ratio)));
    }


    /* periodBoundsTop.back()           = 0.5 * periodSet.back() + 0.5 * periodSet[secondToLast]; */
    /* periodBoundsBottom[secondToLast] = 0.5 * periodSet.back() + 0.5 * periodSet[secondToLast]; */
  }

private:


  double framerateEstim;


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
  ros::Subscriber framerateSubscriber;

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

  bool gui;

  int numberOfBins;

  bool cameraRotated;

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

  bool gotCam2Base;


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
  Eigen::VectorXd X2,X3;


  ros::Subscriber OdomSubscriber;
  ros::Subscriber DiagSubscriber;
  bool            reachedTarget;
  ros::Time       lastSeen;

  bool               followTriggered;
  ros::ServiceServer ser_trigger;

  ros::Publisher targetInCamPub;
  ros::Publisher targetInBasePub;
  ros::Publisher yawdiffPub;
  ros::Publisher setyawPub;
  ros::Publisher yawodomPub;
  ros::Publisher setpointPub;
  ros::Publisher measuredDist;

  ros::Publisher measuredPose[2];

  bool               targetAcquired[2];
  /* Lkf* trackers[2]; */

  /* nav_msgs::OdometryPtr msgOdom; */
  geometry_msgs::PoseWithCovarianceStampedPtr msgOdom;

  int frequenciesPerTarget;
  int targetCount;
  std::vector< std::vector< cv::Point3i > > separatedPoints;
  std::vector<double> frequencySet;
  std::vector<double> periodSet;
  std::vector<double> periodBoundsTop;
  std::vector<double> periodBoundsBottom;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "uvdar_reporter");
  ros::NodeHandle nodeA;
  PoseReporter        pr(nodeA);

  ROS_INFO("UVDAR Pose reporter node initiated");

  ros::spin();

  return 0;
}
