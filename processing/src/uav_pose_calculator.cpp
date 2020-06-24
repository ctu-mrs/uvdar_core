#define camera_delay 0.10
#define maxSpeed 2.0
#define maxDistInit 100.0

#define min_frequency 3
#define max_frequency 36.0
#define boundary_ratio 0.5

#define bracket_step 10

#define qpix 1 //pixel std. dev

#define TUBE_LENGTH (_beacon_?10:1000)

#include <ros/ros.h>
#include <ros/package.h>

#include <thread>
#include <mutex>
#include <numeric>
#include <boost/filesystem/operations.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <std_msgs/Float32.h>
#include <mrs_msgs/ImagePointsWithFloatStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "OCamCalib/ocam_functions.h"
#include <unscented/unscented.h>
#include <p3p/P3p.h>
#include <color_selector/color_selector.h>
#include <frequency_classifier/frequency_classifier.h>


#define sqr(X) ((X) * (X))
#define cot(X) (cos(X)/sin(X))
#define deg2rad(X) ((X)*0.01745329251)
#define rad2deg(X) ((X)*57.2957795131)

namespace e = Eigen;

namespace uvdar {

class UVDARPoseCalculator {
public:


  /* Constructor //{ */
  UVDARPoseCalculator(ros::NodeHandle& nh_) {
    ROS_INFO("[UVDARPoseCalculator]: Initializing pose calculator...");

    mrs_lib::ParamLoader param_loader(nh_, "UVDARPoseCalculator");

    param_loader.loadParam("uav_name", _uav_name_, std::string());

    param_loader.loadParam("debug", _debug_, bool(false));

    param_loader.loadParam("gui", _gui_, bool(false));

    param_loader.loadParam("arm_length",_arm_length_,double(0.2775));

    param_loader.loadParam("quadrotor",_quadrotor_,bool(false));

    param_loader.loadParam("beacon",_beacon_,bool(false));
    param_loader.loadParam("beacon_height",_beacon_height_,double(0.2));

    param_loader.loadParam("frequenciesPerTarget", frequenciesPerTarget, int(4));
    param_loader.loadParam("targetCount", _target_count_, int(4));
    
    // load the frequencies
    param_loader.loadParam("frequencies", _frequencies_);
    if (_frequencies_.empty()){
      std::vector<double> default_frequency_set;
      if (_beacon_){
        default_frequency_set = {30, 15};
      }
      else {
        default_frequency_set = {5, 6, 8, 10, 15, 30};
      }
      ROS_WARN("[UVDARBlinkProcessor]: No frequencies were supplied, using the default frequency set. This set is as follows: ");
      for (auto f : default_frequency_set){
        ROS_WARN_STREAM("[UVDARBlinkProcessor]: " << f << " hz");
      }
      _frequencies_ = default_frequency_set;
    }
    ufc_ = std::make_unique<UVDARFrequencyClassifier>(_frequencies_);

    if (_beacon_){
      prepareBlinkerBrackets();
    }

    /* subscribe to blinkersSeen //{ */
    std::vector<std::string> _blinkers_seen_topics;
    param_loader.loadParam("blinkers_seen_topics", _blinkers_seen_topics, _blinkers_seen_topics);
    if (_blinkers_seen_topics.empty()) {
      ROS_WARN("[UVDARPoseCalculator]: No topics of blinkers were supplied");
    }
    _camera_count_ = (unsigned int)(_blinkers_seen_topics.size());

    // Create callbacks for each camera
    blinkersSeenCallbacks.resize(_blinkers_seen_topics.size());
    separated_points_.resize(_blinkers_seen_topics.size());
    for (size_t i = 0; i < _blinkers_seen_topics.size(); ++i) {
      blinkers_seen_callback_t callback = [imageIndex=i,this] (const mrs_msgs::ImagePointsWithFloatStampedConstPtr& pointsMessage) { 
        ProcessPoints(pointsMessage, imageIndex);
      };
      blinkersSeenCallbacks[i] = callback;
      /* blinkersSeenCallbacks.push_back(callback); */
    // Subscribe to corresponding topics
      ROS_INFO_STREAM("[UVDARPoseCalculator]: Subscribing to " << _blinkers_seen_topics[i]);
      blinkersSeenSubscribers.push_back(
          nh_.subscribe(_blinkers_seen_topics[i], 1, &blinkers_seen_callback_t::operator(), &blinkersSeenCallbacks[i]));
    }

    /* Load calibration files //{ */
    param_loader.loadParam("calib_files", _calib_files_, _calib_files_);
    if (_calib_files_.empty()) {
      ROS_ERROR("[UVDARPoseCalculator]: No camera calibration files were supplied. You can even use \"default\" for the cameras, but no calibration is not permissible. Returning.");
      return;
    }
    if (!loadCalibrations()){
      ROS_ERROR("[UVDARPoseCalculator The camera calibration files could not be loaded!");
      return;
    }

    /* prepare masks if necessary //{ */
    param_loader.loadParam("use_masks", _use_masks_, bool(false));
    if (_use_masks_){
      param_loader.loadParam("mask_file_names", _mask_file_names_, _mask_file_names_);

      if (_mask_file_names_.size() != _camera_count_){
        ROS_ERROR_STREAM("[UVDARPoseCalculator Masks are enabled, but the number of mask filenames provided does not match the number of camera topics (" << _camera_count_ << ")!");
        return;
      }

      if (!loadMasks()){
        ROS_ERROR("[UVDARPoseCalculator Masks are enabled, but the mask files could not be loaded!");
        return;
      }
    }
    //}

    //}
    
    /* subscribe to estimatedFramerate //{ */

    std::vector<std::string> _estimated_framerate_topics;

    param_loader.loadParam("estimatedFramerateTopics", _estimated_framerate_topics, _estimated_framerate_topics);
    // fill the framerates with -1
    estimatedFramerate.insert(estimatedFramerate.begin(), _estimated_framerate_topics.size(), -1);

    if (_estimated_framerate_topics.empty()) {
      ROS_WARN("[UVDARPoseCalculator]: No topics of estimated framerates were supplied");
    }
    if (_blinkers_seen_topics.size() != _estimated_framerate_topics.size()) {
      ROS_ERROR_STREAM("[UVDARPoseCalculator]: The size of blinkers_seen_topics (" << _blinkers_seen_topics.size() <<
          ") is different from estimatedFramerateTopics (" << _estimated_framerate_topics.size() << ")");
    }
    //}

    /* Create callbacks for each camera //{ */
    for (size_t i = 0; i < _estimated_framerate_topics.size(); ++i) {
      estimated_framerate_callback_t callback = [imageIndex=i,this] (const std_msgs::Float32ConstPtr& framerateMessage) { 
        estimatedFramerate[imageIndex] = framerateMessage->data;
      };
      estimatedFramerateCallbacks.push_back(callback);
    }
    //}

    /* Subscribe to corresponding topics //{ */
    for (size_t i = 0; i < _estimated_framerate_topics.size(); ++i) {
      ROS_INFO_STREAM("[UVDARPoseCalculator]: Subscribing to " << _estimated_framerate_topics[i]);
      estimatedFramerateSubscribers.push_back(
          nh_.subscribe(_estimated_framerate_topics[i], 1, &estimated_framerate_callback_t::operator(), &estimatedFramerateCallbacks[i]));
    }

    //}

    if (!_beacon_){
      measuredPose.resize(_target_count_);
      ROS_INFO("[%s]: targetCount: %d", ros::this_node::getName().c_str(), _target_count_ );
      for (int i=0;i<_target_count_;i++){
        ROS_INFO("[%s]: Advertising measuredPose%d", ros::this_node::getName().c_str(), i+1);
        measuredPose[i] = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("measuredPose" + std::to_string(i+1), 1);
      }
    }
    else {
      msg_measurement_array_.resize(_blinkers_seen_topics.size());
      measured_poses_.resize(_blinkers_seen_topics.size());
      ROS_INFO("[%s]: Advertising measuredPoses", ros::this_node::getName().c_str());
      for (int i = 0; i < (int)(_blinkers_seen_topics.size()); i++) {
        measured_poses_[i] = nh_.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("measuredPoses"+std::to_string(i+1), 1);
      }
    }

    X2 = Eigen::VectorXd(9,9);
    X2q = Eigen::VectorXd(8,8);
    X2qb = Eigen::VectorXd(8,8);
    X3 = Eigen::VectorXd(10,10);
    X3qb = Eigen::VectorXd(9,9);
    X4qb = Eigen::VectorXd(12,12);
    Px2 = Eigen::MatrixXd(9,9);
    Px2q = Eigen::MatrixXd(8,8);
    Px2qb = Eigen::MatrixXd(8,8);
    Px3 = Eigen::MatrixXd(10,10);
    Px3qb = Eigen::MatrixXd(9,9);
    Px4qb = Eigen::MatrixXd(12,12);

    foundTarget = false;

    /* transformations for cameras //{ */

    param_loader.loadParam("camera_frames", _camera_frames_, _camera_frames_);
    if (_camera_frames_.size() != _blinkers_seen_topics.size()) {
      ROS_ERROR_STREAM("The size of camera_frames (" << _camera_frames_.size() << 
          ") is different from blinkers_seen_topics size (" << _blinkers_seen_topics.size() << ")");
    }

    if (_gui_) {
      show_thread  = std::thread(&UVDARPoseCalculator::ShowThread, this);
    }

    transformer_      = mrs_lib::Transformer("UVDARPoseCalculator", _uav_name_);

    //}
    
    initialized_ = true;
  }
  //}

    /* Destructor //{ */
  ~UVDARPoseCalculator() {
  }
  //}
  
  /* loadMasks //{ */
  /**
   * @brief Load the mask files - either form absolute path or composite filename found in the mrs_uav_general package.
   *
   * @return success
   */
    bool loadMasks(){
      std::string file_name;
      for (unsigned int i=0; i<_camera_count_; i++){

        file_name = _mask_file_names_[i];

        ROS_INFO_STREAM("[UVDARPoseCalculator Loading mask file [" << file_name << "]");
        if (!(boost::filesystem::exists(file_name))){
          ROS_ERROR_STREAM("[UVDARPoseCalculator Mask [" << file_name << "] does not exist!");
          return false;
        }

        _masks_.push_back(cv::imread(file_name, cv::IMREAD_GRAYSCALE));
        if (!(_masks_.back().data)){
          ROS_ERROR_STREAM("[UVDARPoseCalculator Mask [" << file_name << "] could not be loaded!");
          return false;
        }

      }
      return true;
    }
    //}
    
  /* loadCalibrations //{ */
  /**
   * @brief Load the camera calibration files - either form absolute path or composite filename found in the mrs_uav_general package.
   *
   * @return success
   */
    bool loadCalibrations(){
      std::string file_name;
      _oc_models_.resize(_calib_files_.size());
      int i=0;
      for (auto calib_file : _calib_files_){
        if (calib_file == "default"){
          file_name = ros::package::getPath("uvdar")+"/include/OCamCalib/config/calib_results_bf_uv_fe.txt";
        }
        else {
          file_name = calib_file;
        }

        ROS_INFO_STREAM("[UVDARPoseCalculator]: Loading camera calibration file [" << file_name << "]");
        if (!(boost::filesystem::exists(file_name))){
          ROS_ERROR_STREAM("[UVDARPoseCalculator Calibration file [" << file_name << "] does not exist!");
          return false;
        }

        get_ocam_model(&_oc_models_.at(i), (char*)(file_name.c_str()));
        ROS_INFO_STREAM("[UVDARPoseCalculator]: Calibration parameters for virtual camera " << i << " came from the file " <<  file_name);
        for (int j=0; j<_oc_models_.at(i).length_pol; j++){
          if (isnan(_oc_models_.at(i).pol[j])){
            ROS_ERROR("[UVDARPoseCalculator]: Calibration polynomial containts NaNs! Returning.");
            return false;
          }
        }
        i++;
      }
      return true;
    }
    //}

  /* ProcessPoints //{ */
  void ProcessPoints(const mrs_msgs::ImagePointsWithFloatStampedConstPtr& msg, size_t imageIndex) {
    /* int                        countSeen; */
    std::vector< cv::Point3i > points;
    lastBlinkTime = msg->stamp;
    /* countSeen = (int)((msg)->layout.dim[0].size); */
    if (_debug_)
      ROS_INFO_STREAM("Received points: " << msg->points.size());
    if (msg->points.size() < 1) {
      foundTarget = false;
      return;
    }
    if (estimatedFramerate.size() <= imageIndex || estimatedFramerate[imageIndex] < 0) {
      ROS_INFO_THROTTLE(1.0,"Framerate is not yet estimated. Waiting...");
      return;
    }

    for (auto& point : msg->points) {
      if (_use_masks_){
        if (_masks_[imageIndex].at<unsigned char>(cv::Point2i(point.x, point.y)) < 100){
          if (_debug_){
            ROS_INFO_STREAM("Discarding point " << cv::Point2i(point.x, point.y) << " with f="  << point.value);
          }
          continue;
        }
      }

      if (point.value <= 200) {
        points.push_back(cv::Point3i(point.x, point.y, point.value));
      }
    }

    std::scoped_lock lock(mutex_separated_points);

    if (_beacon_){
      msg_measurement_array_[imageIndex] = boost::make_shared<mrs_msgs::PoseWithCovarianceArrayStamped>();
      msg_measurement_array_[imageIndex]->header.frame_id = _camera_frames_[imageIndex];
      msg_measurement_array_[imageIndex]->header.stamp = lastBlinkTime;
    }

    if ((int)(points.size()) > 0) {
      if (_beacon_){
        separated_points_[imageIndex] = separateByBeacon(points);
        /* return; */
      }
      else {
        separated_points_[imageIndex] = separateByFrequency(points);
      }

      for (int i = 0; i < (_beacon_?((int)(separated_points_[imageIndex].size())):_target_count_); i++) {
        if (_debug_){
          /* if (true){ */
          ROS_INFO_STREAM("target [" << i << "]: ");
          ROS_INFO_STREAM("p: " << separated_points_[imageIndex][i]);
        }
        extractSingleRelative(separated_points_[imageIndex][i], i, imageIndex);
        }
      }
      if (_beacon_){
        measured_poses_[imageIndex].publish(msg_measurement_array_[imageIndex]);
      }
    }
    //}


private:

  /* uvdarHexarotorPose1p //{ */
  unscented::measurement uvdarHexarotorPose1p_meas(Eigen::Vector2d X,double tubewidth, double tubelength, double meanDist, int camera_index){
    double v1[3];
    double x[2] = {X.y(),X.x()};
    cam2world(v1, x, &_oc_models_[camera_index]);
    Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
    V1 = V1*meanDist;

    unscented::measurement ms;
    ms.x = Eigen::VectorXd(6);
    ms.C = Eigen::MatrixXd(6,6);

    ms.x << V1.x(),V1.y(),V1.z(),0,0,0; 
    Eigen::MatrixXd temp;
    temp.setIdentity(6,6);
    ms.C = temp*666;//large covariance for angles in radians
    ms.C.topLeftCorner(3, 3) = calc_position_covariance(V1,tubewidth,tubelength);

    /* std::cout << "ms.C: " << ms.C << std::endl; */


    return ms;

  }
  //}

  /* uvdarHexarotorPose2p //{ */
  Eigen::VectorXd uvdarHexarotorPose2p(Eigen::VectorXd X, Eigen::VectorXd expFrequencies, int camera_index){

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




      /* cv::Point3d central = (a+b) / 2.0; */
      double      v1[3], v2[3];
      double      va[2] = {double(a.y), double(a.x)};
      double      vb[2] = {double(b.y), double(b.x)};
      ;
      cam2world(v1, va, &(_oc_models_[camera_index]));
      cam2world(v2, vb, &(_oc_models_[camera_index]));
      /* double vc[3]; */
      /* double pc[2] = {central.y, central.x}; */
      /* cam2world(vc, pc, &oc_model); */

      Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
      Eigen::Vector3d V2(v2[1], v2[0], -v2[2]);
      /* Eigen::Vector3d Vc(vc[1], vc[0], -vc[2]); */

      /* double alpha = acos(V1.dot(V2)); */

      /* double vd = sqrt(0.75 * _arm_length_); */

      /* double distance = (_arm_length_ / 2.0) / tan(alpha / 2.0) + vd; */
      /* if (first) { */
      /*   distanceSlider.filterInit(distance, filterDistLength); */
      /*   first = false; */
      /* } */
      double d = _arm_length_;
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
      Eigen::Vector3d Vc=Rp*V1;
      Eigen::Vector3d Yt=l*Vc;

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
      Eigen::Transform< double, 3, Eigen::Affine > Ro(Eigen::AngleAxis< double >( Gamma,(Vc.cross(obs_normal)).normalized()));
      obs_normal=Ro*obs_normal;
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

  //}

  /* uvdarHexarotorPose3p //{ */
  Eigen::VectorXd uvdarHexarotorPose3p(Eigen::VectorXd X, Eigen::VectorXd expFrequencies, int camera_index){
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
      if (_debug_)
        ROS_INFO_STREAM("Central led: " << b);
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

      cam2world(v1, va, &(_oc_models_[camera_index]));
      cam2world(v2, vb, &(_oc_models_[camera_index]));
      cam2world(v3, vc, &(_oc_models_[camera_index]));

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
      /* std::cout << "alpha: " << Alpha << " beta: " << Beta << std::endl; */
      /* std::cout << "A: " << A << " B: " << B << std::endl; */

      double O = (A * A - A * B + sqrt(3.0) * A + B * B + sqrt(3.0) * B + 3.0);
      /* std::cout << "long operand: " << O << std::endl; */
      double delta = 2.0 * atan(((B * (2.0 * sqrt(O / (B * B + 2.0 * sqrt(3.0) + 3.0)) - 1.0)) +
                                 (6.0 * sqrt(O / ((sqrt(3.0) * B + 3.0) * (sqrt(3.0) * B + 3.0)))) + (2.0 * A + sqrt(3.0))) /
                                (sqrt(3.0) * B + 3.0));


      /* double gamma      = CV_PI - (delta + Alpha); */

      /* double distMiddle = sin(gamma) * _arm_length_ / sin(Alpha); */
      /* double distMiddle=(_arm_length_*sin(M_PI-(delta+Alpha)))/(sin(Alpha)); */
      double distMiddle=0.5*_arm_length_*((cos(delta)+sin(delta)*A)+(cos((M_PI*(4.0/3.0))-delta)+sin((M_PI*(4.0/3.0))-delta)*B));


      double l = sqrt(fmax(0.1, distMiddle * distMiddle + _arm_length_ * _arm_length_ - 2 * distMiddle * _arm_length_ * cos(delta + (M_PI / 3.0))));

      double Epsilon=asin((_arm_length_/l)*sin(delta+M_PI/3));
      /* phi=asin((b/l)*sin(delta+pi/3)); */

      /* double phi = asin(sin(delta + (CV_PI / 3.0)) * (_arm_length_ / l)); */
      double phi = asin(sin(delta + (CV_PI / 3.0)) * (distMiddle / l));
      /* std::cout << "delta: " << delta << std::endl; */
      /* std::cout << "Estimated distance: " << l << std::endl; */
      /* std_msgs::Float32 dM, fdM; */
      /* dM.data  = distance; */
      /* fdM.data = distanceFiltered; */
      /* measuredDist.publish(dM); */
      /* filteredDist.publish(fdM); */

      /* std::cout << "Estimated angle from mid. LED: " << phi * (180.0 / CV_PI) << std::endl; */

      double C=acos(V2_c.dot(V2));
      Eigen::Vector3d V2_d=V2_c-V2;
      if (V2_d(1)<0)
        C=-C;
      double t=acos(V1.dot(V3));

      double Omega1=asin(fmax(-1.0,fmin(1.0,(C/t)*(2.0*sqrt(3.0)))));

      /* Eigen::Vector3d Pv = V2.cross(V1).normalized(); */
      Eigen::Transform< double, 3, Eigen::Affine > Rc(Eigen::AngleAxis< double >(Epsilon, norm13));
      /* vc=Rc(1:3,1:3)*v2_c; */
      Eigen::Vector3d Vc = Rc*V2_c;

      Eigen::VectorXd Yt=l*Vc;



      /* std::cout << "Estimated center in CAM: " << Yt << std::endl; */

      double tilt_perp=Omega1+atan2(Vc(1),sqrt(sqr(Vc(2))+sqr(Vc(0))));

      double relyaw;

      if (_debug_)
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

      Eigen::Transform< double, 3, Eigen::Affine > Re(Eigen::AngleAxis< double >( Gamma+(M_PI/2),(Vc.cross(latComp)).normalized()));
      Eigen::Vector3d exp_normal=Re*Vc;
      Eigen::Transform< double, 3, Eigen::Affine > Ro(Eigen::AngleAxis< double >( Gamma,(Vc.cross(obs_normal)).normalized()));
      obs_normal=Ro*obs_normal;
      /* obs_normal=Ro(1:3,1:3)*obs_normal; */
      /* exp_normal=Re(1:3,1:3)*vc; */
      double tilt_par=acos(obs_normal.dot(exp_normal));
      if (V1(1)<V2(1))
        tilt_par=-tilt_par;


      if (_debug_){
        ROS_INFO_STREAM("latComp: " << latComp);
        ROS_INFO_STREAM("Vc: " << Vc);
        ROS_INFO_STREAM("Gamma: " << Gamma);
        ROS_INFO_STREAM("exp_normal: " << exp_normal);
        ROS_INFO_STREAM("obs_normal: " << obs_normal);
        ROS_INFO_STREAM("tilt_par: " << tilt_par);
        ROS_INFO_STREAM("tilt_perp: " << tilt_perp);
      }

      double relyaw_view=relyaw+phi;
      relyaw=relyaw-atan2(Vc(0),Vc(2))+phi;

      double xl=(_arm_length_/2)*cos(phi);
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
      if (_debug_){
        ROS_INFO_STREAM("tpitch: " << tpitch << " ta: " << ta << " tc: " << tc);
        ROS_INFO_STREAM("troll: " << tpitch << " tb: " << ta << " tc: " << tc);
        ROS_INFO_STREAM("reltilt_abs: " << reltilt_abs << " tilt_perp: " << tilt_perp << " tilt_par: " << tilt_par);
        ROS_INFO_STREAM("Omega1: " << Omega1 << " t: " << t << " C: " << C);
      }

      Y << Yt.x(),Yt.y(),Yt.z(),troll,tpitch,relyaw;
      return Y;
  }
  //}

  /* uvdarQuadrotorPose1p //{ */
  unscented::measurement uvdarQuadrotorPose1p_meas(Eigen::Vector2d X,double tubewidth, double tubelength, double meanDist, int camera_index){
    double v1[3];
    double x[2] = {X.y(),X.x()};
    cam2world(v1, x, &(_oc_models_[camera_index]));
    Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
    V1 = V1*meanDist;

    unscented::measurement ms;
    ms.x = Eigen::VectorXd(6);
    ms.C = Eigen::MatrixXd(6,6);

    ms.x << V1.x(),V1.y(),V1.z(),0,0,0; 
    Eigen::MatrixXd temp;
    temp.setIdentity(6,6);
    ms.C = temp*666;//large covariance for angles in radians
    ms.C.topLeftCorner(3, 3) = calc_position_covariance(V1,tubewidth,tubelength);

    /* std::cout << "ms.C: " << ms.C << std::endl; */


    return ms;
  }
    //}

  /* uvdarQuadrotorPose2p //{ */
  Eigen::VectorXd uvdarQuadrotorPose2p(Eigen::VectorXd X, Eigen::VectorXd expFrequencies, int camera_index){
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




      /* cv::Point3d central = (a+b) / 2.0; */
      double      v1[3], v2[3];
      double      va[2] = {double(a.y), double(a.x)};
      double      vb[2] = {double(b.y), double(b.x)};
      
      if (_debug_){
        ROS_INFO_STREAM("[UVDARPoseCalculator]: a: " << a << " b: " << b);
      }
      cam2world(v1, va, &(_oc_models_[camera_index]));
      cam2world(v2, vb, &(_oc_models_[camera_index]));
      /* double vc[3]; */
      /* double pc[2] = {central.y, central.x}; */
      /* cam2world(vc, pc, &oc_model); */

      Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
      Eigen::Vector3d V2(v2[1], v2[0], -v2[2]);
      if (_debug_){
        ROS_INFO_STREAM("[UVDARPoseCalculator]: V1: " << V1 << " V2: " << V2);
      }
      /* Eigen::Vector3d Vc(vc[1], vc[0], -vc[2]); */

      /* double alpha = acos(V1.dot(V2)); */

      /* double vd = sqrt(0.75 * _arm_length_); */

      /* double distance = (_arm_length_ / 2.0) / tan(alpha / 2.0) + vd; */
      /* if (first) { */
      /*   distanceSlider.filterInit(distance, filterDistLength); */
      /*   first = false; */
      /* } */
      double d = _arm_length_;
      double v=d*sqrt(1.0/2.0);
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
        (2*v*Alpha - 
         v*csAlpha -
         v*cos(Alpha - 2*delta) -
         3*v*snAlpha -
         v*sin(Alpha-2*delta) +
         2*v*Alpha*sn2delta -
         sqrt(2)*sqrt(
           sqv*
           (1-2*csdelta*sndelta)*
           (1-2*Alpha2-2*Alpha*csAlpha+2*Alpha*cos(Alpha-2*delta)+cos(2*(Alpha-delta))+2*Alpha*snAlpha+2*Alpha*sin(Alpha-2*delta)-2*Alpha2*sn2delta)))
        /
        (2*(csdelta*(Alpha-2*snAlpha)+Alpha*sndelta));

      /* ROS_INFO("sqrt element is %f", sqrt( */
      /*      sqv* */
      /*      (1-2*csdelta*sndelta)* */
      /*      (1-2*Alpha2-2*Alpha*csAlpha+2*Alpha*cos(Alpha-2*delta)+cos(2*(Alpha-delta))+2*Alpha*snAlpha+2*Alpha*sin(Alpha-2*delta)-2*Alpha2*sn2delta)) */
      /*   ); */

      if (_debug_){
        /* ROS_INFO("long element is %f", 1-2*Alpha2-2*Alpha*csAlpha+2*Alpha*cos(Alpha-2*delta)+cos(2*(Alpha-delta))+2*Alpha*snAlpha+2*Alpha*sin(Alpha-2*delta)-2*Alpha2*sn2delta); */

        ROS_INFO("Alpha = %f, delta = %f, v = %f", Alpha, delta, v);
        /* ROS_INFO_STREAM("csAlpha = " << csAlpha << " V1 = " << V1 << " V2 = " << V2); */
        /* ROS_INFO_STREAM("a = " << a << " b = " << b ); */

        /* distanceSlider.filterPush(distance); */

        /* std::cout << "Estimated distance: " << l << std::endl; */
        /* std::cout << "Filtered distance: " << distanceFiltered << std::endl; */

        /* std::cout << "Estimated direction in CAM: " << (Rp*V2) << std::endl; */
        /* std::cout << "Central LED direction in CAM: " << (V2) << std::endl; */
        /* std::cout << "Rotation: " << Rp.matrix()   << std::endl; */
      }

      /* foundTarget = true; */
      /* lastSeen    = ros::Time::now(); */

      /* std_msgs::Float32 dM, fdM; */
      /* dM.data  = distance; */
      /* fdM.data = distanceFiltered; */
      /* measuredDist.publish(dM); */
      /* filteredDist.publish(fdM); */



      double kappa = M_PI/2-delta;
      double d1=(v)+v*tan(delta);
      double xl=v/cos(delta);
      double yl=l-xl;
      double alpha1=atan((d1*sin(kappa))/(yl-d1*cos(kappa)));
      Eigen::Vector3d Pv = V2.cross(V1).normalized();
      Eigen::Transform< double, 3, Eigen::Affine > Rp(Eigen::AngleAxis< double >(-alpha1, Pv));
      Eigen::Vector3d Vc=Rp*V1;
      Eigen::Vector3d Yt=l*Vc;

      double relyaw = delta;

      if (expFrequencies.size() == 2){
        if (fabs(expFrequencies[1] - expFrequencies[0]) > 1.0){
          if ((id(0)==ids[0]) && (id(1)==ids[0]))
            relyaw=(M_PI/2)+delta;
          else if ((id(0)==ids[1]) && (id(1)==ids[1]))
            relyaw=(-M_PI/2)+delta;
          else if ((id(0)==ids[0]) && (id(1)==ids[1]))
            relyaw=0+delta;
          else
            relyaw=M_PI+delta;
        }
      }

        
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
      double tilt_perp=X(7);
      Eigen::Vector3d obs_normal=V2.cross(V1);
      obs_normal=obs_normal/(obs_normal.norm());
      Eigen::Vector3d latComp;
      latComp << Vc(0),0,Vc(2);
      latComp = latComp/(latComp.norm());

      if (Vc(1)<0) latComp = -latComp;
      /* ROS_INFO_STREAM("cross: " << Vc.cross(latComp)); */

      Eigen::Transform< double, 3, Eigen::Affine > Re(Eigen::AngleAxis< double >( Gamma+(M_PI/2),(Vc.cross(latComp)).normalized()));
      Eigen::Vector3d exp_normal=Re*Vc;
      Eigen::Transform< double, 3, Eigen::Affine > Ro(Eigen::AngleAxis< double >( Gamma,(Vc.cross(obs_normal)).normalized()));
      obs_normal=Ro*obs_normal;
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

       
    return Eigen::VectorXd();
  }
    //}

  /* uvdarQuadrotorPose3p //{ */
  Eigen::VectorXd uvdarQuadrotorPose3p(Eigen::VectorXd X, Eigen::VectorXd expFrequencies, int camera_index){
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
      if (_debug_)
        ROS_INFO_STREAM("Central led: " << b);
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

      cam2world(v1, va, &(_oc_models_[camera_index]));
      cam2world(v2, vb, &(_oc_models_[camera_index]));
      cam2world(v3, vc, &(_oc_models_[camera_index]));

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
      if (_debug_){
        std::cout << "a: " << a << " b: " << b << " c: " << c << std::endl;
        std::cout << "alpha: " << Alpha << " beta: " << Beta << std::endl;
        std::cout << "A: " << A << " B: " << B << std::endl;
      }

      /* double O = sqrt(2 + 2*A + A*A + 2*B + B*B); */
      /* std::cout << "long operand: " << O << std::endl; */
      /* double delta = atan(-((1+A)/(O))/(((1+A+B+A*B)/O)/(1+A))); */
      double delta = atan(-(1.0+B)/(1+A))+(M_PI);


      /* double gamma      = CV_PI - (delta + Alpha); */

      /* double distMiddle = sin(gamma) * _arm_length_ / sin(Alpha); */
      /* double distMiddle=(_arm_length_*sin(M_PI-(delta+Alpha)))/(sin(Alpha)); */
      double v = sqrt(0.5)*_arm_length_;
      double distMiddle=v*((cos(delta)+sin(delta)*A)+(cos((M_PI*1.5)-delta)+sin((M_PI*1.5)-delta)*B));


      double l = sqrt(fmax(0.1, distMiddle * distMiddle + _arm_length_ * _arm_length_ - 2 * distMiddle * _arm_length_ * cos(delta + (M_PI / 4.0))));

      double Epsilon=asin((_arm_length_/l)*sin(delta+M_PI/4.0));
      /* phi=asin((b/l)*sin(delta+pi/3)); */

      /* double phi = asin(sin(delta + (CV_PI / 3.0)) * (_arm_length_ / l)); */
      double phi = asin(sin(delta + (CV_PI/4.0)) * (distMiddle / l));
      if (_debug_){
        std::cout << "delta: " << delta << std::endl;
        std::cout << "distMiddle: " << distMiddle << std::endl;
        std::cout << "Estimated distance: " << l << std::endl;
      }
      /* std_msgs::Float32 dM, fdM; */
      /* dM.data  = distance; */
      /* fdM.data = distanceFiltered; */
      /* measuredDist.publish(dM); */
      /* filteredDist.publish(fdM); */

      /* std::cout << "Estimated angle from mid. LED: " << phi * (180.0 / CV_PI) << std::endl; */

      double C=acos(V2_c.dot(V2));
      Eigen::Vector3d V2_d=V2_c-V2;
      if (V2_d(1)<0)
        C=-C;
      double t=acos(V1.dot(V3));

      double Omega1=asin(fmax(-1.0,fmin(1.0,(C/t)*(2.0*sqrt(3.0)))));

      /* Eigen::Vector3d Pv = V2.cross(V1).normalized(); */
      Eigen::Transform< double, 3, Eigen::Affine > Rc(Eigen::AngleAxis< double >(Epsilon, norm13));
      /* vc=Rc(1:3,1:3)*v2_c; */
      Eigen::Vector3d Vc = Rc*V2_c;

      Eigen::VectorXd Yt=l*Vc;

      double tilt_perp=Omega1+atan2(Vc(1),sqrt(sqr(Vc(2))+sqr(Vc(0))));

      double relyaw = 2*ambig;;

      if (_debug_)
        ROS_INFO_STREAM("leds: " << id);

      if (expFrequencies.size() == 2){
        if (fabs(expFrequencies[1] - expFrequencies[0]) > 1.0){
          if ((id(0)==ids[0]) && (id(1)==ids[0]) && (id(2)==ids[1]))
            relyaw=(M_PI/4);
          else if ((id(0)==ids[0]) && (id(1)==ids[1]) && (id(2)==ids[1]))
            relyaw=(-M_PI/4);
          else if ((id(0)==ids[1]) && (id(1)==ids[1]) && (id(2)==ids[0]))
            relyaw=(-3*M_PI/4);
          else if ((id(0)==ids[1]) && (id(1)==ids[0]) && (id(2)==ids[0]))
            relyaw=(3*M_PI/4);
          else
            if     ((id(0)==ids[0]) && (id(1)==ids[0]) && (id(2)==ids[0]))
              relyaw=(M_PI/2)+ambig;
            else if ((id(0)==ids[1]) && (id(1)==ids[1]) && (id(2)==ids[1]))
              relyaw=(-M_PI/2)+ambig;
            else
              relyaw=2*ambig;
        }
      }

      double latnorm=sqrt(sqr(Yt(0))+sqr(Yt(2)));
      double Gamma=atan2(Yt(1),latnorm);
      Eigen::Vector3d obs_normal =V3.cross(V1); //not exact, but in practice changes very little
      obs_normal=obs_normal/(obs_normal.norm());
      Eigen::Vector3d latComp;
      latComp << Vc(0),0,Vc(2);
      latComp = latComp/(latComp.norm());
      if (Vc(1)<0) latComp = -latComp;

      Eigen::Transform< double, 3, Eigen::Affine > Re(Eigen::AngleAxis< double >( Gamma+(M_PI/2),(Vc.cross(latComp)).normalized()));
      Eigen::Vector3d exp_normal=Re*Vc;
      Eigen::Transform< double, 3, Eigen::Affine > Ro(Eigen::AngleAxis< double >( Gamma,(Vc.cross(obs_normal)).normalized()));
      obs_normal=Ro*obs_normal;
      /* obs_normal=Ro(1:3,1:3)*obs_normal; */
      /* exp_normal=Re(1:3,1:3)*vc; */
      double tilt_par=acos(obs_normal.dot(exp_normal));
      if (V1(1)<V2(1))
        tilt_par=-tilt_par;


      if (_debug_){
        ROS_INFO_STREAM("latComp: " << latComp);
        ROS_INFO_STREAM("Vc: " << Vc);
        ROS_INFO_STREAM("Gamma: " << Gamma);
        ROS_INFO_STREAM("exp_normal: " << exp_normal);
        ROS_INFO_STREAM("obs_normal: " << obs_normal);
        ROS_INFO_STREAM("tilt_par: " << tilt_par);
        ROS_INFO_STREAM("tilt_perp: " << tilt_perp);
      }

      double relyaw_view=relyaw+phi;
      relyaw=relyaw-atan2(Vc(0),Vc(2))+phi;

      double xl=(_arm_length_/2)*cos(phi);
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
      if (_debug_){
        ROS_INFO_STREAM("tpitch: " << tpitch << " ta: " << ta << " tc: " << tc);
        ROS_INFO_STREAM("troll: " << tpitch << " tb: " << ta << " tc: " << tc);
        ROS_INFO_STREAM("reltilt_abs: " << reltilt_abs << " tilt_perp: " << tilt_perp << " tilt_par: " << tilt_par);
        ROS_INFO_STREAM("Omega1: " << Omega1 << " t: " << t << " C: " << C);
      }

      Y << Yt.x(),Yt.y(),Yt.z(),troll,tpitch,relyaw;
      return Y;
  }
    //}

  /* uvdarQuadrotorPose2pB //{ */
  Eigen::VectorXd uvdarQuadrotorPose2pB(Eigen::VectorXd X, Eigen::VectorXd expFrequencies, int camera_index){
    if (_debug_){
      ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: input: " << X);
    }
      cv::Point3d a;
      cv::Point3d b;

      a = cv::Point3d(X(0),X(1),0);
      b = cv::Point3d(X(2),X(3),X(4));
      double tilt_par=X(5); //tilt about the line of sight
      double tilt_perp=X(6);//basically pitch wrt the line of sight
      tilt_perp = std::fmin(std::fmax(tilt_perp,-(M_PI_2/2.0)),(M_PI_2/2.0));

      double ambig=X(7);

      /* std::cout << "right led: " << b << std::endl; */
      Eigen::Vector3i ids;
      ids << 0,1,2;
      Eigen::Vector3d expPeriods;
      expPeriods = expFrequencies.cwiseInverse(); 
      Eigen::Vector3d periods;
      periods << b.z;
      Eigen::Vector3d id;
      Eigen::MatrixXd::Index   minIndex;
      ((expPeriods.array()-(periods(0))).cwiseAbs()).minCoeff(&minIndex);
      id(0) = minIndex;

      double      v1[3], v2[3];
      double      va[2] = {double(a.y), double(a.x)};
      double      vb[2] = {double(b.y), double(b.x)};
      
      cam2world(v1, va, &(_oc_models_[camera_index]));
      cam2world(v2, vb, &(_oc_models_[camera_index]));

      Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
      Eigen::Vector3d V2(v2[1], v2[0], -v2[2]);
      Eigen::Vector3d DW(0, 1, 0); //downwards

      double p = acos(V1.dot(V2));
      double rho = acos((V2-V1).normalized().dot(DW));
      double xi = rho - tilt_par;
      double q = p*sin(xi);

      Eigen::Transform<double, 3, Eigen::Affine > Rt(Eigen::AngleAxis< double >( tilt_par,V1.normalized()));
      e::Vector3d DWt = Rt*DW;
      if (acos(((V2-V1).normalized().cross(DWt)).dot(V1.normalized())) < M_PI_2)
        q = -q;

      double c = p*cos(xi); //p, c, s, q are angles - approaching orthogonal projection at distance
      double s = -sin(tilt_perp)*c*(_arm_length_/_beacon_height_);
      double yaw = -asin(std::fmin(std::fmax((q*cos(tilt_perp)*_beacon_height_) / (_arm_length_*(c-s)), -1),1))+(M_PI/4);
      if (_debug_){
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: (q*cos(tilt_perp)*_beacon_height_): " << (q*cos(tilt_perp)*_beacon_height_));
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: (_arm_length_*(c-s)): " << (_arm_length_*(c-s)));
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: ((q*cos(tilt_perp)*_beacon_height_) / (_arm_length_*(c-s))): " << (q*cos(tilt_perp)*_beacon_height_) / (_arm_length_*(c-s)));
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: yaw origin: " << yaw);
      }

      double l = ((_beacon_height_*cos(tilt_perp)) / tan(c-s) );

      if (_debug_){
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: c: " << c);
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: p: " << p);
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: rho: " << rho);
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: xi: " << xi);
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: q: " << q);
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: s: " << s);
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: tilt_par: " << tilt_par);
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: tilt_perp: " << tilt_perp);
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: l: " << l);
      }

      Eigen::Transform< double, 3, Eigen::Affine > Rc(Eigen::AngleAxis< double >( c-s,(V1.cross(DW)).normalized()));
      e::Vector3d VC = Rc*V1; 

      e::VectorXd output(6,6); 

      output.topRows(3) = VC*l;

      double latang=atan2(VC(0),VC(2));


      double relyaw = 4*ambig;
        /* ROS_INFO_STREAM("[0]"); */
      if (expFrequencies.size() == 2){
        /* ROS_INFO_STREAM("[A]"); */
        if (fabs(expFrequencies[0] - expFrequencies[0]) > 1.0)
        {
        /* ROS_INFO_STREAM("[B]"); */
          if (id(0)==ids[0])
            relyaw= (M_PI_2) + ambig;
          else
            relyaw= (-M_PI_2) + ambig;
        }
      }
      yaw += relyaw;

      double yaw_view=yaw;
      yaw=yaw-latang;

      double reltilt_abs=atan(sqrt(sqr(tan(tilt_perp))+sqr(tan(tilt_par))));
      double tiltdir=atan2(-tan(tilt_par),tan(tilt_perp));
      double tiltdir_adj=tiltdir-yaw_view;
      double ta=cos(tiltdir_adj);
      double tb=-sin(tiltdir_adj);
      double tc=cot(reltilt_abs);
      double tpitch=atan2(ta,tc);
      double troll=atan2(tb,tc);

      output(3) = troll;
      output(4) = tpitch;
      output(5) = yaw;
      if (_debug_){
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: troll: " << troll);
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: tpitch: " << tpitch);
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: yaw: " << yaw);
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: relyaw: " << relyaw);
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: yaw_view: " << yaw_view);
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: latang: " << latang);
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: VC: " << VC);
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: 2p: output: " << output);
      }
    /* ROS_WARN("[%s]: One point beacon measurement Not Implemented yet", ros::this_node::getName().c_str()); */
    return output;
  }
    //}
    
  /* uvdarQuadrotorPose3pB //{ */
  Eigen::VectorXd uvdarQuadrotorPose3pB(Eigen::VectorXd X, Eigen::VectorXd expFrequencies, int camera_index){
      cv::Point3d tmp;
      cv::Point3d a(X(0),X(1),0);
      cv::Point3d b;
      cv::Point3d c;
      double ambig = X(8);

      if ((X(2)) < (X(5))) {
        b = cv::Point3d(X(2),X(3),X(4));
        c = cv::Point3d(X(5),X(6),X(7));
      } else {
        b = cv::Point3d(X(5),X(6),X(7));
        c = cv::Point3d(X(2),X(3),X(4));
      }

      if (_debug_)
        ROS_INFO_STREAM("left led: " << b);
      Eigen::Vector3i ids;
      ids << 0,1,2;
      Eigen::Vector3d expPeriods;
      expPeriods = expFrequencies.cwiseInverse(); 
      Eigen::Vector3d periods;
      periods << b.z,c.z;
      Eigen::Vector3d id;
      Eigen::MatrixXd::Index   minIndex;
      ((expPeriods.array()-(periods(0))).cwiseAbs()).minCoeff(&minIndex);
      id(0) = minIndex;
      ((expPeriods.array()-(periods(1))).cwiseAbs()).minCoeff(&minIndex);
      id(1) = minIndex;


      double v1[3], v2[3], v3[3];
      double va[2] = {(double)(a.y), (double)(a.x)};
      double vb[2] = {(double)(b.y), (double)(b.x)};
      double vc[2] = {(double)(c.y), (double)(c.x)};

      cam2world(v1, va, &(_oc_models_[camera_index]));
      cam2world(v2, vb, &(_oc_models_[camera_index]));
      cam2world(v3, vc, &(_oc_models_[camera_index]));

      Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
      Eigen::Vector3d V2(v2[1], v2[0], -v2[2]);
      Eigen::Vector3d V3(v3[1], v3[0], -v3[2]);

      std::vector<e::Vector3d> direction_vectors = {V1, V2, V3};

      std::vector<e::Vector3d> object_points;
      double armCoord = _arm_length_*sqrt(0.5);
      object_points.push_back(e::Vector3d(0,0,_beacon_height_));
      object_points.push_back(e::Vector3d(-armCoord,armCoord,0));
      object_points.push_back(e::Vector3d(-armCoord,-armCoord,0));

      std::vector<e::Vector3d> cam_centers;
      std::vector<e::Matrix3d> cam_rotations;

      if (_debug_){
        for (int it = 0; it < (int)(direction_vectors.size()); it++) {
          ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() << "]: direction_vectors: " << direction_vectors[it].transpose());
          ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() << "]: object_points: " << object_points[it].transpose());
        }
      }

      Eigen::Matrix3d camToBase;
      camToBase <<
        0,  0,  1,
       -1,  0,  0,
        0, -1,  0;
      p3p_kneip::P3PComputePoses(
          direction_vectors,
          object_points,
          &cam_rotations,
          &cam_centers);


      e::VectorXd output_candidate(6,6);
      std::vector<e::VectorXd> output_candidates;
      e::VectorXd output(6,6);
      if (cam_centers.size() == 0){
        if (_debug_){
          ROS_INFO("[%s]: No solution found.", ros::this_node::getName().c_str());
        }
      }
      else {
        for (int i = 0; i < (int)(cam_centers.size()); i++) {
          e::Matrix3d object_orientation_cam = cam_rotations[i];
          e::Vector3d object_position_cam = -cam_rotations[i]*cam_centers[i];
          e::Vector3d object_position = camToBase*object_position_cam;
          double roll   = rotmatToRoll(camToBase*object_orientation_cam);
          double pitch  = rotmatToPitch(camToBase*object_orientation_cam);
          double yaw    = rotmatToYaw(camToBase*object_orientation_cam);
          if (_debug_){
            ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() << "]: cam_center: " << (object_position).transpose());
            ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() << "]: cam_rotations: " << roll << " : " << pitch << " : " << yaw);
          }
         if ( ( fabs(roll) > (deg2rad(50)) ) || (fabs(pitch) > (deg2rad(50)) ) 
               ||
              ( acos(object_position_cam.normalized().dot(V1)) > deg2rad(10) ) 
            )
           continue;
         output_candidate <<
           object_position_cam.x(), object_position_cam.y(), object_position_cam.z(), 
           roll, pitch, yaw;
         output_candidates.push_back(output_candidate);
         if (_debug_){
           ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() << "]: output: " << output_candidate.transpose());
         }
        }
      }

      /* output_candidate << 0,0,0,0,0,0; */
      double dist_max = 0;
      for (auto &candidate : output_candidates) {
        if (candidate.topRows(3).norm() > dist_max){
          dist_max = candidate.topRows(3).norm();
          output = candidate;
        }
      }
      /* output_candidate = output_candidate / (int)(output_candidates.size()); // this should be done with unscentedTransform, but I'll do this for now. */

      double relyaw = ambig;

      if (expFrequencies.size() == 2){
        if (fabs(expFrequencies[1] - expFrequencies[0]) > 1.0)
        {
          if ((id(0)==ids[0]) && (id(1)==ids[0]))
            relyaw=(M_PI/2);
          else if ((id(0)==ids[1]) && (id(1)==ids[1]))
            relyaw=(-M_PI/2);
          else if ((id(0)==ids[0]) && (id(1)==ids[1]))
            relyaw=0;
          else
            relyaw=M_PI;
        }
      }

      
      output[5] += relyaw;

      if (_debug_){
        ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() << "]: output final: " << output.transpose());
      }


    /* ROS_WARN("[%s]: Two point beacon measurement Implemented yet", ros::this_node::getName().c_str()); */
    return output;
  }
    //}
    
  /* uvdarQuadrotorPose4pB //{ */
  Eigen::VectorXd uvdarQuadrotorPose4pB(Eigen::VectorXd X, Eigen::VectorXd expFrequencies, int camera_index){
      cv::Point3d tmp;
      cv::Point3d a(X(0),X(1),0);
      cv::Point3d b(X(2),X(3),X(4));
      cv::Point3d c(X(5),X(6),X(7));
      cv::Point3d d(X(8),X(9),X(10));
      double ambig = X(11);

      if ((d.x) < (b.x)) {
        tmp = b;
        b = d;
        d = tmp;
      }
      if ((c.x) < (b.x)) {
        tmp = c;
        c = b;
        b = tmp;
      }
      if ((c.x) > (d.x)) {
        tmp = d;
        d   = c;
        c   = tmp;
      }
      if (_debug_)
        ROS_INFO_STREAM("Central led: " << c);
      Eigen::Vector3i ids;
      ids << 0,1,2;
      Eigen::Vector3d expPeriods;
      expPeriods = expFrequencies.cwiseInverse(); 
      Eigen::Vector3d periods;
      periods << b.z,c.z,d.z;
      Eigen::Vector3d id;
      Eigen::MatrixXd::Index   minIndex;
      ((expPeriods.array()-(periods(0))).cwiseAbs()).minCoeff(&minIndex);
      id(0) = minIndex;
      ((expPeriods.array()-(periods(1))).cwiseAbs()).minCoeff(&minIndex);
      id(1) = minIndex;
      ((expPeriods.array()-(periods(2))).cwiseAbs()).minCoeff(&minIndex);
      id(2) = minIndex;


      double v1[3], v2[3], v3[3], v4[4];
      double va[2] = {(double)(a.y), (double)(a.x)};
      double vb[2] = {(double)(b.y), (double)(b.x)};
      double vc[2] = {(double)(c.y), (double)(c.x)};
      double vd[2] = {(double)(d.y), (double)(d.x)};

      cam2world(v1, va, &(_oc_models_[camera_index]));
      cam2world(v2, vb, &(_oc_models_[camera_index]));
      cam2world(v3, vc, &(_oc_models_[camera_index]));
      cam2world(v4, vd, &(_oc_models_[camera_index]));

      Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
      Eigen::Vector3d V2(v2[1], v2[0], -v2[2]);
      Eigen::Vector3d V3(v3[1], v3[0], -v3[2]);
      Eigen::Vector3d V4(v4[1], v4[0], -v4[2]);

      Eigen::Matrix3d camToBase;
      camToBase <<
        0,  0,  1,
       -1,  0,  0,
        0, -1,  0;

      std::vector<e::Vector3d> direction_vectors = {camToBase*V1, camToBase*V2, camToBase*V4};

      std::vector<e::Vector3d> object_points;
      double armCoord = _arm_length_*sqrt(0.5);
      object_points.push_back(e::Vector3d(0,0,_beacon_height_));
      object_points.push_back(e::Vector3d(armCoord,armCoord,0));
      object_points.push_back(e::Vector3d(-armCoord,-armCoord,0)); //default is rotated by pi/4 to the RIGHT

      std::vector<e::Vector3d> cam_centers;
      std::vector<e::Matrix3d> cam_rotations;

      if (_debug_){
        for (int it = 0; it < (int)(direction_vectors.size()); it++) {
          ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() << "]: direction_vectors: " << direction_vectors[it].transpose());
          ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() << "]: object_points: " << object_points[it].transpose());
        }
      }

      p3p_kneip::P3PComputePoses(
          direction_vectors,
          object_points,
          &cam_rotations,
          &cam_centers);


      e::VectorXd output_candidate(6,6);
      std::vector<e::VectorXd> output_candidates;
      e::VectorXd output(6,6);
      if (cam_centers.size() == 0){
        if (_debug_){
          ROS_INFO("[%s]: No solution found.", ros::this_node::getName().c_str());
        }
      }
      else {
        for (int i = 0; i < (int)(cam_centers.size()); i++) {
          e::Matrix3d object_orientation = cam_rotations[i];
          /* e::Matrix3d object_orientation_cam = camToBase.transpose()*cam_rotations[i]; */
          e::Vector3d object_position= -cam_rotations[i]*cam_centers[i];
          e::Vector3d object_position_cam  = camToBase.transpose()*object_position;
          /* ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() << "]: 4p:  rotation_matrix: "); */
          /* ROS_INFO_STREAM(std::endl << object_orientation_cam); */
          /* ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() << "]: 4p:  rotation_matrix_original: "); */
          /* ROS_INFO_STREAM(std::endl << object_orientation); */
          double roll   = rotmatToRoll(object_orientation);
          double pitch  = rotmatToPitch(object_orientation);
          double yaw    = rotmatToYaw(object_orientation);

          if  (_debug_){
            ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() << "]: 4p:  camera_center: " << (cam_centers[i]).transpose());
            ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() << "]: 4p:  object_center: " << (object_position).transpose());
            ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() << "]: 4p:  object_rotations: " << roll << " : " << pitch << " : " << yaw);
          }
          double yaw_view_offset = yaw+atan2(object_position_cam(0),object_position_cam(2));
          yaw_view_offset -= deg2rad(45);
          if (_debug_){
            ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() << "]: 4p:  view_relative_yaw_zero: " << atan2(object_position_cam(0),object_position_cam(2)));
            ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() << "]: 4p:  view_relative_yaw: " << rad2deg(yaw_view_offset));
          }
         if ( ( fabs(roll) > (deg2rad(50)) ) || (fabs(pitch) > (deg2rad(50)) )  
               ||
              ( acos(object_position_cam.normalized().dot(V1)) > deg2rad(10) ) 
              ||
              (fabs(yaw_view_offset) > (deg2rad(60)))
            )
           continue;
         output_candidate <<
           object_position_cam.x(), object_position_cam.y(), object_position_cam.z(), 
           roll, pitch, yaw;
         output_candidates.push_back(output_candidate);
         if (_debug_){
           ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() << "]: 4p:  output: " << output_candidate.transpose());
         }
        }
      }

      /* output_candidate << 0,0,0,0,0,0; */
      double dist_max = 0;
      for (auto &candidate : output_candidates) {
        if (candidate.topRows(3).norm() > dist_max){
          dist_max = candidate.topRows(3).norm();
          output = candidate;
        }
      }
      /* output_candidate = output_candidate / (int)(output_candidates.size()); // this should be done with unscentedTransform, but I'll do this for now. */

      double relyaw = ambig;
      /* ROS_INFO_STREAM("relyaw: " << relyaw); */

      if (_debug_)
        ROS_INFO_STREAM("leds: " << id);

      if (expFrequencies.size() == 2){
        if (fabs(expFrequencies[1] - expFrequencies[0]) > 1.0){
          if ((id(0)==ids[0]) && (id(1)==ids[0]) && (id(2)==ids[1]))
            relyaw=(M_PI/4);
          else if ((id(0)==ids[0]) && (id(1)==ids[1]) && (id(2)==ids[1]))
            relyaw=(-M_PI/4);
          else if ((id(0)==ids[1]) && (id(1)==ids[1]) && (id(2)==ids[0]))
            relyaw=(-3*M_PI/4);
          else if ((id(0)==ids[1]) && (id(1)==ids[0]) && (id(2)==ids[0]))
            relyaw=(3*M_PI/4);
          else
            if     ((id(0)==ids[0]) && (id(1)==ids[0]) && (id(2)==ids[0]))
              relyaw=(M_PI/2)+ambig;
            else if ((id(0)==ids[1]) && (id(1)==ids[1]) && (id(2)==ids[1]))
              relyaw=(-M_PI/2)+ambig;
            else
              relyaw=2*ambig;
        }
      }

      /* ROS_INFO_STREAM("relyaw: " << relyaw); */
      /* ROS_INFO_STREAM("ambig: " << ambig); */
      /* ROS_INFO_STREAM("expFr: " << expFrequencies); */
      
      output[5] += relyaw;

      if (_debug_){
        ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() << "]: 4p:  output final: " << output.transpose());
      }


    /* ROS_WARN("[%s]: Three point beacon measusrement Not Implemented yet", ros::this_node::getName().c_str()); */
    return output;
  }
    //}

  /* rotate_covariance //{ */
  static Eigen::Matrix3d rotate_covariance(const Eigen::Matrix3d& covariance, const Eigen::Matrix3d& rotation) {
    return rotation * covariance * rotation.transpose();  // rotate the covariance to point in direction of est. position
  }
  //}

  /* calc_position_covariance //{ */
  //Courtesy of Matous Vrba
  static Eigen::Matrix3d calc_position_covariance(const Eigen::Vector3d& position_sf, const double xy_covariance_coeff, const double z_covariance_coeff) {
    Eigen::Matrix3d v_x;
    Eigen::Vector3d b;
    Eigen::Vector3d v;
    double sin_ab, cos_ab;
    Eigen::Matrix3d vec_rot;
    const Eigen::Vector3d a(0.0, 0.0, 1.0);
    /* Calculates the corresponding covariance matrix of the estimated 3D position */
    Eigen::Matrix3d pos_cov = Eigen::Matrix3d::Identity();  // prepare the covariance matrix
    /* const double tol = 1e-9; */
    pos_cov(0, 0) = pos_cov(1, 1) = xy_covariance_coeff*xy_covariance_coeff;

    /* double dist = position_sf.norm(); */
    /* pos_cov(2, 2) = dist * sqrt(dist) * z_covariance_coeff; */
    /* if (pos_cov(2, 2) < 0.33 * z_covariance_coeff) */
    /*   pos_cov(2, 2) = 0.33 * z_covariance_coeff; */
    pos_cov(2, 2) = z_covariance_coeff;

    // Find the rotation matrix to rotate the covariance to point in the direction of the estimated position
    b = position_sf.normalized();
    v = a.cross(b);
    sin_ab = v.norm();
    cos_ab = a.dot(b);
    const double angle = atan2(sin_ab, cos_ab);     // the desired rotation angle
    /* ROS_INFO_STREAM("pi/2 about x: "<< Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(1,0,0)).toRotationMatrix()); */
    vec_rot = Eigen::AngleAxisd(angle, v.normalized()).toRotationMatrix();
    pos_cov = rotate_covariance(pos_cov, vec_rot);  // rotate the covariance to point in direction of est. position
    if (pos_cov.array().isNaN().any()){
      ROS_INFO_STREAM("NAN IN  COVARIANCE!!!!");
      /* if (_debug_){ */
      /*   ROS_INFO_STREAM("pos_cov: \n" <<pos_cov); */
      /*   /1* ROS_INFO_STREAM("v_x: \n" <<v_x); *1/ */
      /*   ROS_INFO_STREAM("sin_ab: " <<sin_ab); */
      /*   ROS_INFO_STREAM("cos_ab: " <<cos_ab); */
      /*   ROS_INFO_STREAM("vec_rot: \n"<<vec_rot); */
      /*   ROS_INFO_STREAM("a: " <<a.transpose()); */
      /*   ROS_INFO_STREAM("b: " <<b.transpose()); */
      /*   ROS_INFO_STREAM("v: " <<v.transpose()); */
      /* } */
      }
    return pos_cov;
  }
  //}

  /* separateByFrequency //{ */
  std::vector<std::vector<cv::Point3i>> separateByFrequency(std::vector< cv::Point3i > points){
    std::vector<std::vector<cv::Point3i>> separated_points;
    separated_points.resize(_target_count_);


      for (int i = 0; i < (int)(points.size()); i++) {
        if (points[i].z > 1) {
          int mid = ufc_->findMatch(points[i].z);
          int tid = classifyMatch(mid);
          if (_debug_)
            ROS_INFO("[%s]: FR: %d, MID: %d, TID: %d", ros::this_node::getName().c_str(),points[i].z, mid, tid);
          /* separated_points[classifyMatch(findMatch(points[i].z))].push_back(points[i]); */
          if (tid>=0)
            separated_points[tid].push_back(points[i]);
        }
      }
      return separated_points;
  }
  //}

  /* getClosestSet //{ */
  
  std::pair<int,std::vector<int>> getClosestSet(cv::Point3i &beacon, std::vector<cv::Point3i> &points, std::vector<bool> &marked_points){
    std::pair<int, std::vector<int>> output;
    output.first = -1;
    int b = 0;
    for (auto &bracket : bracket_set){
      cv::Rect bracket_placed(bracket.tl()+cv::Point(beacon.x,beacon.y), bracket.size());
      /* ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() <<"]: beacon: " << cv::Point(beacon.x,beacon.y) << " bracket_placed: " << bracket_placed); */
      std::vector<int> curr_selected_points;

      int i = -1;
      for (auto &point : points){
        i++;
        /* ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() <<"]: trying initial point: " << cv::Point2i(point.x, point.y)); */
        if (marked_points[i] == true){
          /* ROS_INFO("[%s]: MARKED!", ros::this_node::getName().c_str()); */
          continue;
        }

        if (bracket_placed.contains(cv::Point2i(point.x, point.y))){
            cv::Rect local_bracket(cv::Point2i(bracket_placed.tl().x,point.y-bracket_step), cv::Point2i(bracket_placed.br().x,point.y+bracket_step));
            /* ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() <<"]: point: " << point << " local_bracket: " << local_bracket); */
            int j = -1;
            for (auto &point_inner : points){
              j++;
              /* ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() <<"]: trying point: " << cv::Point2i(point_inner.x, point_inner.y)); */
              if (marked_points[j] == true){
                /* ROS_INFO("[%s]: MARKED!", ros::this_node::getName().c_str()); */
                continue;
              }
              if (local_bracket.contains(cv::Point2i(point_inner.x, point_inner.y))){
                  curr_selected_points.push_back(j);
                  /* ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() <<"]: PASSED"); */
                  }
              else{
                /* ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() <<"]: FAILED"); */
              }

            }
            break;
        }

      }

      if (curr_selected_points.size() > 0){
        output = {b, curr_selected_points};
        
        break;
      }


      b++;
    }

    return output;
  }
  
  //}

  /* separateByBeacon //{ */
  
  std::vector<std::vector<cv::Point3i>> separateByBeacon(std::vector< cv::Point3i > points){
    std::vector<std::vector<cv::Point3i>> separated_points;
  
    std::vector<bool> marked_points(points.size(), false);
    std::vector<int> midPoints(points.size(), -1);

    std::vector<cv::Point3i> emptySet;
  
    for (int i = 0; i < (int)(points.size()); i++) {
      if (points[i].z > 1) {
        int mid = ufc_->findMatch(points[i].z);
        /* ROS_INFO("[%s]: FR: %d, MID: %d", ros::this_node::getName().c_str(),points[i].z, mid); */
        midPoints[i] = mid;
        if (mid == 0){
          separated_points.push_back(emptySet);
          separated_points.back().push_back(points[i]);
          marked_points[i] = true;
        }
      }
    }


    int marked_beacon_count = 0;
    std::vector<bool> marked_beacons(points.size(), false);
    while (marked_beacon_count < (int)(marked_beacons.size())){
      std::vector<std::pair<int,std::vector<int>>> closest_sets;
      int i = -1;
      for (auto &curr_set : separated_points){
        i++;
        closest_sets.push_back({-1, std::vector<int>()});
        if ( marked_beacons[i] ){
          continue;
        }
        auto curr_beacon = curr_set[0];
        closest_sets[i] = getClosestSet(curr_beacon, points, marked_points);

      }
      int best_index = -1;
      for (int m = 0; m < (int)(closest_sets.size()); m++) {
        if (closest_sets[m].first < 0)
          continue;
        if (best_index == -1){
          best_index = m;
          continue;
        }

        if (closest_sets[best_index].first > closest_sets[m].first){
          best_index = m;
        }
        else if (closest_sets[best_index].first == closest_sets[m].first) {
          if (closest_sets[best_index].second.size() < closest_sets[m].second.size()){
            best_index = m;
          }
        }
      }

      marked_beacon_count++;

      if (best_index == -1){
        break;
      }

      for (auto &subset_point_index : closest_sets[best_index].second){
        separated_points[best_index].push_back(points[subset_point_index]);
        marked_points[subset_point_index] = true;

      }
      marked_beacons[best_index] = true;

    }

    int i=-1;
    for (auto &point: points){
      i++;
      if (marked_points[i])
        continue;

      separated_points.push_back(emptySet);
      separated_points.back().push_back(cv::Point3i(-1,-1,-1)); //this means that this set does not have a beacon
      separated_points.back().push_back(points[i]);
      marked_points[i] = true;
      for (int j = i+1; j < (int)(points.size()); j++) {
        if (marked_points[j])
          continue;
        if (cv::norm(point - points[j]) < maxDistInit){
          separated_points.back().push_back(points[j]);
          marked_points[j] = true;
        }
      }
    }

    return separated_points;
  }
  
  //}

  /* extractSingleRelative //{ */
  void extractSingleRelative(std::vector< cv::Point3i > points, int target, size_t imageIndex) {

    double leftF;
    double rightF;
    if (_beacon_){
      leftF = _frequencies_[1];
      if (_frequencies_.size() == 3)
        rightF = _frequencies_[2];
      else
        rightF = _frequencies_[1];
    }
    else {
      leftF = _frequencies_[target*2];
      rightF = _frequencies_[target*2+1];
    }
    double          maxDist = maxDistInit;

    int countSeen = (int)(points.size());

    bool missing_beacon = (_beacon_)&&(points[0].x == -1);

    if (missing_beacon){
      points.erase(points.begin());
    }

    if ((!_beacon_) || (missing_beacon)){
      if ((int)(points.size()) > 1) {

        for (int i = 0; i < (int)(points.size()); i++) {
          if (points[i].z < 1) {
            points.erase(points.begin() + i);
            i--;
            continue;
          }
        }
        bool separated = false;
        while ((points.size() > 3) || (!separated)) {
          separated = true;
          for (int i = 0; i < (int)(points.size()); i++) {
            bool viable = false;
            for (int j = 0; j < (int)(points.size()); j++) {
              if (i == j)
                continue;

              /* if ((cv::norm(points[i] - points[j]) < maxDist) && (abs(points[i].y - points[j].y) < abs(points[i].x - points[j].x))) { */
              if (_debug_)
                ROS_INFO_STREAM("Distance: " <<cv::norm(cv::Point2i(points[i].x,points[i].y) - cv::Point2i(points[j].x,points[j].y)) << " maxDist: " << maxDist);

              if ((cv::norm(cv::Point2i(points[i].x,points[i].y) - cv::Point2i(points[j].x,points[j].y)) < maxDist)) {
                viable = true;
              }
              else{
                separated = false;
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
      }


      unscented::measurement ms;

      if (_debug_)
        ROS_INFO_STREAM("framerateEstim: " << estimatedFramerate[imageIndex]);

      double perr=0.2/estimatedFramerate[imageIndex];

            /* if (_debug_) */
      /* ROS_INFO_STREAM("points: " << points); */
      if (_quadrotor_) {
        if ((_beacon_) && (!missing_beacon)){
          if (points.size() == 1) {
            ROS_INFO_THROTTLE(1.0,"[%s]: Only one beacon visible - no distance information", ros::this_node::getName().c_str());
            if (_debug_)
              std::cout << "led: " << points[0] << std::endl;


            ms = uvdarQuadrotorPose1p_meas(Eigen::Vector2d(points[0].x,points[0].y),_arm_length_, 1000,10.0, imageIndex);


            foundTarget = true;
            lastSeen    = ros::Time::now();
          }
          else if (points.size() == 2){
            if (_debug_)
              ROS_INFO_STREAM("points: " << points);
            X2qb <<
              points[0].x ,points[0].y, // presume that the beacon really is a beacon
              points[1].x ,points[1].y, 1.0/(double)(points[1].z),
              0,  //to account for ambiguity (roll)
              0,  //to account for ambiguity (pitch)
              0;  //ambiguity - which of the two markers of a given ID is it?
            Px2qb <<
              qpix,0,     0,   0,   0,         0,            0,           0,
              0,   qpix,  0,   0,   0,         0,            0,           0,
              0,   0,     qpix,0,   0,         0,            0,           0,
              0,   0,     0,   qpix,0,         0,            0,           0,
              0,   0,     0,   0,   sqr(perr), 0,            0,           0,
              0,   0,     0,   0,   0,         sqr(M_PI/36), 0,           0,          //tilt_par
              0,   0,     0,   0,   0,         0,           sqr(M_PI/18),   0,          //tilt_perp
              0,   0,     0,   0,   0,         0,            0,           sqr(M_PI_2) //ambig
                ;
            if (_debug_)
              ROS_INFO_STREAM("X2qb: " << X2qb);
            boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd,int)> callback;
            callback=boost::bind(&UVDARPoseCalculator::uvdarQuadrotorPose2pB,this,_1,_2,_3);
            ms = unscented::unscentedTransform(X2qb,Px2qb,callback,leftF,rightF,-1,imageIndex);
          }
          else if (points.size() == 3){
            if (_debug_)
              ROS_INFO_STREAM("points: " << points);
            X3qb <<
              points[0].x ,points[0].y, // presume that the beacon really is a beacon
              points[1].x ,points[1].y, 1.0/(double)(points[1].z),
              points[2].x ,points[2].y, 1.0/(double)(points[2].z),
              0;
            Px3qb <<
              qpix,0,    0,   0,   0,        0,   0,   0,         0,
              0,   qpix, 0,   0,   0,        0,   0,   0,         0,
              0,   0,    qpix,0,   0,        0,   0,   0,         0,   
              0,   0,    0,   qpix,0,        0,   0,   0,         0,  
              0,   0,    0,   0,   sqr(perr),0,   0,   0,         0,    
              0,   0,    0,   0,   0,        qpix,0,   0,         0,     
              0,   0,    0,   0,   0,        0,   qpix,0,         0,      
              0,   0,    0,   0,   0,        0,   0,   sqr(perr), 0,
              0,   0,    0,   0,   0,        0,   0,   0,         sqr(M_PI*2)
                ;
            if (_debug_)
              ROS_INFO_STREAM("X3qb: " << X3qb);
            boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd,int)> callback;
            callback=boost::bind(&UVDARPoseCalculator::uvdarQuadrotorPose3pB,this,_1,_2,_3);
            ms = unscented::unscentedTransform(X3qb,Px3qb,callback,leftF,rightF,-1,imageIndex);
          }
          else if (points.size() == 4){
            if (_debug_)
              ROS_INFO_STREAM("points: " << points);
            X4qb <<
              points[0].x ,points[0].y, // presume that the beacon really is a beacon
              points[1].x ,points[1].y, 1.0/(double)(points[1].z),
              points[2].x ,points[2].y, 1.0/(double)(points[2].z),
              points[3].x ,points[3].y, 1.0/(double)(points[3].z),
              0;
            Px4qb <<
              qpix,0,   0,   0,   0,        0,   0,   0,         0,   0,    0,         0,
              0,   qpix,0,   0,   0,        0,   0,   0,         0,   0,    0,         0,
              0,   0,   qpix,0,   0,        0,   0,   0,         0,   0,    0,         0,   
              0,   0,   0,   qpix,0,        0,   0,   0,         0,   0,    0,         0,  
              0,   0,   0,   0,   sqr(perr),0,   0,   0,         0,   0,    0,         0,    
              0,   0,   0,   0,   0,        qpix,0,   0,         0,   0,    0,         0,     
              0,   0,   0,   0,   0,        0,   qpix,0,         0,   0,    0,         0,      
              0,   0,   0,   0,   0,        0,   0,   sqr(perr), 0,   0,    0,         0,
              0,   0,   0,   0,   0,        0,   0,   0,         qpix,0,    0,         0, 
              0,   0,   0,   0,   0,        0,   0,   0,         0,   qpix, 0,         0, 
              0,   0,   0,   0,   0,        0,   0,   0,         0,   0,    sqr(perr), 0,
              0,   0,   0,   0,   0,        0,   0,   0,         0,   0,    0,         sqr(2*M_PI/3)
                ;
            if (_debug_)
              ROS_INFO_STREAM("X4qb: " << X4qb);
            boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd,int)> callback;
            callback=boost::bind(&UVDARPoseCalculator::uvdarQuadrotorPose4pB,this,_1,_2,_3);
            ms = unscented::unscentedTransform(X4qb,Px4qb,callback,leftF,rightF,-1,imageIndex);
            /* if (_debug_) */
              /* ROS_INFO_STREAM("Vert. angle: " << deg2rad(atan2(ms.x(1),sqrt(ms.x(0)*ms.x(0)+ms.x(2)*ms.x(2))))); */
          }
          else {
            ROS_INFO_THROTTLE(1.0,"[%s]: No valid points seen. Waiting", ros::this_node::getName().c_str());
            tailingComponent      = 0;
            foundTarget           = false;
            return;
          }
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: beacons not yet implemented"); */
        }
        else {
          if (points.size() == 3) {
            if (_debug_)
              ROS_INFO_STREAM("points: " << points);
            X3 <<
              points[0].x ,points[0].y, 1.0/(double)(points[0].z),
              points[1].x ,points[1].y, 1.0/(double)(points[1].z),
              points[2].x, points[2].y, 1.0/(double)(points[2].z),
              0;  //to account for ambiguity
            Px3 <<
              qpix,0,   0,        0,   0,   0,        0,   0,   0,        0,
              0,   qpix,0,        0,   0,   0,        0,   0,   0,        0,
              0,   0,   sqr(perr),0,   0,   0,        0,   0,   0,        0,
              0,   0,   0,        qpix,0,   0,        0,   0,   0,        0,
              0,   0,   0,        0,   qpix,0,        0,   0,   0,        0,
              0,   0,   0,        0,   0,   sqr(perr),0,   0,   0,        0,
              0,   0,   0,        0,   0,   0,        qpix,0,   0,        0,
              0,   0,   0,        0,   0,   0,        0,   qpix,0,        0,
              0,   0,   0,        0,   0,   0,        0,   0,   sqr(perr),0,
              0,   0,   0,        0,   0,   0,        0,   0,   0,        sqr(2*M_PI/3)
                ;
            if (_debug_)
              ROS_INFO_STREAM("X3: " << X3);
            boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd,int)> callback;
            callback=boost::bind(&UVDARPoseCalculator::uvdarQuadrotorPose3p,this,_1,_2,_3);
            ms = unscented::unscentedTransform(X3,Px3,callback,leftF,rightF,-1,imageIndex);
          }
          else if (points.size() == 2) {
            if (_debug_)
              ROS_INFO_STREAM("points: " << points);
            X2q <<
              (double)(points[0].x) ,(double)(points[0].y),1.0/(double)(points[0].z),
              (double)(points[1].x) ,(double)(points[1].y),1.0/(double)(points[1].z),
              0.0,0.0;
            Px2q <<
              qpix ,0,0,0,0,0,0,0,
                   0,qpix ,0,0,0,0,0,0,
                   0,0,sqr(perr),0,0,0,0,0,
                   0,0,0,qpix ,0,0,0,0,
                   0,0,0,0,qpix ,0,0,0,
                   0,0,0,0,0,sqr(perr),0,0,
                   0,0,0,0,0,0,sqr(deg2rad((missing_beacon?70:10))),0,
                   0,0,0,0,0,0,0,sqr(deg2rad(10))
                     ;

            if (_debug_)
              ROS_INFO_STREAM("X2: " << X2q);
            boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd, int)> callback;
            callback=boost::bind(&UVDARPoseCalculator::uvdarQuadrotorPose2p,this,_1,_2,_3);
            ms = unscented::unscentedTransform(X2q,Px2q,callback,leftF,rightF,-1,imageIndex);
          }
          else if (points.size() == 1) {
            ROS_INFO_THROTTLE(1.0,"[%s]: Only single point visible - no distance information", ros::this_node::getName().c_str());
            if (_debug_)
              std::cout << "led: " << points[0] << std::endl;


            ms = uvdarQuadrotorPose1p_meas(Eigen::Vector2d(points[0].x,points[0].y),_arm_length_, 1000,10.0, imageIndex);


            foundTarget = true;
            lastSeen    = ros::Time::now();
          } else {
            ROS_INFO_THROTTLE(1.0,"[%s]: No valid points seen. Waiting", ros::this_node::getName().c_str());
            tailingComponent      = 0;
            foundTarget           = false;
            return;
          }

        }
      }
      else {
        if (_beacon_) {
            ROS_WARN_THROTTLE(1.0,"[%s]: Beacon-based estimation for hexarotors is not implemented!", ros::this_node::getName().c_str());
        }

        if (points.size() == 3) {
          if (_debug_)
            ROS_INFO_STREAM("points: " << points);
          X3 <<
            points[0].x ,points[0].y, 1.0/(double)(points[0].z),
            points[1].x ,points[1].y, 1.0/(double)(points[1].z),
            points[2].x, points[2].y, 1.0/(double)(points[2].z),
            0;  //to account for ambiguity
          Px3 <<
            qpix ,0,0,0,0,0,0,0,0,0,
                 0,qpix ,0,0,0,0,0,0,0,0,
                 0,0,sqr(perr),0,0,0,0,0,0,0,
                 0,0,0,qpix ,0,0,0,0,0,0,
                 0,0,0,0,qpix ,0,0,0,0,0,
                 0,0,0,0,0,sqr(perr),0,0,0,0,
                 0,0,0,0,0,0,qpix ,0,0,0,
                 0,0,0,0,0,0,0,qpix ,0,0,
                 0,0,0,0,0,0,0,0,sqr(perr),0,
                 0,0,0,0,0,0,0,0,0,sqr(2*M_PI/3)
                   ;
          if (_debug_)
            ROS_INFO_STREAM("X3: " << X3);
          boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd, int)> callback;
          callback=boost::bind(&UVDARPoseCalculator::uvdarHexarotorPose3p,this,_1,_2,_3);
          ms = unscented::unscentedTransform(X3,Px3,callback,leftF,rightF,-1,imageIndex);
        }
        else if (points.size() == 2) {
          if (_debug_)
            ROS_INFO_STREAM("points: " << points);
          X2 <<
            (double)(points[0].x) ,(double)(points[0].y),1.0/(double)(points[0].z),
            (double)(points[1].x) ,(double)(points[1].y),1.0/(double)(points[1].z),
            0.0,0.0,0.0;
          Px2 <<
            qpix ,0,0,0,0,0,0,0,0,
                 0,qpix ,0,0,0,0,0,0,0,
                 0,0,sqr(perr),0,0,0,0,0,0,
                 0,0,0,qpix ,0,0,0,0,0,
                 0,0,0,0,qpix ,0,0,0,0,
                 0,0,0,0,0,sqr(perr),0,0,0,
                 0,0,0,0,0,0,sqr(deg2rad(15)),0,0,
                 0,0,0,0,0,0,0,sqr(deg2rad(60)),0,
                 0,0,0,0,0,0,0,0,sqr(deg2rad(10))
                   ;

          if (_debug_)
            ROS_INFO_STREAM("X2: " << X2);
          boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd, int)> callback;
          callback=boost::bind(&UVDARPoseCalculator::uvdarHexarotorPose2p,this,_1,_2,_3);
          ms = unscented::unscentedTransform(X2,Px2,callback,leftF,rightF,-1,imageIndex);
        }
        else if (points.size() == 1) {
          std::cout << "Only single point visible - no distance information" << std::endl;
          /* std::cout << "led: " << points[0] << std::endl; */


          ms = uvdarHexarotorPose1p_meas(Eigen::Vector2d(points[0].x,points[0].y),_arm_length_, (TUBE_LENGTH),10.0, imageIndex);


          foundTarget = true;
          lastSeen    = ros::Time::now();
        } else {
          std::cout << "No valid points seen. Waiting" << std::endl;
          tailingComponent      = 0;
          foundTarget           = false;
          return;
        }
      }


      /* Eigen::MatrixXd iden(6,6); */
      /* iden.setIdentity(); */
      /* ms.C +=iden*0.1; */

      ms.C += e::MatrixXd::Identity(6,6)*0.0001;
      if (_debug_){
      /* if (true){ */
        ROS_INFO_STREAM("Y: \n" << ms.x );
        ROS_INFO_STREAM("Py: \n" << ms.C );
      }
      if (ms.x.topLeftCorner(3,1).norm() < 1.5)
        return;

      e::Vector3d unit_vec;
      unit_vec << 0,0,1.0;
      if (acos(ms.x.topLeftCorner(3,1).normalized().dot(unit_vec)) > rad2deg(190))
        return;

      tf::Quaternion qtemp;
      qtemp.setRPY((ms.x(3)), (ms.x(4)), (ms.x(5)));
      qtemp=tf::Quaternion(-0.5,0.5,-0.5,-0.5)*qtemp; //bring relative orientations to the optical frame of the camera
      qtemp.normalize();//just in case

      if (!_beacon_){
        msg_odom_ = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();

        msg_odom_->pose.pose.position.x = ms.x(0);
        msg_odom_->pose.pose.position.y = ms.x(1);
        msg_odom_->pose.pose.position.z = ms.x(2);
        msg_odom_->pose.pose.orientation.x = qtemp.x();
        msg_odom_->pose.pose.orientation.y = qtemp.y();
        msg_odom_->pose.pose.orientation.z = qtemp.z();
        msg_odom_->pose.pose.orientation.w = qtemp.w();
        for (int i=0; i<ms.C.cols(); i++){
          for (int j=0; j<ms.C.rows(); j++){
            msg_odom_->pose.covariance[ms.C.cols()*j+i] =  ms.C(j,i);
          }
        }
        msg_odom_->header.frame_id = _camera_frames_[imageIndex];
        msg_odom_->header.stamp = lastBlinkTime;

        measuredPose[target].publish(msg_odom_);
      }
      else {

        mrs_msgs::PoseWithCovarianceIdentified pose;

        pose.id = imageIndex*1000+msg_measurement_array_[imageIndex]->poses.size();
        pose.pose.position.x = ms.x(0);
        pose.pose.position.y = ms.x(1);
        pose.pose.position.z = ms.x(2);
        pose.pose.orientation.x = qtemp.x();
        pose.pose.orientation.y = qtemp.y();
        pose.pose.orientation.z = qtemp.z();
        pose.pose.orientation.w = qtemp.w();
        for (int i=0; i<ms.C.cols(); i++){
          for (int j=0; j<ms.C.rows(); j++){
            pose.covariance[ms.C.cols()*j+i] =  ms.C(j,i);
          }
        }
        msg_measurement_array_[imageIndex]->poses.push_back(pose);
      }

  }
    //}

    /* sgn //{ */
  template < typename T >
  int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }
  //}

  /* classifyMatch //{ */
  int classifyMatch(int ID) {
    return ID/frequenciesPerTarget;
  }
  //}

  /* prepareBlinkerBrackets //{ */
  void prepareBlinkerBrackets() {
    double frame_ratio = _arm_length_/(_beacon_height_*0.75);
    double max_dist = 100;

    cv::Rect curr_rect;
      for (int i = 0; i < max_dist/bracket_step; i++) {
        curr_rect = cv::Rect(cv::Point2i(-frame_ratio*i*bracket_step-5, 0), cv::Point(frame_ratio*i*bracket_step+5,i*bracket_step+5));
        bracket_set.push_back(curr_rect);
      }
  }
  //}
  
  /* ShowThread() //{ */

  void ShowThread() {
    cv::Mat temp;

    while (ros::ok()) {
      if (initialized_){
        std::scoped_lock lock(mutex_show);

        if (_gui_){
          if (GenerateVisualization() >= 0){
            cv::imshow("ocv_point_separation_" + _uav_name_, view_image_);
          }
        }

      }
      cv::waitKey(1000.0 / 10.0);
    }
  }

  //}
  
  /* GenerateVisualization() //{ */

    int GenerateVisualization() {
      
      int image_count = (int)(separated_points_.size());
      int image_width, image_height;

      image_width = 752;
      image_height = 480;
      view_image_ = cv::Mat(
          image_height, 
          (image_width + 1) * image_count - 1, 
          CV_8UC3,
          cv::Scalar(0,0,0));


      if (view_image_.rows == 0 || view_image_.cols == 0) return -1;

      /* loop through separated points //{ */

      std::scoped_lock lock(mutex_separated_points);
      int imageIndex = -1;
      for (auto &image_point_set : separated_points_){
        imageIndex++;
        cv::line(view_image_, cv::Point2i(image_width, 0), cv::Point2i(image_width,image_height-1),cv::Scalar(255,255,255));
        int differenceX = (image_width + 2) * imageIndex;
        int i = -1;
        for (auto &point_group : image_point_set){
          i++;
          for (auto &point : point_group){
            cv::Point center = cv::Point(point.x + differenceX, point.y);

            cv::Scalar color = ColorSelector::markerColor(i);
            cv::circle(view_image_, center, 5, color);
          }
          
          
        }
      }

      //}

      return 0;
  }

  //}



/* rotmatToYaw //{ */
double rotmatToYaw(e::Matrix3d m){
  return atan2(m(1,0),m(0,0));
}
//}
/* rotmatToPitch //{ */
double rotmatToPitch(e::Matrix3d m){
  return atan2( -m(2,0), sqrt( m(2,1)*m(2,1) +m(2,2)*m(2,2) )  );
}
//}
/* rotmatToRoll //{ */
double rotmatToRoll(e::Matrix3d m){
  return atan2(m(2,1),m(2,2));
}
//}

  /* Variables //{ */

  bool stopped;

  bool Flip;

  ros::Time RangeRecTime;

  ros::Subscriber framerateSubscriber;

  std::vector<std::string> _camera_frames_;

  using blinkers_seen_callback_t = std::function<void (const mrs_msgs::ImagePointsWithFloatStampedConstPtr& msg)>;
  std::vector<blinkers_seen_callback_t> blinkersSeenCallbacks;
  std::vector<ros::Subscriber> blinkersSeenSubscribers;

  using estimated_framerate_callback_t = std::function<void (const std_msgs::Float32ConstPtr& msg)>;
  std::vector<estimated_framerate_callback_t> estimatedFramerateCallbacks;
  std::vector<ros::Subscriber> estimatedFramerateSubscribers;

  std::vector<double> estimatedFramerate;

  ros::Time begin;

  // Input arguments
  bool _debug_;
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

  double tailingComponent;

  bool _gui_;
  std::thread show_thread;
  std::mutex mutex_show;
  cv::Mat view_image_;

  bool cameraRotated;

  double max_px_speed_t;
  float  maxAccel;
  bool   checkAccel;

  std::string _uav_name_;

  ros::Time lastBlinkTime;


  /* uvLedDetect_gpu *uvdg; */

  mrs_lib::Transformer                        transformer_;

  /* std::vector< sensor_msgs::Imu > imu_register; */

  std::vector<struct ocam_model> _oc_models_;

  bool                           foundTarget;


  Eigen::MatrixXd Px2,Px3,Px2q;
  Eigen::MatrixXd Px2qb,Px3qb,Px4qb;
  Eigen::VectorXd X2,X3,X2q;
  Eigen::VectorXd X2qb,X3qb,X4qb;


  ros::Subscriber OdomSubscriber;
  ros::Subscriber DiagSubscriber;
  bool            reachedTarget = false;
  ros::Time       lastSeen;

  bool               followTriggered = false;
  ros::ServiceServer ser_trigger;

  std::vector<ros::Publisher> measuredPose;
  std::vector<ros::Publisher> measured_poses_;

  /* Lkf* trackers[2]; */

  geometry_msgs::PoseWithCovarianceStampedPtr msg_odom_;
  std::vector<mrs_msgs::PoseWithCovarianceArrayStampedPtr> msg_measurement_array_;

  int frequenciesPerTarget;
  int _target_count_;
  std::vector<double> _frequencies_;
  std::unique_ptr<UVDARFrequencyClassifier> ufc_;

  unsigned int _camera_count_;

  std::vector<cv::Rect> bracket_set;

  std::vector<std::string> _calib_files_;

  bool _use_masks_;
  std::vector<std::string> _mask_file_names_;
  std::vector<cv::Mat> _masks_;

  double _arm_length_;
  double _beacon_height_;

  bool _quadrotor_;
  bool _beacon_;

  bool initialized_ = false;

  std::mutex mutex_separated_points;
  std::vector<std::vector<std::vector<cv::Point3i>>> separated_points_;
  //}
};

} //uvdar

int main(int argc, char** argv) {
  ros::init(argc, argv, "uvdar_pose_calculator");

  ros::NodeHandle nh("~");
  uvdar::UVDARPoseCalculator        upc(nh);
  ROS_INFO("[UVDARPoseCalculator]: UVDAR Pose calculator node initiated");
  ros::spin();
  return 0;
}

