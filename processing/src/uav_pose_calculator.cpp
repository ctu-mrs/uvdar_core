#define MAX_DIST_INIT 100.0
#define BRACKET_STEP 10
#define QPIX 1 //pixel std. dev
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
#include <mrs_lib/image_publisher.h>
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


  /**
   * @brief A processing class for converting retrieved blinking markers from a UV camera image into relative poses of observed UAV that carry these markers, as well as the error covariances of these estimates
   */
  class UVDARPoseCalculator {
    public:

      /**
       * @brief Constructor - loads parameters and initializes necessary structures
       *
       * @param nh_ Private NodeHandle of this ROS node
       */
      /* Constructor //{ */
      UVDARPoseCalculator(ros::NodeHandle& nh_) {
        ROS_INFO("[UVDARPoseCalculator]: Initializing pose calculator...");

        mrs_lib::ParamLoader param_loader(nh_, "UVDARPoseCalculator");

        param_loader.loadParam("uav_name", _uav_name_, std::string());

        param_loader.loadParam("debug", _debug_, bool(false));

        param_loader.loadParam("gui", _gui_, bool(false));
        param_loader.loadParam("publish_visualization", _publish_visualization_, bool(false));

        param_loader.loadParam("arm_length",_arm_length_,double(0.2775));

        param_loader.loadParam("quadrotor",_quadrotor_,bool(false));

        param_loader.loadParam("beacon",_beacon_,bool(false));
        param_loader.loadParam("beacon_height",_beacon_height_,double(0.2));

        /* load the frequencies //{ */
        param_loader.loadParam("frequencies", _frequencies_);
        if (_frequencies_.empty()){
          std::vector<double> default_frequency_set;
          if (_beacon_){
            default_frequency_set = {30, 15};
          }
          else {
            default_frequency_set = {5, 6, 8, 10, 15, 30};
          }
          ROS_WARN("[UVDARPoseCalculator]: No frequencies were supplied, using the default frequency set. This set is as follows: ");
          for (auto f : default_frequency_set){
            ROS_WARN_STREAM("[UVDARPoseCalculator]: " << f << " hz");
          }
          _frequencies_ = default_frequency_set;
        }
        ufc_ = std::make_unique<UVDARFrequencyClassifier>(_frequencies_);

        param_loader.loadParam("frequencies_per_target", _frequencies_per_target_, int(1));
        _target_count_ = (int)(_frequencies_.size())/_frequencies_per_target_;

        //}

        if (_beacon_){
          prepareBlinkerBrackets();
        }

        /* Subscribe to blinking point topics and advertise poses//{ */
        std::vector<std::string> _blinkers_seen_topics;
        param_loader.loadParam("blinkers_seen_topics", _blinkers_seen_topics, _blinkers_seen_topics);
        if (_blinkers_seen_topics.empty()) {
          ROS_WARN("[UVDARPoseCalculator]: No topics of blinkers were supplied");
        }
        _camera_count_ = (unsigned int)(_blinkers_seen_topics.size());

        cals_blinkers_seen_.resize(_blinkers_seen_topics.size());
        separated_points_.resize(_blinkers_seen_topics.size());
        for (size_t i = 0; i < _blinkers_seen_topics.size(); ++i) {
          blinkers_seen_callback_t callback = [image_index=i,this] (const mrs_msgs::ImagePointsWithFloatStampedConstPtr& pointsMessage) { 
            ProcessPoints(pointsMessage, image_index);
          };
          cals_blinkers_seen_[i] = callback;
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Subscribing to " << _blinkers_seen_topics[i]);
          sub_blinkers_seen_.push_back(
              nh_.subscribe(_blinkers_seen_topics[i], 1, cals_blinkers_seen_[i]));

          ROS_INFO_STREAM("[UVDARPoseCalculator]: Advertising measured poses " << i+1);
          pub_measured_poses_.push_back(nh_.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("measuredPoses"+std::to_string(i+1), 1)); 


          camera_image_sizes_.push_back(cv::Size(-1,-1));

          mutex_separated_points_.push_back(std::make_shared<std::mutex>());
        }
        //}

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

        /* Subscribe to estimated framerate topics //{ */
        std::vector<std::string> _estimated_framerate_topics;
        param_loader.loadParam("estimated_framerate_topics", _estimated_framerate_topics, _estimated_framerate_topics);
        // fill the framerates with -1
        estimated_framerate_.insert(estimated_framerate_.begin(), _estimated_framerate_topics.size(), -1);

        if (_estimated_framerate_topics.empty()) {
          ROS_WARN("[UVDARPoseCalculator]: No topics of estimated framerates were supplied");
        }
        if (_blinkers_seen_topics.size() != _estimated_framerate_topics.size()) {
          ROS_ERROR_STREAM("[UVDARPoseCalculator]: The size of blinkers_seen_topics (" << _blinkers_seen_topics.size() <<
              ") is different from estimated_framerate_topics (" << _estimated_framerate_topics.size() << ")");
        }
        for (size_t i = 0; i < _estimated_framerate_topics.size(); ++i) {
          estimated_framerate_callback_t callback = [image_index=i,this] (const std_msgs::Float32ConstPtr& framerateMessage) { 
            estimated_framerate_[image_index] = framerateMessage->data;
          };
          cals_estimated_framerate_.push_back(callback);

          ROS_INFO_STREAM("[UVDARPoseCalculator]: Subscribing to " << _estimated_framerate_topics[i]);
          sub_estimated_framerate_.push_back(
              nh_.subscribe(_estimated_framerate_topics[i], 1, cals_estimated_framerate_[i]));
        }
        //}

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


        /* Set transformation frames for cameras //{ */
        param_loader.loadParam("camera_frames", _camera_frames_, _camera_frames_);
        if (_camera_frames_.size() != _blinkers_seen_topics.size()) {
          ROS_ERROR_STREAM("The size of camera_frames (" << _camera_frames_.size() << 
              ") is different from blinkers_seen_topics size (" << _blinkers_seen_topics.size() << ")");
        }
        //}

        if (_gui_ || _publish_visualization_){
          if (_publish_visualization_){
            pub_visualization_ = std::make_unique<mrs_lib::ImagePublisher>(boost::make_shared<ros::NodeHandle>(nh_));
          }
          timer_visualization_ = nh_.createTimer(ros::Rate(1), &UVDARPoseCalculator::VisualizationThread, this, false);
        }


        initialized_ = true;
      }
      //}

      /* Destructor //{ */
      ~UVDARPoseCalculator() {
      }
      //}

      /**
       * @brief Load the mask files - either form absolute path or composite filename found in the mrs_uav_general package.
       *
       * @return success
       */
      /* loadMasks //{ */
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

      /**
       * @brief Load the camera calibration files - either form absolute path or composite filename found in the mrs_uav_general package.
       *
       * @return success
       */
      /* loadCalibrations //{ */
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


      /**
       * @brief Callback to process a new set of retrieved blinking markers into estimates of relative poses of observed UAVs
       *
       * @param msg The input message - set of retrieved blinking marker points from a UV camera - contain image positions and blinking frequencies
       * @param image_index The index of the current camera used to generate the input message
       */
      /* ProcessPoints //{ */
      void ProcessPoints(const mrs_msgs::ImagePointsWithFloatStampedConstPtr& msg, size_t image_index) {
        /* int                        countSeen; */
        std::vector< cv::Point3d > points;
        last_blink_time_ = msg->stamp;
        if (_debug_)
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Received points: " << msg->points.size());
        if (msg->points.size() < 1) {
          return;
        }
        if (estimated_framerate_.size() <= image_index || estimated_framerate_[image_index] < 0) {
          ROS_INFO_THROTTLE(1.0,"[UVDARPoseCalculator]: Framerate is not yet estimated. Waiting...");
          return;
        }

        camera_image_sizes_[image_index].width = msg->image_width;
        camera_image_sizes_[image_index].height = msg->image_height;

        for (auto& point : msg->points) {
          if (_use_masks_){
            if (_masks_[image_index].at<unsigned char>(cv::Point2i(point.x, point.y)) < 100){
              if (_debug_){
                ROS_INFO_STREAM("[UVDARPoseCalculator]: Discarding point " << cv::Point2i(point.x, point.y) << " with f="  << point.value);
              }
              continue;
            }
          }

          if (point.value <= 200) {
            points.push_back(cv::Point3d(point.x, point.y, point.value));
          }
        }

        std::scoped_lock lock(*(mutex_separated_points_[image_index]));

        mrs_msgs::PoseWithCovarianceArrayStamped msg_measurement_array;
        msg_measurement_array.header.frame_id = _camera_frames_[image_index];
        msg_measurement_array.header.stamp = last_blink_time_;

        if ((int)(points.size()) > 0) {
          if (_beacon_){
            separated_points_[image_index] = separateByBeacon(points);
          }
          else {
            separated_points_[image_index] = separateByFrequency(points);
          }

          for (int i = 0; i < ((int)(separated_points_[image_index].size())); i++) {
            if (_debug_){
              ROS_INFO_STREAM("[UVDARPoseCalculator]: target [" << separated_points_[image_index][i].first << "]: ");
              ROS_INFO_STREAM("[UVDARPoseCalculator]: p: " << separated_points_[image_index][i].second);
            }
            mrs_msgs::PoseWithCovarianceIdentified pose;
            extractSingleRelative(separated_points_[image_index][i].second, separated_points_[image_index][i].first, image_index, pose);
            msg_measurement_array.poses.push_back(pose);
          }
        }
        pub_measured_poses_[image_index].publish(msg_measurement_array);
      }
      //}


    private:

      /* Specific calculations used for individual cases of UAV models detection of various numbers of their markers//{ */


      /**
       * @brief Generates relative position and error covariance of a hexarotor if only one of its markers, carried on the ends of its arms, is seen. The result is an elongated covariance covering a range of positions along the line of sight towards the marker This is not used with unscented transform
       *
       * @param X 2D vector containing the image coordinates of the sole observed marker
       * @param tubewidth Defines the "width" of the output covariance (standard deviation perpendicular to the line of sight)
       * @param tubelength Defines the "length" of the output covariance (standard deviation parallel to the line of sight)
       * @param mean_dist The mean distance of the UAV in the output measurement - with one marker we do not have informaiton on distance, but we want the mean and covariance to cover the possible range as much as possible, and avoid extending too much behind the camera
       * @param image_index The index of the current camera used to generate the input observed marker
       *
       * @return Measurement comprising mean pose and its error covariance
       */
      /* uvdarHexarotorPose1p //{ */
      unscented::measurement uvdarHexarotorPose1p_meas(Eigen::Vector2d X,double tubewidth, double tubelength, double mean_dist, int image_index){
        double v1[3];
        double x[2] = {X.y(),X.x()};
        cam2world(v1, x, &_oc_models_[image_index]);
        Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
        V1 = V1*mean_dist;

        unscented::measurement ms;
        ms.x = Eigen::VectorXd(6);
        ms.C = Eigen::MatrixXd(6,6);

        ms.x << V1.x(),V1.y(),V1.z(),0,0,0; 
        Eigen::MatrixXd temp;
        temp.setIdentity(6,6);
        ms.C = temp*666;//large covariance for angles in radians
        ms.C.topLeftCorner(3, 3) = getLongCovariance(V1,tubewidth,tubelength);

        /* std::cout << "ms.C: " << ms.C << std::endl; */


        return ms;

      }
      //}


      /**
       * @brief Calculates relative position of a hexarotor if two of its markers, carried on the ends of its arms, is seen. This is used with unscented transform to express error covariance of the estimated pose from the errors of the input
       *
       * @param X The input vector containing image positions and frequencies of the markers, as well as the means of ambiguities of relative yaw, relative tilt and a range used to cover ambiguity of a specific sequences of points. These means will be perturbed in the unscented transform to spread the output error covariance accordingly
       * @param expFrequencies A vector of expected frequencies belonging to the given UAV. These are used to compare to the retrieved frequencies and account for the possible error of frequency retrieval
       * @param image_index The index of the current camera used to generate the input observed marker
       *
       * @return Estimated relative position of the UAV, as well as its orientation expressed in relative Roll, Pitch and Yaw
       */
      /* uvdarHexarotorPose2p //{ */
      Eigen::VectorXd uvdarHexarotorPose2p(Eigen::VectorXd X, Eigen::VectorXd expFrequencies, int image_index){

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

        double      v1[3], v2[3];
        double      va[2] = {double(a.y), double(a.x)};
        double      vb[2] = {double(b.y), double(b.x)};
        ;
        cam2world(v1, va, &(_oc_models_[image_index]));
        cam2world(v2, vb, &(_oc_models_[image_index]));

        Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
        Eigen::Vector3d V2(v2[1], v2[0], -v2[2]);

        // Details of the following calculation are explained in [V Walter, M Saska and A Franchi. "Fast mutual relative localization of uavs using ultraviolet led markers." (ICUAS 2018). 2018] 

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

        if (expFrequencies.size() == 2) //different frequencies on each side (portside and starboard)
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

        relyaw=relyaw-latang;

        double latnorm=sqrt(sqr(Yt(0))+sqr(Yt(2)));
        double Gamma=atan2(Yt(1),latnorm);
        double tilt_perp=X(8);
        Eigen::Vector3d obs_normal=V2.cross(V1);
        obs_normal=obs_normal/(obs_normal.norm());
        Eigen::Vector3d latComp;
        latComp << Vc(0),0,Vc(2);
        latComp = latComp/(latComp.norm());

        if (Vc(1)<0) latComp = -latComp;

        Eigen::Transform< double, 3, Eigen::Affine > Re(Eigen::AngleAxis< double >( Gamma+(M_PI/2),(Vc.cross(latComp)).normalized()));
        Eigen::Vector3d exp_normal=Re*Vc;
        Eigen::Transform< double, 3, Eigen::Affine > Ro(Eigen::AngleAxis< double >( Gamma,(Vc.cross(obs_normal)).normalized()));
        obs_normal=Ro*obs_normal;
        double tilt_par=acos(obs_normal.dot(exp_normal));
        if (V1(1)<V2(1))
          tilt_par=-tilt_par;

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

      /**
       * @brief Calculates relative position of a hexarotor if three of its markers, carried on the ends of its arms, is seen. This is used with unscented transform to express error covariance of the estimated pose from the errors of the input
       *
       * @param X The input vector containing image positions and frequencies of the markers, as well as the mean of range used to cover ambiguity of a specific sequences of points. This mean will be perturbed in the unscented transform to spread the output error covariance accordingly
       * @param expFrequencies A vector of expected frequencies belonging to the given UAV. These are used to compare to the retrieved frequencies and account for the possible error of frequency retrieval
       * @param image_index The index of the current camera used to generate the input observed marker
       *
       * @return Estimated relative position of the UAV, as well as its orientation expressed in relative Roll, Pitch and Yaw
       */
      /* uvdarHexarotorPose3p //{ */
      Eigen::VectorXd uvdarHexarotorPose3p(Eigen::VectorXd X, Eigen::VectorXd expFrequencies, int image_index){
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
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Central led: " << b);
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

        cam2world(v1, va, &(_oc_models_[image_index]));
        cam2world(v2, vb, &(_oc_models_[image_index]));
        cam2world(v3, vc, &(_oc_models_[image_index]));

        Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
        Eigen::Vector3d V2(v2[1], v2[0], -v2[2]);
        Eigen::Vector3d V3(v3[1], v3[0], -v3[2]);

        // Details of the following calculation are explained in [V Walter, N Staub, A Franchi and M Saska. "UVDAR System for Visual Relative Localization With Application to Leader–Follower Formations of Multirotor UAVs." (IEEE RA-L) July 2019] 

        Eigen::Vector3d norm13=V3.cross(V1);
        norm13=norm13/norm13.norm();
        double dist132=V2.dot(norm13);
        Eigen::Vector3d V2_c=V2-dist132*norm13;
        V2_c=V2_c/V2_c.norm();

        double Alpha = acos(V1.dot(V2_c));
        double Beta  = acos(V2_c.dot(V3));

        double A = 1.0 / tan(Alpha);
        double B = 1.0 / tan(Beta);

        double O = (A * A - A * B + sqrt(3.0) * A + B * B + sqrt(3.0) * B + 3.0);
        double delta = 2.0 * atan(((B * (2.0 * sqrt(O / (B * B + 2.0 * sqrt(3.0) + 3.0)) - 1.0)) +
              (6.0 * sqrt(O / ((sqrt(3.0) * B + 3.0) * (sqrt(3.0) * B + 3.0)))) + (2.0 * A + sqrt(3.0))) /
            (sqrt(3.0) * B + 3.0));


        double distMiddle=0.5*_arm_length_*((cos(delta)+sin(delta)*A)+(cos((M_PI*(4.0/3.0))-delta)+sin((M_PI*(4.0/3.0))-delta)*B));


        double l = sqrt(fmax(0.1, distMiddle * distMiddle + _arm_length_ * _arm_length_ - 2 * distMiddle * _arm_length_ * cos(delta + (M_PI / 3.0))));

        double Epsilon=asin((_arm_length_/l)*sin(delta+M_PI/3));

        double phi = asin(sin(delta + (CV_PI / 3.0)) * (distMiddle / l));

        double C=acos(V2_c.dot(V2));
        Eigen::Vector3d V2_d=V2_c-V2;
        if (V2_d(1)<0)
          C=-C;
        double t=acos(V1.dot(V3));

        double Omega1=asin(fmax(-1.0,fmin(1.0,(C/t)*(2.0*sqrt(3.0)))));

        Eigen::Transform< double, 3, Eigen::Affine > Rc(Eigen::AngleAxis< double >(Epsilon, norm13));
        Eigen::Vector3d Vc = Rc*V2_c;

        Eigen::VectorXd Yt=l*Vc;

        double tilt_perp=Omega1+atan2(Vc(1),sqrt(sqr(Vc(2))+sqr(Vc(0))));

        double relyaw;

        if (_debug_)
          ROS_INFO_STREAM("[UVDARPoseCalculator]: leds: " << id);

        if (expFrequencies.size() == 2){ //two frequencies on the UAV (portside and starboard)
          if (fabs(expFrequencies[1] - expFrequencies[0]) > 1.0){ //the frequencies are not the samw
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
            else{
              if (id(0)==ids[0])
                relyaw=(M_PI/2)+ambig;
              else
                relyaw=(-M_PI/2)+ambig;
            }
          }
          else {
            relyaw=2*ambig;
          }
        }
        else { //we have three frequencies, with specific arrangement
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
          else{
            if (id(0)==ids[0])
              relyaw=(M_PI/2)+ambig;
            else
              relyaw=(-M_PI/2)+ambig;
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
          ROS_INFO_STREAM("[UVDARPoseCalculator]: latComp: " << latComp);
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Vc: " << Vc);
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Gamma: " << Gamma);
          ROS_INFO_STREAM("[UVDARPoseCalculator]: exp_normal: " << exp_normal);
          ROS_INFO_STREAM("[UVDARPoseCalculator]: obs_normal: " << obs_normal);
          ROS_INFO_STREAM("[UVDARPoseCalculator]: tilt_par: " << tilt_par);
          ROS_INFO_STREAM("[UVDARPoseCalculator]: tilt_perp: " << tilt_perp);
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
          ROS_INFO_STREAM("[UVDARPoseCalculator]: tpitch: " << tpitch << " ta: " << ta << " tc: " << tc);
          ROS_INFO_STREAM("[UVDARPoseCalculator]: troll: " << tpitch << " tb: " << ta << " tc: " << tc);
          ROS_INFO_STREAM("[UVDARPoseCalculator]: reltilt_abs: " << reltilt_abs << " tilt_perp: " << tilt_perp << " tilt_par: " << tilt_par);
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Omega1: " << Omega1 << " t: " << t << " C: " << C);
        }

        Y << Yt.x(),Yt.y(),Yt.z(),troll,tpitch,relyaw;
        return Y;
      }
      //}

      /**
       * @brief Generates relative position and error covariance of a quadrotor if only one of its markers, carried on the ends of its arms or on top (beacon) , is seen.  The result is an elongated covariance covering a range of positions along the line of sight towards the marker This is not used with unscented transform
       *
       * @param X 2D vector containing the image coordinates of the sole observed marker
       * @param tubewidth Defines the "width" of the output covariance (standard deviation perpendicular to the line of sight)
       * @param tubelength Defines the "length" of the output covariance (standard deviation parallel to the line of sight)
       * @param mean_dist The mean distance of the UAV in the output measurement - with one marker we do not have informaiton on distance, but we want the mean and covariance to cover the possible range as much as possible, and avoid extending too much behind the camera
       * @param image_index The index of the current camera used to generate the input observed marker
       *
       * @return Measurement comprising mean pose and its error covariance
       */
      /* uvdarQuadrotorPose1p //{ */
      unscented::measurement uvdarQuadrotorPose1p_meas(Eigen::Vector2d X,double tubewidth, double tubelength, double mean_dist, int image_index){
        double v1[3];
        double x[2] = {X.y(),X.x()};
        cam2world(v1, x, &(_oc_models_[image_index]));
        Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
        V1 = V1*mean_dist;

        unscented::measurement ms;
        ms.x = Eigen::VectorXd(6);
        ms.C = Eigen::MatrixXd(6,6);

        ms.x << V1.x(),V1.y(),V1.z(),0,0,0; 
        Eigen::MatrixXd temp;
        temp.setIdentity(6,6);
        ms.C = temp*666;//large covariance for angles in radians - we have no information on this front
        ms.C.topLeftCorner(3, 3) = getLongCovariance(V1,tubewidth,tubelength);

        return ms;
      }
      //}

      /**
       * @brief Calculates relative position of a quadrotor without beacon if two of its markers, carried on the ends of its arms, is seen. This is used with unscented transform to express error covariance of the estimated pose from the errors of the input
       *
       * @param X The input vector containing image positions and frequencies of the markers, as well as the means of ambiguities of relative yaw and relative tilt. These means will be perturbed in the unscented transform to spread the output error covariance accordingly
       * @param expFrequencies A vector of expected frequencies belonging to the given UAV. These are used to compare to the retrieved frequencies and account for the possible error of frequency retrieval
       * @param image_index The index of the current camera used to generate the input observed marker
       *
       * @return Estimated relative position of the UAV, as well as its orientation expressed in relative Roll, Pitch and Yaw
       */
      /* uvdarQuadrotorPose2p //{ */
      Eigen::VectorXd uvdarQuadrotorPose2p(Eigen::VectorXd X, Eigen::VectorXd expFrequencies, int image_index){

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

        double      v1[3], v2[3];
        double      va[2] = {double(a.y), double(a.x)};
        double      vb[2] = {double(b.y), double(b.x)};

        if (_debug_){
          ROS_INFO_STREAM("[UVDARPoseCalculator]: a: " << a << " b: " << b);
        }
        cam2world(v1, va, &(_oc_models_[image_index]));
        cam2world(v2, vb, &(_oc_models_[image_index]));

        Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
        Eigen::Vector3d V2(v2[1], v2[0], -v2[2]);
        if (_debug_){
          ROS_INFO_STREAM("[UVDARPoseCalculator]: V1: " << V1 << " V2: " << V2);
        }

        // The following calculation is based on analytical solution of the geometrical relation between direction vectors towards two closest markers and the relative position of the UAV carrying them. The equations are equivalent to the hexarotor case in [V Walter, M Saska and A Franchi. "Fast mutual relative localization of uavs using ultraviolet led markers." (ICUAS 2018). 2018]
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

        if (_debug_){
          ROS_INFO("[UVDARPoseCalculator]: Alpha = %f, delta = %f, v = %f", Alpha, delta, v);
        }



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

        relyaw=relyaw-latang;

        double latnorm=sqrt(sqr(Yt(0))+sqr(Yt(2)));
        double Gamma=atan2(Yt(1),latnorm);
        double tilt_perp=X(7);
        Eigen::Vector3d obs_normal=V2.cross(V1);
        obs_normal=obs_normal/(obs_normal.norm());
        Eigen::Vector3d latComp;
        latComp << Vc(0),0,Vc(2);
        latComp = latComp/(latComp.norm());

        if (Vc(1)<0) latComp = -latComp;

        Eigen::Transform< double, 3, Eigen::Affine > Re(Eigen::AngleAxis< double >( Gamma+(M_PI/2),(Vc.cross(latComp)).normalized()));
        Eigen::Vector3d exp_normal=Re*Vc;
        Eigen::Transform< double, 3, Eigen::Affine > Ro(Eigen::AngleAxis< double >( Gamma,(Vc.cross(obs_normal)).normalized()));
        obs_normal=Ro*obs_normal;
        double tilt_par=acos(obs_normal.dot(exp_normal));
        if (V1(1)<V2(1))
          tilt_par=-tilt_par;

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

      /**
       * @brief Calculates relative position of a quadrotor if three of its markers, carried on the ends of its arms, is seen. This is used with unscented transform to express error covariance of the estimated pose from the errors of the input
       *
       * @param X The input vector containing image positions and frequencies of the markers, as well as the mean of range used to cover ambiguity of relative yaw. This mean will be perturbed in the unscented transform to spread the output error covariance accordingly
       * @param expFrequencies A vector of expected frequencies belonging to the given UAV. These are used to compare to the retrieved frequencies and account for the possible error of frequency retrieval
       * @param image_index The index of the current camera used to generate the input observed marker
       *
       * @return Estimated relative position of the UAV, as well as its orientation expressed in relative Roll, Pitch and Yaw
       */
      /* uvdarQuadrotorPose3p //{ */
      Eigen::VectorXd uvdarQuadrotorPose3p(Eigen::VectorXd X, Eigen::VectorXd expFrequencies, int image_index){
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
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Central led: " << b);
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

        cam2world(v1, va, &(_oc_models_[image_index]));
        cam2world(v2, vb, &(_oc_models_[image_index]));
        cam2world(v3, vc, &(_oc_models_[image_index]));

        Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
        Eigen::Vector3d V2(v2[1], v2[0], -v2[2]);
        Eigen::Vector3d V3(v3[1], v3[0], -v3[2]);

        // The following calculation is based on analytical solution of the geometrical relation between direction vectors towards three closest markers and the relative position of the UAV carrying them. The equations are equivalent to the hexarotor case in [V Walter, N Staub, A Franchi and M Saska. "UVDAR System for Visual Relative Localization With Application to Leader–Follower Formations of Multirotor UAVs." (IEEE RA-L) July 2019]
        
        Eigen::Vector3d norm13=V3.cross(V1);
        norm13=norm13/norm13.norm();
        double dist132=V2.dot(norm13);
        Eigen::Vector3d V2_c=V2-dist132*norm13;
        V2_c=V2_c/V2_c.norm();

        double Alpha = acos(V1.dot(V2_c));
        double Beta  = acos(V2_c.dot(V3));

        double A = 1.0 / tan(Alpha);
        double B = 1.0 / tan(Beta);

        double delta = atan(-(1.0+B)/(1+A))+(M_PI);

        double v = sqrt(0.5)*_arm_length_;
        double distMiddle=v*((cos(delta)+sin(delta)*A)+(cos((M_PI*1.5)-delta)+sin((M_PI*1.5)-delta)*B));


        double l = sqrt(fmax(0.1, distMiddle * distMiddle + _arm_length_ * _arm_length_ - 2 * distMiddle * _arm_length_ * cos(delta + (M_PI / 4.0))));

        double Epsilon=asin((_arm_length_/l)*sin(delta+M_PI/4.0));

        double phi = asin(sin(delta + (CV_PI/4.0)) * (distMiddle / l));

        double C=acos(V2_c.dot(V2));
        Eigen::Vector3d V2_d=V2_c-V2;
        if (V2_d(1)<0)
          C=-C;
        double t=acos(V1.dot(V3));

        double Omega1=asin(fmax(-1.0,fmin(1.0,(C/t)*(2.0*sqrt(3.0)))));

        Eigen::Transform< double, 3, Eigen::Affine > Rc(Eigen::AngleAxis< double >(Epsilon, norm13));
        Eigen::Vector3d Vc = Rc*V2_c;

        Eigen::VectorXd Yt=l*Vc;

        double tilt_perp=Omega1+atan2(Vc(1),sqrt(sqr(Vc(2))+sqr(Vc(0))));

        double relyaw = 2*ambig;;

        if (_debug_)
          ROS_INFO_STREAM("[UVDARPoseCalculator]: leds: " << id);

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
        double tilt_par=acos(obs_normal.dot(exp_normal));
        if (V1(1)<V2(1))
          tilt_par=-tilt_par;


        if (_debug_){
          ROS_INFO_STREAM("[UVDARPoseCalculator]: latComp: " << latComp);
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Vc: " << Vc);
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Gamma: " << Gamma);
          ROS_INFO_STREAM("[UVDARPoseCalculator]: exp_normal: " << exp_normal);
          ROS_INFO_STREAM("[UVDARPoseCalculator]: obs_normal: " << obs_normal);
          ROS_INFO_STREAM("[UVDARPoseCalculator]: tilt_par: " << tilt_par);
          ROS_INFO_STREAM("[UVDARPoseCalculator]: tilt_perp: " << tilt_perp);
        }

        double relyaw_view=relyaw+phi;
        relyaw=relyaw-atan2(Vc(0),Vc(2))+phi;

        double xl=(_arm_length_/2)*cos(phi);
        double dist = Yt.norm();
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
          ROS_INFO_STREAM("[UVDARPoseCalculator]: tpitch: " << tpitch << " ta: " << ta << " tc: " << tc);
          ROS_INFO_STREAM("[UVDARPoseCalculator]: troll: " << tpitch << " tb: " << ta << " tc: " << tc);
          ROS_INFO_STREAM("[UVDARPoseCalculator]: reltilt_abs: " << reltilt_abs << " tilt_perp: " << tilt_perp << " tilt_par: " << tilt_par);
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Omega1: " << Omega1 << " t: " << t << " C: " << C);
        }

        Y << Yt.x(),Yt.y(),Yt.z(),troll,tpitch,relyaw;
        return Y;
      }
      //}

      /**
       * @brief Calculates relative position of a quadrotor with beacon if one of its markers, carried on the ends of its arms, and the beacon, on the top of the UAV, are seen. This is used with unscented transform to express error covariance of the estimated pose from the errors of the input
       *
       * @param X The input vector containing image positions and frequencies of the markers, as well as the means of ambiguities of relative rotation along and perpendicular to the line of sight and of relative yaw. These means will be perturbed in the unscented transform to spread the output error covariance accordingly
       * @param expFrequencies A vector of expected frequencies belonging to the given UAV. These are used to compare to the retrieved frequencies and account for the possible error of frequency retrieval
       * @param image_index The index of the current camera used to generate the input observed marker
       *
       * @return Estimated relative position of the UAV, as well as its orientation expressed in relative Roll, Pitch and Yaw
       */
      /* uvdarQuadrotorPose2pB //{ */
      Eigen::VectorXd uvdarQuadrotorPose2pB(Eigen::VectorXd X, Eigen::VectorXd expFrequencies, int image_index){
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

        cam2world(v1, va, &(_oc_models_[image_index]));
        cam2world(v2, vb, &(_oc_models_[image_index]));

        Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
        Eigen::Vector3d V2(v2[1], v2[0], -v2[2]);
        Eigen::Vector3d DW(0, 1, 0); //downwards

        //The following calculcation is based on various geometrical considerations, since this case of observation has various ambiguities - the relative orientations of the direction vectors towards the markers depend on the distance and orientation of the target UAV.
        //They are resolved by seeding them within reasonable ranges using the unscented transform as sigma points, and solving for each sigma point as if that value was exact.
        //The output of the unscented transform will then combine the varied reults into an estimate with reasonable error covariance accounting for errors in our assumptions.
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
        if (expFrequencies.size() == 2){
          if (fabs(expFrequencies[0] - expFrequencies[0]) > 1.0)
          {
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
        return output;
      }
      //}

      /**
       * @brief Calculates relative position of a quadrotor with beacon if two of its markers, carried on the ends of its arms, and the beacon, on the top of the UAV, are seen. This is used with unscented transform to express error covariance of the estimated pose from the errors of the input
       *
       * @param X The input vector containing image positions and frequencies of the markers, as well as the mean of ambiguity of relative yaw. This mean will be perturbed in the unscented transform to spread the output error covariance accordingly
       * @param expFrequencies A vector of expected frequencies belonging to the given UAV. These are used to compare to the retrieved frequencies and account for the possible error of frequency retrieval
       * @param image_index The index of the current camera used to generate the input observed marker
       *
       * @return Estimated relative position of the UAV, as well as its orientation expressed in relative Roll, Pitch and Yaw
       */
      /* uvdarQuadrotorPose3pB //{ */
      Eigen::VectorXd uvdarQuadrotorPose3pB(Eigen::VectorXd X, Eigen::VectorXd expFrequencies, int image_index){
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
          ROS_INFO_STREAM("[UVDARPoseCalculator]: left led: " << b);
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

        cam2world(v1, va, &(_oc_models_[image_index]));
        cam2world(v2, vb, &(_oc_models_[image_index]));
        cam2world(v3, vc, &(_oc_models_[image_index]));

        Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
        Eigen::Vector3d V2(v2[1], v2[0], -v2[2]);
        Eigen::Vector3d V3(v3[1], v3[0], -v3[2]);

        //The following calculation uses P3p - based object pose estimate using the implementation by Laurent Kneip of ETH Zurich.
        //We assume that the two markers, apart from the beacon, are the closest pair on adjacent arms of the quadrotor.
        //We know the real layout of the markers and the beacon, so we can use P3p, and disambiguate the results by selecting that which provides the most distant estimate of the UAV center (it must be behind the two non-beacon markers).
        //Additionally, we discard solutions that violate some basic assumptions on the expected relative pose.

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

        double dist_max = 0;
        for (auto &candidate : output_candidates) {
          if (candidate.topRows(3).norm() > dist_max){
            dist_max = candidate.topRows(3).norm();
            output = candidate;
          }
        }

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

        return output;
      }
      //}

      /**
       * @brief Calculates relative position of a quadrotor with beacon if three of its markers, carried on the ends of its arms, and the beacon, on the top of the UAV, are seen. This is used with unscented transform to express error covariance of the estimated pose from the errors of the input
       *
       * @param X The input vector containing image positions and frequencies of the markers, as well as the mean of ambiguity of relative yaw. This mean will be perturbed in the unscented transform to spread the output error covariance accordingly
       * @param expFrequencies A vector of expected frequencies belonging to the given UAV. These are used to compare to the retrieved frequencies and account for the possible error of frequency retrieval
       * @param image_index The index of the current camera used to generate the input observed marker
       *
       * @return Estimated relative position of the UAV, as well as its orientation expressed in relative Roll, Pitch and Yaw
       */
      /* uvdarQuadrotorPose4pB //{ */
      Eigen::VectorXd uvdarQuadrotorPose4pB(Eigen::VectorXd X, Eigen::VectorXd expFrequencies, int image_index){
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
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Central led: " << c);
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

        cam2world(v1, va, &(_oc_models_[image_index]));
        cam2world(v2, vb, &(_oc_models_[image_index]));
        cam2world(v3, vc, &(_oc_models_[image_index]));
        cam2world(v4, vd, &(_oc_models_[image_index]));

        Eigen::Vector3d V1(v1[1], v1[0], -v1[2]);
        Eigen::Vector3d V2(v2[1], v2[0], -v2[2]);
        Eigen::Vector3d V3(v3[1], v3[0], -v3[2]);
        Eigen::Vector3d V4(v4[1], v4[0], -v4[2]);

        //The following calculation uses P3p - based object pose estimate using the implementation by Laurent Kneip of ETH Zurich.
        //Of the three markers, apart from the beacon, we only use the two that are not center. The center marker, due to its typical position close to the line connecting the two others, will not add significantly more precision in estimation.
        //We deem it more likely that taking the center marker into account increases the chances of random errors due to matching with the incorrect object point.
        //We assume that the two marker, are a pair placed on opposing arms of the quadrotor.
        //We know the real layout of the markers and the beacon, so we can use P3p, and disambiguate the results by selecting that which provides the most distant estimate of the UAV center (exploiting foreshortening).
        //Additionally, we discard solutions that violate some basic assumptions on the expected relative pose.
        
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
            e::Vector3d object_position= -cam_rotations[i]*cam_centers[i];
            e::Vector3d object_position_cam  = camToBase.transpose()*object_position;
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

        double dist_max = 0;
        for (auto &candidate : output_candidates) {
          if (candidate.topRows(3).norm() > dist_max){
            dist_max = candidate.topRows(3).norm();
            output = candidate;
          }
        }
        double relyaw = ambig;

        if (_debug_)
          ROS_INFO_STREAM("[UVDARPoseCalculator]: leds: " << id);

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

        output[5] += relyaw;

        if (_debug_){
          ROS_INFO_STREAM("["<< ros::this_node::getName().c_str() << "]: 4p:  output final: " << output.transpose());
        }
        return output;
      }
      //}
      
      //}


      /**
       * @brief Rotates a 3D covariance 
       *
       * @param covariance The input 3x3 covariance matrix to rotate
       * @param rotation The rotation matrix to apply
       *
       * @return Rotated 3x3 covariance matrix
       */
      /* rotateCovariance //{ */
      static Eigen::Matrix3d rotateCovariance(const Eigen::Matrix3d& covariance, const Eigen::Matrix3d& rotation) {
        return rotation * covariance * rotation.transpose();  // rotate the covariance to point in direction of est. position
      }
      //}

      /**
       * @brief Generates an elongated covariance, with the primary eigenvector parallel to posiiton_sf. This is used to generate covariance for target measurement composed of only a single marker observation that provides no intrinsic information on the distance
       *
       * @param position_sf Vector defining the direction of the long covariance - parallel to one of the output covariance eigenvectors
       * @param xy_covariance_coeff The standard deviation of the covariance perpendicular to position_sf - defines the "thickness" of the covariance
       * @param z_covariance_coeff The standard deviation of the covariance parallel to position_sf - defines the "length" of the covariance
       *
       * @return 
       */
      /* getLongCovariance //{ */
      static Eigen::Matrix3d getLongCovariance(const Eigen::Vector3d& position_sf, const double xy_covariance_coeff, const double z_covariance_coeff) {
        Eigen::Matrix3d v_x;
        Eigen::Vector3d b;
        Eigen::Vector3d v;
        double sin_ab, cos_ab;
        Eigen::Matrix3d vec_rot;
        const Eigen::Vector3d a(0.0, 0.0, 1.0);
        Eigen::Matrix3d pos_cov = Eigen::Matrix3d::Identity();  // prepare the covariance matrix
        pos_cov(0, 0) = pos_cov(1, 1) = xy_covariance_coeff*xy_covariance_coeff;
        pos_cov(2, 2) = z_covariance_coeff;

        // Find the rotation matrix to rotate the covariance to point in the direction of the estimated position
        b = position_sf.normalized();
        v = a.cross(b);
        sin_ab = v.norm();
        cos_ab = a.dot(b);
        const double angle = atan2(sin_ab, cos_ab);     // the desired rotation angle
        vec_rot = Eigen::AngleAxisd(angle, v.normalized()).toRotationMatrix();
        pos_cov = rotateCovariance(pos_cov, vec_rot);  // rotate the covariance to point in direction of est. position
        if (pos_cov.array().isNaN().any()){
          ROS_INFO_STREAM("[UVDARPoseCalculator]: NaN in long covariance!!!!");
        }
        return pos_cov;
      }
      //}

      /**
       * @brief Separate set of input points into groups presumed to belong each to an individual UAV. Here, the separation is done primarily based on blinking frequencies. Each UAV has its own set of frequencies. If multiple distant clusters of frequencies associated with the same UAV are found, they are returned with IDs increased by 1000 for each subsequent cluster
       *
       * @param points A set of points, where the X and Y coordinates correspond to their image positions and Z corresponds to their blinking frequencies
       *
       * @return A set of separated points sets, each accompanied by a unique integer identifier. The idientifier is equal to TID + 1000*CL where TID is the id of the UAV associated with frequencies of the markers and CL is the order number of the current cluster starting with 0. Ordinarilly, there should be only one cluster per target.
       */
      /* separateByFrequency //{ */
      std::vector<std::pair<int,std::vector<cv::Point3d>>> separateByFrequency(std::vector< cv::Point3d > points){
        std::vector<std::pair<int,std::vector<cv::Point3d>>> separated_points;
        separated_points.resize(_target_count_);


        for (int i = 0; i < (int)(points.size()); i++) {
          if (points[i].z > 1) {
            int mid = ufc_->findMatch(points[i].z);
            int tid = classifyMatch(mid);
            if (_debug_)
              ROS_INFO("[%s]: FR: %d, MID: %d, TID: %d", ros::this_node::getName().c_str(),(int)points[i].z, mid, tid);
            /* separated_points[classifyMatch(findMatch(points[i].z))].push_back(points[i]); */
            if (tid>=0){
              separated_points[tid].first = tid;
              separated_points[tid].second.push_back(points[i]);
            }
          }
        }


        auto fullAverage = [] (std::vector<cv::Point3d> points) { 
          cv::Point2d sum(0,0);
          for (auto p : points){
            sum += cv::Point2d(p.x,p.y);
          }
          sum /= (double)(points.size());
          return sum;
        };

        for (int s = 0; s < (int)(separated_points.size()); s++){
          std::vector<std::vector<cv::Point3d>> clusters; // subsets of the frequency-separated split apart by distance
          std::vector<cv::Point2d> cluster_centroids;
          for (int i=0; i<(int)(separated_points[s].second.size());i++){
            bool cluster_found = false;
            for (int j=0; j<(int)(clusters.size());j++){
              if ( cv::norm((cv::Point2d(separated_points[s].second[i].x,separated_points[s].second[i].y) - cluster_centroids[j])) < MAX_DIST_INIT){
                clusters[j].push_back(separated_points[s].second[i]);
                cluster_centroids.push_back(fullAverage(clusters[j]));
                cluster_found = true;
              }
            }
            if (!cluster_found){
              std::vector<cv::Point3d> new_cluster;
              new_cluster.push_back(separated_points[s].second[i]);
              clusters.push_back(new_cluster);
              cv::Point2d init_cluster_pt_2d(separated_points[s].second[i].x, separated_points[s].second[i].y);
              cluster_centroids.push_back(init_cluster_pt_2d);
            }
          }
          if (clusters.size() > 1){
            int tid_orig = separated_points[s].first;
            separated_points.erase(separated_points.begin()+s);

            auto it = separated_points.begin()+s;
            int cl_n = 0;
            for (auto& cluster : clusters){
              separated_points.insert (it+cl_n,1,{cl_n*1000+tid_orig,cluster});
              cl_n++;
            }
            s+=cl_n++;
          }
        }

        return separated_points;
      }
      //}


      /**
       * @brief Returns a set of markers associated with the given beacon. These are selected such that they are below the marker roughly in horizontal line, and are the closest set to the beacon if there are multiple such compliant sets.
       *
       * @param beacon The beacon for which we are requesting other markers associated to the same UAV
       * @param points The set of points from which we are selecting the output set
       * @param marked_points A set of flags for each input point, marking them as already associated with a beacon - this allows us to progressively collect even partially ovelapping clusters
       *
       * @return Set of markers associated with the given beacon, paired with index of the searching bracket in which they were found. The bracket index tells us how far are they from the beacon, and if any were found (index = -1)
       */
      /* getClosestSet //{ */
      std::pair<int,std::vector<int>> getClosestSet(cv::Point3d &beacon, std::vector<cv::Point3d> &points, std::vector<bool> &marked_points){
        std::pair<int, std::vector<int>> output;
        output.first = -1;
        int b = 0;
        for (auto &bracket : bracket_set){
          cv::Rect bracket_placed(bracket.tl()+cv::Point(beacon.x,beacon.y), bracket.size());
          std::vector<int> curr_selected_points;

          int i = -1;
          for (auto &point : points){
            i++;
            if (marked_points[i] == true){
              continue;
            }

            if (bracket_placed.contains(cv::Point2i(point.x, point.y))){
              cv::Rect local_bracket(cv::Point2i(bracket_placed.tl().x,point.y-BRACKET_STEP), cv::Point2i(bracket_placed.br().x,point.y+BRACKET_STEP));
              int j = -1;
              for (auto &point_inner : points){
                j++;
                if (marked_points[j] == true){
                  continue;
                }
                if (local_bracket.contains(cv::Point2i(point_inner.x, point_inner.y))){
                  curr_selected_points.push_back(j);
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


      /**
       * @brief Separate set of input points into groups presumed to belong each to an individual UAV. Here, the separation is done primarily based on observed beacon markers on top of the UAVs. The beacons have specific frequency, different from the other markers on the UAVs.
       *
       * @param points A set of points, where the X and Y coordinates correspond to their image positions and Z corresponds to their blinking frequencies
       *
       * @return A set of separated points sets, each accompanied by a unique integer identifier
       */
      /* separateByBeacon //{ */
      std::vector<std::pair<int,std::vector<cv::Point3d>>> separateByBeacon(std::vector< cv::Point3d > points){
        std::vector<std::pair<int,std::vector<cv::Point3d>>> separated_points;

        std::vector<bool> marked_points(points.size(), false);
        std::vector<int> midPoints(points.size(), -1);

        std::vector<cv::Point3d> emptySet;

        for (int i = 0; i < (int)(points.size()); i++) {
          if (points[i].z > 1) {
            int mid = ufc_->findMatch(points[i].z);
            midPoints[i] = mid;
            if (mid == 0){
              separated_points.push_back({(int)(separated_points.size()),emptySet});
              separated_points.back().second.push_back(points[i]);
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
            auto curr_beacon = curr_set.second[0];
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
            separated_points[best_index].second.push_back(points[subset_point_index]);
            marked_points[subset_point_index] = true;

          }
          marked_beacons[best_index] = true;

        }

        int i=-1;
        for (auto &point: points){
          i++;
          if (marked_points[i])
            continue;

          separated_points.push_back({(int)(separated_points.size()),emptySet});
          separated_points.back().second.push_back(cv::Point3d(-1,-1,-1)); //this means that this set does not have a beacon
          separated_points.back().second.push_back(points[i]);
          marked_points[i] = true;
          for (int j = i+1; j < (int)(points.size()); j++) {
            if (marked_points[j])
              continue;
            if (cv::norm(point - points[j]) < MAX_DIST_INIT){
              separated_points.back().second.push_back(points[j]);
              marked_points[j] = true;
            }
          }
        }

        return separated_points;
      }

      //}


      /**
       * @brief Calculates a pose with error covariance of a UAV observed as a set of its blinking markers. This heavily exploits the approach for accounting for input errors and ambiguity ranges using the unscented transform shown in [V Walter, M Vrba and M Saska. "On training datasets for machine learning-based visual relative localization of micro-scale UAVs" (ICRA 2020). 2020].
       *
       * @param points The set of observed blinking markers in the image space of the current camera
       * @param target The index of the current target UAV
       * @param image_index The index of the current camera observing the UAV
       * @param output_pose The output estimated pose with covariance, encapsulated in a ros message. Also includes the target index
       */
      /* extractSingleRelative //{ */
      void extractSingleRelative(std::vector< cv::Point3d > points, int target, size_t image_index, mrs_msgs::PoseWithCovarianceIdentified& output_pose) {

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
        double          max_dist = MAX_DIST_INIT;

        int countSeen = (int)(points.size());

        bool missing_beacon = (_beacon_)&&(points[0].x == -1);

        if (missing_beacon){
          points.erase(points.begin());
        }

        if ((!_beacon_) || (missing_beacon)){

          //Remove furthest markers until there are at most 3 in the current set
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

                  if (_debug_)
                    ROS_INFO_STREAM("[UVDARPoseCalculator]: Distance: " <<cv::norm(cv::Point2i(points[i].x,points[i].y) - cv::Point2i(points[j].x,points[j].y)) << " max_dist: " << max_dist);

                  if ((cv::norm(cv::Point2i(points[i].x,points[i].y) - cv::Point2i(points[j].x,points[j].y)) < max_dist)) {
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
              max_dist = 0.5 * max_dist;
            }
          }
        }


        unscented::measurement ms;

        if (_debug_){
          ROS_INFO_STREAM("[UVDARPoseCalculator]: framerateEstim: " << estimated_framerate_[image_index]);
        }

        double perr=0.2/estimated_framerate_[image_index]; // The expected error of the frequency estimate (depends on the sampling frequency / camera framerate)

        if (_quadrotor_) { //if we are expecting quadrotors (markers on each arm)
          if ((_beacon_) && (!missing_beacon)){ //if we are using beacons and we see the beacon of the current UAV
            if (points.size() == 1) { //only the beacon is visible
              ROS_INFO_THROTTLE(1.0,"[%s]: Only one beacon visible - no distance information", ros::this_node::getName().c_str());
              if (_debug_)
                std::cout << "led: " << points[0] << std::endl;

              ms = uvdarQuadrotorPose1p_meas(Eigen::Vector2d(points[0].x,points[0].y),_arm_length_, 1000,10.0, image_index);
            }
            else if (points.size() == 2){ //beacon and one other marker is visible
              if (_debug_)
                ROS_INFO_STREAM("[UVDARPoseCalculator]: points: " << points);
              X2qb <<
                points[0].x ,points[0].y, // presume that the beacon really is a beacon
                points[1].x ,points[1].y, 1.0/(double)(points[1].z),
                0,  //to account for ambiguity (roll)
                0,  //to account for ambiguity (pitch)
                0;  //ambiguity - which of the two markers of a given ID is it, or multiplied by 4 if all markers of the UAV (apart from the beacon) have the same ID
              Px2qb <<
                QPIX,0,     0,   0,   0,         0,            0,           0,
                0,   QPIX,  0,   0,   0,         0,            0,           0,
                0,   0,     QPIX,0,   0,         0,            0,           0,
                0,   0,     0,   QPIX,0,         0,            0,           0,
                0,   0,     0,   0,   sqr(perr), 0,            0,           0,
                0,   0,     0,   0,   0,         sqr(M_PI/36), 0,           0,          //tilt_par
                0,   0,     0,   0,   0,         0,           sqr(M_PI/18),   0,        //tilt_perp
                0,   0,     0,   0,   0,         0,            0,           sqr(M_PI_2) //ambig
                  ;
              if (_debug_)
                ROS_INFO_STREAM("[UVDARPoseCalculator]: X2qb: " << X2qb);
              boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd,int)> callback;
              callback=boost::bind(&UVDARPoseCalculator::uvdarQuadrotorPose2pB,this,_1,_2,_3);
              ms = unscented::unscentedTransform(X2qb,Px2qb,callback,leftF,rightF,-1,image_index);
            }
            else if (points.size() == 3){ //beacon and two other markers are visible
              if (_debug_)
                ROS_INFO_STREAM("[UVDARPoseCalculator]: points: " << points);
              X3qb <<
                points[0].x ,points[0].y, // presume that the beacon really is a beacon
                points[1].x ,points[1].y, 1.0/(double)(points[1].z),
                points[2].x ,points[2].y, 1.0/(double)(points[2].z),
                0; // ambiguity - if the UAV has only one frequency for all markers except for the beacon, we don't know the orientation
              Px3qb <<
                QPIX,0,    0,   0,   0,        0,   0,   0,         0,
                0,   QPIX, 0,   0,   0,        0,   0,   0,         0,
                0,   0,    QPIX,0,   0,        0,   0,   0,         0,   
                0,   0,    0,   QPIX,0,        0,   0,   0,         0,  
                0,   0,    0,   0,   sqr(perr),0,   0,   0,         0,    
                0,   0,    0,   0,   0,        QPIX,0,   0,         0,     
                0,   0,    0,   0,   0,        0,   QPIX,0,         0,      
                0,   0,    0,   0,   0,        0,   0,   sqr(perr), 0,
                0,   0,    0,   0,   0,        0,   0,   0,         sqr(M_PI*2) //ambig
                  ;
              if (_debug_)
                ROS_INFO_STREAM("[UVDARPoseCalculator]: X3qb: " << X3qb);
              boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd,int)> callback;
              callback=boost::bind(&UVDARPoseCalculator::uvdarQuadrotorPose3pB,this,_1,_2,_3);
              ms = unscented::unscentedTransform(X3qb,Px3qb,callback,leftF,rightF,-1,image_index);
            }
            else if (points.size() == 4){ //beacon and three other markers are visible
              if (_debug_)
                ROS_INFO_STREAM("[UVDARPoseCalculator]: points: " << points);
              X4qb <<
                points[0].x ,points[0].y, // presume that the beacon really is a beacon
                points[1].x ,points[1].y, 1.0/(double)(points[1].z),
                points[2].x ,points[2].y, 1.0/(double)(points[2].z),
                points[3].x ,points[3].y, 1.0/(double)(points[3].z),
                0; //ambiguity - if markers (apart from beacon) have only one frequency, the orientation is ambiguous. Alternatively multiply by 2 if there are two frequencies in an unexpected sequence
              Px4qb <<
                QPIX,0,   0,   0,   0,        0,   0,   0,         0,   0,    0,         0,
                0,   QPIX,0,   0,   0,        0,   0,   0,         0,   0,    0,         0,
                0,   0,   QPIX,0,   0,        0,   0,   0,         0,   0,    0,         0,   
                0,   0,   0,   QPIX,0,        0,   0,   0,         0,   0,    0,         0,  
                0,   0,   0,   0,   sqr(perr),0,   0,   0,         0,   0,    0,         0,    
                0,   0,   0,   0,   0,        QPIX,0,   0,         0,   0,    0,         0,     
                0,   0,   0,   0,   0,        0,   QPIX,0,         0,   0,    0,         0,      
                0,   0,   0,   0,   0,        0,   0,   sqr(perr), 0,   0,    0,         0,
                0,   0,   0,   0,   0,        0,   0,   0,         QPIX,0,    0,         0, 
                0,   0,   0,   0,   0,        0,   0,   0,         0,   QPIX, 0,         0, 
                0,   0,   0,   0,   0,        0,   0,   0,         0,   0,    sqr(perr), 0,
                0,   0,   0,   0,   0,        0,   0,   0,         0,   0,    0,         sqr(2*M_PI/3)
                  ;
              if (_debug_)
                ROS_INFO_STREAM("[UVDARPoseCalculator]: X4qb: " << X4qb);
              boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd,int)> callback;
              callback=boost::bind(&UVDARPoseCalculator::uvdarQuadrotorPose4pB,this,_1,_2,_3);
              ms = unscented::unscentedTransform(X4qb,Px4qb,callback,leftF,rightF,-1,image_index);
            }
            else { // No points seen - this section should not be reached, since it implie missing_beacon = true
              ROS_INFO_THROTTLE(1.0,"[%s]: No valid points seen. Waiting", ros::this_node::getName().c_str());
              return;
            }
          }
          else { //if we are not using beacons, or we are and we do not see the beacon of the current UAV
            if (points.size() == 3) { // three markers are visible
              if (_debug_)
                ROS_INFO_STREAM("[UVDARPoseCalculator]: points: " << points);
              X3 <<
                points[0].x ,points[0].y, 1.0/(double)(points[0].z),
                points[1].x ,points[1].y, 1.0/(double)(points[1].z),
                points[2].x, points[2].y, 1.0/(double)(points[2].z),
                0; //ambiguity - if markers have only one frequency, the orientation is ambiguous. Alternatively multiply by 2 if there are two frequencies in an unexpected sequence
              Px3 <<
                QPIX,0,   0,        0,   0,   0,        0,   0,   0,        0,
                0,   QPIX,0,        0,   0,   0,        0,   0,   0,        0,
                0,   0,   sqr(perr),0,   0,   0,        0,   0,   0,        0,
                0,   0,   0,        QPIX,0,   0,        0,   0,   0,        0,
                0,   0,   0,        0,   QPIX,0,        0,   0,   0,        0,
                0,   0,   0,        0,   0,   sqr(perr),0,   0,   0,        0,
                0,   0,   0,        0,   0,   0,        QPIX,0,   0,        0,
                0,   0,   0,        0,   0,   0,        0,   QPIX,0,        0,
                0,   0,   0,        0,   0,   0,        0,   0,   sqr(perr),0,
                0,   0,   0,        0,   0,   0,        0,   0,   0,        sqr(2*M_PI/3)
                  ;
              if (_debug_)
                ROS_INFO_STREAM("[UVDARPoseCalculator]: X3: " << X3);
              boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd,int)> callback;
              callback=boost::bind(&UVDARPoseCalculator::uvdarQuadrotorPose3p,this,_1,_2,_3);
              ms = unscented::unscentedTransform(X3,Px3,callback,leftF,rightF,-1,image_index);
            }
            else if (points.size() == 2) { //two markers are visible
              if (_debug_)
                ROS_INFO_STREAM("[UVDARPoseCalculator]: points: " << points);
              X2q <<
                (double)(points[0].x) ,(double)(points[0].y),1.0/(double)(points[0].z),
                (double)(points[1].x) ,(double)(points[1].y),1.0/(double)(points[1].z),
                0, // ambiguity of the relative yaw - the distance between two markers in the image is also affected by the orientation of the target
                0; // ambiguity of the relative tilt - the two closest markers (on the arms) do not tell us how much is the UAV tilted towards or away from the camera
              Px2q <<
                QPIX ,0,0,0,0,0,0,0,
                     0,QPIX ,0,0,0,0,0,0,
                     0,0,sqr(perr),0,0,0,0,0,
                     0,0,0,QPIX ,0,0,0,0,
                     0,0,0,0,QPIX ,0,0,0,
                     0,0,0,0,0,sqr(perr),0,0,
                     0,0,0,0,0,0,sqr(deg2rad((missing_beacon?70:10))),0,  //the range si normally small, since the markers have to be aimed roughly at the camera to be visible
                     0,0,0,0,0,0,0,sqr(deg2rad(10)) //limited by realistic flight requirements - we normally don't tilt too much
                       ;
              if (_debug_)
                ROS_INFO_STREAM("[UVDARPoseCalculator]: X2: " << X2q);
              boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd, int)> callback;
              callback=boost::bind(&UVDARPoseCalculator::uvdarQuadrotorPose2p,this,_1,_2,_3);
              ms = unscented::unscentedTransform(X2q,Px2q,callback,leftF,rightF,-1,image_index);
            }
            else if (points.size() == 1) { // only single marker is visible that is not the beacon
              ROS_INFO_THROTTLE(1.0,"[%s]: Only single point visible - no distance information", ros::this_node::getName().c_str());
              if (_debug_)
                std::cout << "led: " << points[0] << std::endl;


              ms = uvdarQuadrotorPose1p_meas(Eigen::Vector2d(points[0].x,points[0].y),_arm_length_, 1000,10.0, image_index);


            } else { //no markers seen by the current camera
              ROS_INFO_THROTTLE(1.0,"[%s]: No valid points seen. Waiting", ros::this_node::getName().c_str());
              return;
            }

          }
        }
        else { //if we are expecting hexarotors (markers on each arm)
          if (_beacon_) { //not implemented - our current fleet of swarm UAVs is comprised only of quadrotors
            ROS_WARN_THROTTLE(1.0,"[%s]: Beacon-based estimation for hexarotors is not implemented!", ros::this_node::getName().c_str());
          }

          if (points.size() == 3) { //three markers are visible 
            if (_debug_)
              ROS_INFO_STREAM("[UVDARPoseCalculator]: points: " << points);
            X3 <<
              points[0].x ,points[0].y, 1.0/(double)(points[0].z),
              points[1].x ,points[1].y, 1.0/(double)(points[1].z),
              points[2].x, points[2].y, 1.0/(double)(points[2].z),
              0;  //ambiguity - if the three markers have unexpected sequence of frequencies, or if the frequencies are the same on all sides multiplied by 2 (orientation is unknown)
            Px3 <<
              QPIX ,0,0,0,0,0,0,0,0,0,
                   0,QPIX ,0,0,0,0,0,0,0,0,
                   0,0,sqr(perr),0,0,0,0,0,0,0,
                   0,0,0,QPIX ,0,0,0,0,0,0,
                   0,0,0,0,QPIX ,0,0,0,0,0,
                   0,0,0,0,0,sqr(perr),0,0,0,0,
                   0,0,0,0,0,0,QPIX ,0,0,0,
                   0,0,0,0,0,0,0,QPIX ,0,0,
                   0,0,0,0,0,0,0,0,sqr(perr),0,
                   0,0,0,0,0,0,0,0,0,sqr(2*M_PI/3)
                     ;
            if (_debug_)
              ROS_INFO_STREAM("[UVDARPoseCalculator]: X3: " << X3);
            boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd, int)> callback;
            callback=boost::bind(&UVDARPoseCalculator::uvdarHexarotorPose3p,this,_1,_2,_3);
            ms = unscented::unscentedTransform(X3,Px3,callback,leftF,rightF,-1,image_index);
          }
          else if (points.size() == 2) {
            if (_debug_)
              ROS_INFO_STREAM("[UVDARPoseCalculator]: points: " << points);
            X2 <<
              (double)(points[0].x) ,(double)(points[0].y),1.0/(double)(points[0].z),
              (double)(points[1].x) ,(double)(points[1].y),1.0/(double)(points[1].z),
              0, // ambiguity of the relative yaw - the distance between two markers in the image is also affected by the orientation of the target
              0, // ambiguity of the pair of markers seen - if we have two frequencies on each side of a hexarotor (three markers adjacent per frequency), we can see adjacent pair of equal frequency which can correspond to one of two pairs or arms
              0; // ambiguity of the relative tilt - the two closest markers (on the arms) do not tell us how much is the UAV tilted towards or away from the camera
            Px2 <<
              QPIX ,0,0,0,0,0,0,0,0,
                   0,QPIX ,0,0,0,0,0,0,0,
                   0,0,sqr(perr),0,0,0,0,0,0,
                   0,0,0,QPIX ,0,0,0,0,0,
                   0,0,0,0,QPIX ,0,0,0,0,
                   0,0,0,0,0,sqr(perr),0,0,0,
                   0,0,0,0,0,0,sqr(deg2rad(15)),0,0,
                   0,0,0,0,0,0,0,sqr(deg2rad(60)),0,
                   0,0,0,0,0,0,0,0,sqr(deg2rad(10))
                     ;

            if (_debug_)
              ROS_INFO_STREAM("[UVDARPoseCalculator]: X2: " << X2);
            boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd, int)> callback;
            callback=boost::bind(&UVDARPoseCalculator::uvdarHexarotorPose2p,this,_1,_2,_3);
            ms = unscented::unscentedTransform(X2,Px2,callback,leftF,rightF,-1,image_index);
          }
          else if (points.size() == 1) { //only single marker is visible
            std::cout << "Only single point visible - no distance information" << std::endl;

            ms = uvdarHexarotorPose1p_meas(Eigen::Vector2d(points[0].x,points[0].y),_arm_length_, (TUBE_LENGTH),10.0, image_index);

          } else { //no markers seen by the current camera
            std::cout << "No valid points seen. Waiting" << std::endl;
            return;
          }
        }


        ms.C += e::MatrixXd::Identity(6,6)*0.0001;
        if (_debug_){
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Y: \n" << ms.x );
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Py: \n" << ms.C );
        }

        if (ms.x.topLeftCorner(3,1).norm() < 1.5) { //We don't expect to be able to measure relative poses of UAVs this close - the markers would be too bright and too far apart
          return;
        }

        e::Vector3d unit_vec;
        unit_vec << 0,0,1.0;
        if (acos(ms.x.topLeftCorner(3,1).normalized().dot(unit_vec)) > rad2deg(190)) //our lenses only allow us to see UAVs ~92.5 degrees away from the optical axis of the camera
          return;

        tf::Quaternion qtemp;
        qtemp.setRPY((ms.x(3)), (ms.x(4)), (ms.x(5)));
        qtemp=tf::Quaternion(-0.5,0.5,-0.5,-0.5)*qtemp; //bring relative orientations to the optical frame of the camera (Roll, Pitch and Yaw were estimated in the more intuitive sensor frame (X forward, Y to the left, Z up)
        qtemp.normalize();//just in case

        output_pose.id = target;
        output_pose.pose.position.x = ms.x(0);
        output_pose.pose.position.y = ms.x(1);
        output_pose.pose.position.z = ms.x(2);
        output_pose.pose.orientation.x = qtemp.x();
        output_pose.pose.orientation.y = qtemp.y();
        output_pose.pose.orientation.z = qtemp.z();
        output_pose.pose.orientation.w = qtemp.w();
        for (int i=0; i<ms.C.cols(); i++){
          for (int j=0; j<ms.C.rows(); j++){
            output_pose.covariance[ms.C.cols()*j+i] =  ms.C(j,i);
          }
        }

      }
      //}

      /**
       * @brief Returns the index of target UAV with a marker based on the frequency-based ID of that marker
       *
       * @param f_i Index of the frequency of the given marker
       *
       * @return Index of the target carrying the marker
       */
      /* classifyMatch //{ */
      int classifyMatch(int f_i) {
        return f_i/_frequencies_per_target_;
      }
      //}


      /**
       * @brief Prepares structures needed for collecting markers associated with a specific beacon marker
       */
      /* prepareBlinkerBrackets //{ */
      void prepareBlinkerBrackets() {
        double frame_ratio = _arm_length_/(_beacon_height_*0.75);
        double max_dist = 100;

        cv::Rect curr_rect;
        for (int i = 0; i < max_dist/BRACKET_STEP; i++) {
          curr_rect = cv::Rect(cv::Point2i(-frame_ratio*i*BRACKET_STEP-5, 0), cv::Point(frame_ratio*i*BRACKET_STEP+5,i*BRACKET_STEP+5));
          bracket_set.push_back(curr_rect);
        }
      }
      //}

      /**
       * @brief Thread function for optional visualization of separated markers
       *
       * @param te TimerEvent for the timer spinning this thread
       */
      /* VisualizationThread() //{ */
      void VisualizationThread([[maybe_unused]] const ros::TimerEvent& te) {
        if (initialized_){
          /* std::scoped_lock lock(mutex_visualization_); */
          if(generateVisualization(image_visualization_) >= 0){
            if ((image_visualization_.cols != 0) && (image_visualization_.rows != 0)){
              if (_publish_visualization_){
                pub_visualization_->publish("ocv_point_separation", 0.01, image_visualization_, true);
              }
              if (_gui_){
                cv::imshow("ocv_point_separation_" + _uav_name_, image_visualization_);
                cv::waitKey(25);
              }
            }
          }
        }
      }
      //}

      /**
       * @brief Method for generating annotated image for optional visualization
       *
       * @param output_image The generated visualization image
       *
       * @return Success status ( 0 - success, 1 - visualization does not need to be generated as the state has not changed, negative - failed, usually due to missing requirements
       */
      /* generateVisualization() //{ */
      int generateVisualization(cv::Mat& output_image) {

        if (camera_image_sizes_.size() == 0){
          return -3;
        }

        int max_image_height = 0;
        int sum_image_width = 0;
        std::vector<int> start_widths;
        for (auto curr_size : camera_image_sizes_){
          if (max_image_height < curr_size.height){
            max_image_height = curr_size.height;
          }
          start_widths.push_back(sum_image_width);
          sum_image_width += curr_size.width;
        }

        if ( (sum_image_width <= 0) || (max_image_height <= 0) ){
          return -2;
        }

        output_image = cv::Mat(cv::Size(sum_image_width+((int)(camera_image_sizes_.size())-1), max_image_height),CV_8UC3);
        output_image = cv::Scalar(0,0,0);

        if ( (output_image.cols <= 0) || (output_image.rows <= 0) ){
          return -1;
        }

        int image_index = 0;
        for (auto &sep_points_image : separated_points_){
          std::scoped_lock lock(*(mutex_separated_points_[image_index]));
          cv::Point start_point = cv::Point(start_widths[image_index]+image_index, 0);
          if (image_index > 0){
            cv::line(output_image, start_point+cv::Point2i(-1,0), start_point+cv::Point2i(-1,max_image_height-1),cv::Scalar(255,255,255));
          }

          for (auto &point_group : sep_points_image){
            for (auto &point : point_group.second){
              cv::Point center = start_point + cv::Point2i(point.x,point.y);

              cv::Scalar color = ColorSelector::markerColor(point_group.first);
              cv::circle(output_image, center, 5, color);
            }
          }
          image_index++;
        }

        return 0;
      }

      //}


      /* Conversions of rotation matrices to aviation angles //{ */
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
      //}

      /* attributes //{ */
      bool _debug_;

      std::string _uav_name_;


      using blinkers_seen_callback_t = boost::function<void (const mrs_msgs::ImagePointsWithFloatStampedConstPtr& msg)>;
      std::vector<blinkers_seen_callback_t> cals_blinkers_seen_;
      std::vector<ros::Subscriber> sub_blinkers_seen_;
      ros::Time last_blink_time_;

      using estimated_framerate_callback_t = boost::function<void (const std_msgs::Float32ConstPtr& msg)>;
      std::vector<estimated_framerate_callback_t> cals_estimated_framerate_;
      std::vector<ros::Subscriber> sub_estimated_framerate_;
      std::vector<double> estimated_framerate_;

      std::vector<std::string> _camera_frames_;
      unsigned int _camera_count_;


      bool _gui_;
      bool _publish_visualization_;
      ros::Timer timer_visualization_;
      std::vector<cv::Size> camera_image_sizes_;
      cv::Mat image_visualization_;
      std::unique_ptr<mrs_lib::ImagePublisher> pub_visualization_;


      std::vector<struct ocam_model> _oc_models_;


      Eigen::MatrixXd Px2,Px3,Px2q;
      Eigen::MatrixXd Px2qb,Px3qb,Px4qb;
      Eigen::VectorXd X2,X3,X2q;
      Eigen::VectorXd X2qb,X3qb,X4qb;

      std::vector<ros::Publisher> pub_measured_poses_;

      std::vector<double> _frequencies_;
      int _frequencies_per_target_;
      int _target_count_;
      std::unique_ptr<UVDARFrequencyClassifier> ufc_;


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

      std::vector<std::shared_ptr<std::mutex>>  mutex_separated_points_;
      std::vector<std::vector<std::pair<int,std::vector<cv::Point3d>>>> separated_points_;
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

