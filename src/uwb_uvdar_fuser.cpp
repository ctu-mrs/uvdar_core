#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

#include <mrs_lib/dkf.h>

#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/Vec4.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <uvdar_core/ImagePointsWithFloatStamped.h> 

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

/* #include <boost/range/adaptor/indexed.hpp> */ 

#include "OCamCalib/ocam_functions.h"

#include <mutex>

#include <Eigen/Dense>

#define sqr(X) ((X) * (X))

#define DEFAULT_OUTPUT_FRAMERATE 20.0
#define DECAY_AGE_NORMAL 3.0
#define DECAY_AGE_UNVALIDATED 1.0
#define MIN_MEASUREMENTS_TO_VALIDATION 10
#define POS_THRESH 2.0
#define MAH_THRESH 2.0
#define YAW_THRESH 1.5
#define MATCH_LEVEL_THRESHOLD_ASSOCIATE 0.3
#define MATCH_LEVEL_THRESHOLD_REMOVE 0.5

#define LINE_VARIANCE 1 //meters^2
#define RANGE_MIN 1.0 //meters
#define RANGE_MAX 50.0 //meters

/* using namespace boost::adaptors; */

namespace mrs_lib
{
  /* const int n_states = -1; */
  const int n_states = 6;
  const int n_inputs = 0;
  /* const int n_measurements = 6; */
  /* using dkf_t = LKF<n_states, n_inputs, n_measurements>; */
  using dkf_t = DKF<n_states, n_inputs>;
}

using A_t = mrs_lib::dkf_t::A_t;
using B_t = mrs_lib::dkf_t::B_t;
using H_t = mrs_lib::dkf_t::H_t;
using Q_t = mrs_lib::dkf_t::Q_t;
using u_t = mrs_lib::dkf_t::u_t;
using statecov_t = mrs_lib::dkf_t::statecov_t;

namespace e = Eigen;

namespace uvdar {


  /**
   * @brief A processing lass for filtering measurements from UVDAR-based relative UAV pose estimator. This uses a vector of Kalman filter states for tracking multiple targets either if they have known identity or if they are anonymous.
   */
  class UWB_UVDAR_Fuser {

    private:
      /* attributes //{ */

      bool _debug_ = false;

      bool initialized_ = false;
      std::string _uav_name_;

      std::string _output_frame_;
      int _input_count_;
      double _output_framerate_;
      bool _odometry_available_, _indoor_;
      double vl, vv, sn;
      bool _anonymous_measurements_;
      bool _use_velocity_;


      /* std::vector<bool> input_data_initialized_; */

      std::mutex meas_mutex;
      std::mutex filter_mutex;
      std::mutex transformer_mutex;

      std::shared_ptr<mrs_lib::dkf_t> filter;

      struct td_t{
        A_t A;
        B_t B;
        H_t H;
        Q_t Q;
      };
      td_t filter_matrices;

      struct FilterData{
        statecov_t filter_state;
        int update_count;
        ros::Time latest_update;
        ros::Time latest_measurement;
        unsigned long long int id;
      };

      std::vector<FilterData> fd;
      // | ----------------------- subscribers ---------------------- |

      std::vector<ros::Subscriber> sub_blinkers_;
      ros::Subscriber sub_range_;

      // | ----------------------- publishers ---------------------- |
      ros::Publisher pub_filter_;
      ros::Publisher pub_filter_tent_;

      ros::Publisher pub_status_;

      ros::Timer timer;
      ros::Duration filter_update_period;
      double dt;
      unsigned long long int latest_id = 0;

      mrs_lib::Transformer transformer_;


      // | ----------------------- camera data --------------------- |
      using blinkers_seen_callback_t = boost::function<void (const uvdar_core::ImagePointsWithFloatStampedConstPtr& msg)>;
      struct CameraContext{
        std::string topic;
        blinkers_seen_callback_t callback;
        ros::Subscriber subscriber;
        std::string tf_frame;
        geometry_msgs::TransformStamped fromcam_tf;
        /* geometry_msgs::TransformStamped tocam_tf */
        std::string calibration_file;
        struct ocam_model oc_model;
        cv::Size image_size;
      };
      std::vector<CameraContext> cameras_;
      unsigned int _camera_count_;
      std::string calibration_file;
      std::string blinker_topic;
      //}

    public:
      /**
       * @brief Constructor - loads parameters and initializes necessary structures
       *
       * @param nh Private NodeHandle of this ROS node
       */
      /* Constructor //{ */
      UWB_UVDAR_Fuser(ros::NodeHandle nh) {
        ros::Time::waitForValid();

        mrs_lib::ParamLoader param_loader(nh, "UWB_UVDAR_Fuser");

        param_loader.loadParam("debug", _debug_);

        param_loader.loadParam("uav_name", _uav_name_);
        param_loader.loadParam("output_frame", _output_frame_, std::string("local_origin"));
        param_loader.loadParam("output_framerate", _output_framerate_, double(DEFAULT_OUTPUT_FRAMERATE));

        param_loader.loadParam("anonymous_measurements", _anonymous_measurements_, bool(false));

        param_loader.loadParam("indoor", _indoor_, bool(false));
        param_loader.loadParam("odometry_available", _odometry_available_, bool(true));
        param_loader.loadParam("use_velocity", _use_velocity_, bool(false));

        /* const auto transform_lookup_timeout = param_loader.loadParam2<ros::Duration>("transform_lookup_timeout", ros::Duration(0.005)); */

        if (_indoor_){ //lateral and vertical velocities are more limited in indoor conditions
          vl = 1;
          vv = 0.5;
        }
        else {
          vl = 2;
          vv = 1;
        }

        if (_odometry_available_){ //process noise is greater, since without odometry we can't correct for the ego-motion of the observer
          sn = 2;
        }
        else {
          sn = 4;
        }

        filter_update_period = ros::Duration(1.0 / fmax(_output_framerate_,1.0));
        dt = filter_update_period.toSec();
        timer = nh.createTimer(filter_update_period, &UWB_UVDAR_Fuser::spin, this);

        transformer_ = mrs_lib::Transformer("UWB_UVDAR_Fuser");
        transformer_.setDefaultPrefix(_uav_name_);


        std::vector<std::string> _blinkers_seen_topics;
        param_loader.loadParam("blinkers_seen_topics", _blinkers_seen_topics, _blinkers_seen_topics);
        if (_blinkers_seen_topics.empty()) {
          ROS_ERROR("[UVDARPoseCalculator]: No topics of blinkers were supplied");
          return;
        }

        std::vector<std::string> _calib_files;
        param_loader.loadParam("calib_files", _calib_files, _calib_files);
        if (_calib_files.empty()){
          ROS_ERROR("[UVDARPoseCalculator]: No calibration files were supplied");
          return;
          }

        std::vector<std::string> _camera_frames;
        param_loader.loadParam("camera_frames", _camera_frames, _camera_frames);
        if (_camera_frames.empty()){
          ROS_ERROR("[UVDARPoseCalculator]: No camera TF frames were supplied");
          return;
        }

        if ( _blinkers_seen_topics.size() != _calib_files.size()){
          ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: The number of provided blinker topics of " << _blinkers_seen_topics.size() << " does not match the number of provided calibration files of " << _calib_files.size());
          return;
        }

        if ( _blinkers_seen_topics.size() != _camera_frames.size()){
          ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: The number of provided blinker topics of " << _blinkers_seen_topics.size() << " does not match the number of provided TF camera frames of " << _camera_frames.size());
          return;
        }

        _camera_count_ = (unsigned int)(_blinkers_seen_topics.size());
        for (unsigned int i = 0; i < _camera_count_; i++){
          cameras_.push_back(CameraContext());

          cameras_.back().topic = _blinkers_seen_topics.at(i);

          cameras_.back().calibration_file = _calib_files.at(i);

          cameras_.back().tf_frame = _camera_frames.at(i);

          cameras_.back().callback = [image_index=i,this] (const uvdar_core::ImagePointsWithFloatStampedConstPtr& pointsMessage) { 
            ProcessBlinkPoints(pointsMessage, image_index);
          };

          ROS_INFO_STREAM("[UVDARPoseCalculator]: Subscribing to " << _blinkers_seen_topics[i]);
          cameras_.back().subscriber = nh.subscribe(_blinkers_seen_topics[i], 1, cameras_.back().callback);

          cameras_.back().image_size = cv::Size(-1,-1);


          if (_calib_files.at(i) == "default"){
            cameras_.back().calibration_file = ros::package::getPath("uvdar_core")+"/config/ocamcalib/calib_results_bf_uv_fe.txt";
          }
          else {
            cameras_.back().calibration_file = _calib_files.at(i);
          }

          get_ocam_model(&cameras_.back().oc_model, (char*)(cameras_.back().calibration_file.c_str()));
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Calibration parameters for virtual camera " << i << " came from the file " <<  cameras_.back().calibration_file);
          for (int j=0; j<cameras_.back().oc_model.length_pol; j++){
            if (isnan(cameras_.back().oc_model.pol[j])){
              ROS_ERROR("[UVDARPoseCalculator]: Calibration polynomial containts NaNs! Returning.");
              return;
            }
          }

        }

        nh.subscribe("ranges_in", 1, &UWB_UVDAR_Fuser::ProcessRanges, this);


        pub_filter_ = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("filtered_poses", 1);
        pub_filter_tent_ = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("filtered_poses/tentative", 1);

        pub_status_ = nh.advertise<std_msgs::String>("/"+_uav_name_+"/mrs_uav_status/display_string", 1);

        if (_anonymous_measurements_ && _use_velocity_){
          ROS_WARN("[UWB_UVDAR_Fuser]: Velocity estimation for anonymous measurements is not implemented. Returning.");
          return;
        }

        if (_anonymous_measurements_){
          filter_matrices.A.resize(6,6);
          filter_matrices.Q.resize(6,6);
          filter_matrices.H.resize(6,6);

          filter_matrices.H <<
            1,0,0,0,0,0,
            0,1,0,0,0,0,
            0,0,1,0,0,0,
            0,0,0,1,0,0,
            0,0,0,0,1,0,
            0,0,0,0,0,1;

        }
        else {
          if (_use_velocity_){
            filter_matrices.A.resize(9,9);
            filter_matrices.Q.resize(9,9);
            filter_matrices.H.resize(6,9);

            filter_matrices.H <<
              1,0,0,0,0,0,0,0,0,
              0,1,0,0,0,0,0,0,0,
              0,0,1,0,0,0,0,0,0,
              0,0,0,0,0,0,1,0,0,
              0,0,0,0,0,0,0,1,0,
              0,0,0,0,0,0,0,0,1;
          }
          else {
            filter_matrices.A.resize(6,6);
            filter_matrices.Q.resize(6,6);
            filter_matrices.H.resize(6,6);

            filter_matrices.H <<
              1,0,0,0,0,0,
              0,1,0,0,0,0,
              0,0,1,0,0,0,
              0,0,0,1,0,0,
              0,0,0,0,1,0,
              0,0,0,0,0,1;
          }
        }

        filter_matrices.B = B_t();


        filter = std::make_unique<mrs_lib::dkf_t>(filter_matrices.A, filter_matrices.B, filter_matrices.H);


        if (param_loader.loadedSuccessfully()) {
          initialized_ = true;
          ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: initiated");
        } else {
          ROS_ERROR_ONCE("[UWB_UVDAR_Fuser]: PARAMS not loaded correctly, shutting down.");
          nh.shutdown();
        }


      }
      //}

    private:

      /**
       * @brief Callback to process input blinker messges
       *
       * @param msg - array of image points with identities produced by the camera and processing
       * @param image_index - index of the camera producing the given message
       */
      /* ProcessBlinkPoints //{ */
      void ProcessBlinkPoints(const uvdar_core::ImagePointsWithFloatStampedConstPtr& msg, size_t image_index) {
        if (!initialized_){
          ROS_ERROR_STREAM("[UWB_UVDAR_Fuser]: Uninitialized! Ignoring image points.");
          return;
        }
        
        cameras_[image_index].image_size = cv::Size(msg->image_width, msg->image_height);

        if ((int)(msg->points.size()) < 1)
          return;
        if (_debug_)
          ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Getting " << (int)(msg->points.size()) << " points...");

        e::Vector3d camera_origin;
        {
          std::scoped_lock lock(transformer_mutex);

          auto fromcam_tmp = transformer_.getTransform(cameras_[image_index].tf_frame,_uav_name_+"/"+_output_frame_, msg->stamp);
          if (!fromcam_tmp){
            ROS_ERROR_STREAM("[UWB_UVDAR_Fuser]: Could not obtain transform from " << cameras_[image_index].tf_frame  << " to " << _uav_name_+"/"+_output_frame_ << "!");
            return;
          }
          else
            cameras_[image_index].fromcam_tf = fromcam_tmp.value();


          auto camera_origin_tmp = transformer_.transform(e::Vector3d(0,0,0), cameras_[image_index].fromcam_tf);
          if (!camera_origin_tmp){
            ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: Failed to transform camera origin! Returning.");
            return;
          }
          else
            camera_origin = camera_origin_tmp.value();
        }


        uvdar_core::ImagePointsWithFloatStamped msg_local;
        {
          /* std::scoped_lock lock(uvdar_meas_mutex); */
          //TODO consider if it is necessary to mutex or locally copy this
          msg_local = *msg;
        }

        {
          std::scoped_lock lock(filter_mutex);

          std::vector< std::vector<uvdar_core::Point2DWithFloat > > associated_points = associateImagePointsToTargets(msg_local); //each element is a vector of image points with associated target id

          int target = 0;
          for (auto target_points : associated_points) {
            for (auto& pt: target_points){
              e::Vector3d curr_direction_local = directionFromCamPoint(cv::Point2d(pt.x, pt.y), image_index);

              e::Vector3d curr_direction; //global
              {
                std::scoped_lock lock(transformer_mutex);
                auto curr_direction_tmp = transformer_.transformAsVector(curr_direction_local, cameras_[image_index].fromcam_tf);
                if (!curr_direction_tmp){
                  ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: Failed to transform direction vector! Returning.");
                  return;
                }
                else
                  curr_direction = curr_direction_tmp.value();
              }

              //TODO: implement Masreliez Uniform Updater in mrs_lib
              //TODO: implement plane filtering in DKF
              //TODO: initialize fd[target]
              //TODO: mutex fd[target]

              // Apply the correction step for line
              /* fd[target].filter_state = filter.correctLine(fd[target].filter_state, camera_origin, curr_direction, LINE_VARIANCE, true); //true should activate Masreliez uniform filtering */
              fd[target].filter_state = filter->correctLine(fd[target].filter_state, camera_origin, curr_direction, LINE_VARIANCE);

              // Restrict state to be in front of the camera
              double dist_range_span = (RANGE_MAX-RANGE_MIN)/2.0;
              double dist_range_center = RANGE_MIN+dist_range_span;
              /* fd[target].filter_state = filter.correctPlane(fd[target].filter_state, camera_origin+(curr_direction*dist_range_center), curr_direction, dist_range_span, true); //true should activate Masreliez uniform filtering */
              fd[target].filter_state = filter->correctPlane(fd[target].filter_state, camera_origin+(curr_direction*dist_range_center), curr_direction, dist_range_span);


            }
            target++;
          }
        }

      }
      //}
      
      /**
       * @brief Splits all points from the blinker message into groups associated with specific target trackers
       *
       * @param msg - the bliker data to split
       *
       * @return A vector of vectors of blinker points, each associated with its own tracker
       *
       * associateImagePointsToTargets //{ */
      std::vector<std::vector<uvdar_core::Point2DWithFloat>> associateImagePointsToTargets(uvdar_core::ImagePointsWithFloatStamped msg){
        std::vector< std::pair<int, std::vector<uvdar_core::Point2DWithFloat> > > output;
        for (auto &f : fd){
          output.push_back(std::vector<uvdar_core::Point2DWithFloat>());
        }

        for (auto &pt : msg.points){
          int ID = targetIDFromUVDAR(pt.value);
          if ((ID >= 0) && (ID < fd.size())){
            output[ID].push_back(pt);
          }
          else {
            ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: Blinker point with ID " << pt.value << " matches no known target ID.");
            return {{}};
          }
        }
        return output;
      }
      //}
      
      /**
       * @brief Associates the blinking sequence ID retrieved by UVDAR with a specific target
       *
       * @param uvdar_id - the Blinker ID provided by UVDAR
       *
       * @return An ID of the target that is expected to carry the uvdar_id
       *
       * targetIDFromUVDAR //{ */
      int targetIDFromUVDAR(double uvdar_id){
        //TODO - maybe fill in look up table using a pre-loaded file
        return -1;
      }
      //}
      
      /**
       * @brief Callback to process input range measurement messges
       *
       * @param msg
       */
      /* ProcessRanges //{ */
      void ProcessRanges(const uvdar_core::ImagePointsWithFloatStampedConstPtr& msg) {
        return;
      }
      //}
      

      /**
       * @brief Thread for processing, predicting and outputting filter states at a regular rate
       *
       * @param te TimerEvent for the timer spinning this thread
       */
      /* spin //{ */
      void spin([[ maybe_unused ]] const ros::TimerEvent& te){
        std::scoped_lock lock(filter_mutex);
        removeNANs();
        if (_anonymous_measurements_){
          removeOverlaps();
        }
        for (int target=0; target<(int)(fd.size());target++){
          /* int targetsSeen = 0; */
          double age = (ros::Time::now() - fd[target].latest_measurement).toSec();
          if (_debug_)
            ROS_INFO("[UWB_UVDAR_Fuser]: Age of %d is %f", target, age);
          double decay_age;
          if (fd[target].update_count > MIN_MEASUREMENTS_TO_VALIDATION){
            decay_age = DECAY_AGE_NORMAL;
          }
          else {
            decay_age = DECAY_AGE_UNVALIDATED;
          }
          if (age>decay_age){
            ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Removing state " << target << " (ID:" << fd[target].id << ") : " << fd[target].filter_state.x.transpose() << " due to old age of " << age << " s." );
            fd.erase(fd.begin()+target);
            target--;
            continue;
          }
          predictTillTime(fd.at(target), ros::Time::now(), true);

          if (_debug_){
            ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: State: ");
            ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: \n" << fd[target].filter_state.x);
          }


            /* if ( */
            /*     ((fd[target].filter_state.P.diagonal().array() < 0).any()) */
            /*    ){ */
            /*   ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: SPIN: NEGATIVE NUMBERS ON MAIN DIAGONAL!"); */
            /*   ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: State. cov: " << std::endl << fd[target].filter_state.P); */
            /* } */
        }

        publishStates();

      }
      //}

      /**
       * @brief Initiates a new filter based on a measurement
       *
       * @param x Measurement vector
       * @param C Error covariance associated with x
       * @param stamp Time of the measurement
       * @param id Identity of the target (-1 means that it is unknown or irrelevant)
       */
      /* initiateNew //{ */
      void initiateNew(e::VectorXd x, e::MatrixXd C, ros::Time stamp, int id = -1){
        if (fd.size() > 20)
          return;

        bool changed = false;
        auto eigens = C.topLeftCorner(3,3).eigenvalues();
        for (int i=0; i<3; i++){
          if (eigens(i).real() > (x.topLeftCorner(3,1).norm())){
            eigens(i) = 5.0;
            changed = true;
          }
        }

        if (changed){
          if (id == -1){
            e::EigenSolver<e::Matrix3d> es(C.topLeftCorner(3,3));
            C.topLeftCorner(3,3) = es.eigenvectors().real()*eigens.real().asDiagonal()*es.eigenvectors().real().transpose();
            x.topLeftCorner(3,1) = x.topLeftCorner(3,1).normalized()*15;
            //so that we don't initialize with the long covariances intersecting in the origin
          }
        }

          auto C_local = C; // the correlation between angle and position only exists in the measurement - the filter may have many measurements where this relation does not exist anymore
        if (!_use_velocity_){
          int index = (int)(fd.size());
          ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Initiating state " << index << "(ID:" << ((id<0)?(latest_id++):(id)) << ")  with: " << x.transpose());
          ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: The source input of the state is " << (ros::Time::now() - stamp).toSec() << "s old.");

          C_local.topRightCorner(3,3).setZero();  // check the case of _use_velocity_ == true
          C_local.bottomLeftCorner(3,3).setZero();  // check the case of _use_velocity_ == true

          fd.push_back({.filter_state = {.x = x, .P = C_local}, .update_count = 0, .latest_update = stamp, .latest_measurement = stamp, .id = ((id<0)?(latest_id++):(id))});
        }
        else{
          ROS_WARN("[UWB_UVDAR_Fuser]: Initialization of states with velocity is not implemented. Returning.");
          return;
        }

        /* if ( */
        /*     ((fd.back().filter_state.P.diagonal().array() < 0).any()) */
        /*    ){ */
        /*   ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: INIT: NEGATIVE NUMBERS ON MAIN DIAGONAL!"); */
        /*   ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Generating measurement: " << std::endl << C); */
        /*   ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Generating fixed measurement: " << std::endl << C_local); */
        /*   ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: State. cov: " << std::endl << fd.back().filter_state.P); */
        /* } */
      }
      //}

      /**
       * @brief Predicts the filter state at target_time, given that target_time is newer than the last update or measurement
       *
       * @param fd_curr The filter data structure to start from (if apply_update is true, this structure will be updated with the result)
       * @param target_time The time to which the state is to be predicted
       * @param apply_update If true, the output will be applied to the structure fd_curr. This provides more data than the return value, such as the update time
       *
       * @return The resulting state and its output covariance
       */
      /* predictTillTime //{ */
      statecov_t predictTillTime(struct FilterData &fd_curr, ros::Time target_time, bool apply_update = false){
        auto orig_state = fd_curr;
        double dt_from_last = std::fmax(std::fmin((target_time-fd_curr.latest_update).toSec(),(target_time-fd_curr.latest_measurement).toSec()),0.0);
        filter->A = A_dt(dt_from_last);
        /* ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Filter pred. orig: " << std::endl << fd_curr.filter_state.P); */
        auto new_state =  filter->predict(fd_curr.filter_state, u_t(), Q_dt(dt_from_last), dt_from_last);
        /* ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Filter predicted: " << std::endl << fd_curr.filter_state.P); */
        if (apply_update){
          fd_curr.filter_state = new_state;
          fd_curr.latest_update = target_time;
        }

            if (
                ((orig_state.filter_state.P.diagonal().array() < 0).any()) ||
                ((fd_curr.filter_state.P.diagonal().array() < 0).any())
               ){
              ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: NEGATIVE NUMBERS ON MAIN DIAGONAL!");
              ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Orig. state. cov: " << std::endl << orig_state.filter_state.P);
              /* ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: dt: " << dt_from_last); */
              /* ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Q_dt: " << Q_dt(dt_from_last)); */
              ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: New state. cov: " << std::endl << fd_curr.filter_state.P);
            }
        return new_state;
      }
      //}

      /**
       * @brief Predicts the change of the state and error covariance till meas_time, and then corrects the state with measurement
       *
       * @param fd_curr The filter data structure to start from (if apply_update is true, this structure will be updated with the result)
       * @param measurement The input measurement to correct the state with
       * @param match_level The output level of overlap between the original state fd_curr and the measurement in terms of position. This is based on the maximum of multivariate Gaussian multiplication between the measurement and the state, and represents information that is normally lost in the Kalman filter process
       * @param meas_time The time of the input measurement
       * @param prior_predict if true, the state is first brought to meas_time by predicting its change from the last update or measurement
       * @param apply_update If true, the output will be applied to the structure fd_curr. This provides more data than the return value, such as the update time
       *
       * @return The resulting state and its output covariance
       */
      /* correctWithMeasurement //{ */
      statecov_t correctWithMeasurement(struct FilterData &fd_curr, statecov_t measurement, double &match_level_pos, ros::Time meas_time, bool prior_predict, bool apply_update = false, std::string camera_frame = ""){
        auto filter_local = fd_curr;

        auto orig_state = filter_local.filter_state;
        /* ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Original state: " << fd_curr.id << " was " << filter_local.filter_state.x.transpose()); */
        /* ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Original cov: " << std::endl << filter_local.filter_state.P); */

        if (prior_predict){
          predictTillTime(filter_local, meas_time, true);
        }


        /* ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Orig. angles: " << filter_local.filter_state.x.bottomRows(3)); */
        filter_local.filter_state.x[3] = fixAngle(filter_local.filter_state.x[3], measurement.x[3]);
        filter_local.filter_state.x[4] = fixAngle(filter_local.filter_state.x[4], measurement.x[4]);
        filter_local.filter_state.x[5] = fixAngle(filter_local.filter_state.x[5], measurement.x[5]);
        /* ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Fixed. angles: " << filter_local.filter_state.x.bottomRows(3)); */

        match_level_pos = gaussJointMaxVal(
            measurement.P.topLeftCorner(3,3),
            filter_local.filter_state.P.topLeftCorner(3,3),
            measurement.x.topRows(3),
            filter_local.filter_state.x.topRows(3)
            );
        /* double match_level_rot = gaussJointMaxVal( */
        /*     measurement.P.bottomRightCorner(3,3), */
        /*     filter_local.filter_state.P.bottomRightCorner(3,3), */
        /*     measurement.x.bottomRows(3), */
        /*     filter_local.filter_state.x.bottomRows(3) */
        /*     ); */

        auto P_local = measurement.P;
        /* ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Meas. cov: " << std::endl << P_local); */
        P_local.topRightCorner(3,3).setZero();  // check the case of _use_velocity_ == true
        P_local.bottomLeftCorner(3,3).setZero();  // check the case of _use_velocity_ == true

        /* double dt_from_last = std::fmax(std::fmin((filter_local.latest_update-meas_time).toSec(),(filter_local.latest_measurement-meas_time).toSec()),0.0); */
        /* P_local += Q_dt(dt_from_last)*dt_from_last; */

        /* ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Scaling by: " << 1.0/match_level << " due to match level of " << match_level); */

        /* double volume_ratio_pos = std::min(1.0,filter_local.filter_state.P.topLeftCorner(3,3).determinant()/P_local.topLeftCorner(3,3).determinant()); */

        /* P_local.topLeftCorner(3,3) *= (1.0/(match_level_pos*volume_ratio_pos)); */

        P_local.topLeftCorner(3,3) *= (1.0/(match_level_pos));

        /* P_local.bottomRightCorner(3,3) *= (1.0/match_level_rot); */
        /* ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: With ML: " << std::endl << P_local); */
        /* P_local *= std::max(0.1,(1.0/match_level)); */


        /* ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Filter orig: " << std::endl << filter_local.filter_state.P); */
        try {
          filter_local.filter_state = filter->correct(filter_local.filter_state, measurement.x, P_local);
        }
        catch ([[maybe_unused]] std::exception& e) {
          ROS_ERROR_STREAM("[UWB_UVDAR_Fuser]: Attempted to correct with bad covariance (match_level_pos = " << match_level_pos << "). Will replace with big, but manageable one.");
          /* P_local = e::MatrixXd(6,6); */
          P_local.setIdentity();
          P_local *= 10000;
          filter_local.filter_state = filter->correct(filter_local.filter_state, measurement.x, P_local);
        }
        /* ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Filter corr: " << std::endl << filter_local.filter_state.P); */
        /* ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Fixed to: " << std::endl << P_local); */

        /* filter_local.filter_state.P *= (1+match_level);//this makes the filter not increase certainty in case of multiple identical measurements - the mean in the covariances is more probable than the rest of its x<1*sigma space */

        auto eigens_pos = filter_local.filter_state.P.topLeftCorner(3,3).eigenvalues();
        double min_eig_pos = eigens_pos.real().minCoeff();
        auto eigens_rot = filter_local.filter_state.P.bottomRightCorner(3,3).eigenvalues();
        /* double min_eig_rot = eigens_rot.real().minCoeff(); */
        filter_local.filter_state.P.topLeftCorner(3,3) += e::MatrixXd::Identity(3,3)*(min_eig_pos*(match_level_pos));//this makes the filter not increase certainty in case of multiple identical measurements - the mean in the covariances is more probable than the rest of its x<1*sigma space
        /* filter_local.filter_state.P.bottomRightCorner(3,3) += e::MatrixXd::Identity(3,3)*(min_eig_rot*(match_level_rot));//this makes the filter not increase certainty in case of multiple identical measurements - the mean in the covariances is more probable than the rest of its x<1*sigma space */
        /* ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Filter exp.: " << std::endl << filter_local.filter_state.P); */

        filter_local.filter_state.x[3] = fixAngle(filter_local.filter_state.x[3], 0);
        filter_local.filter_state.x[4] = fixAngle(filter_local.filter_state.x[4], 0);
        filter_local.filter_state.x[5] = fixAngle(filter_local.filter_state.x[5], 0);

        if (isInFrontOfCamera(filter_local.filter_state.x.topRows(3), camera_frame, meas_time)){
          if (apply_update){
            /* if (_debug_){ */
            /* if ( */
            /*     ((orig_state.x.topRows(3) - filter_local.filter_state.x.topRows(3)).norm() > 2.0) || */
            /*     ((orig_state.x.bottomRows(3) - filter_local.filter_state.x.bottomRows(3)).norm() > 0.2) */
            /*    ){ */
            /*   ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Updating state: " << fd_curr.id << " with " << measurement.x.transpose()); */
            /*   ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Meas. cov: " << std::endl << measurement.P); */
            /*   ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Fixed to: " << std::endl << P_local); */
            /*   ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: State. cov: " << std::endl << orig_state.P); */
            /*   ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: This yelds state: " << filter_local.filter_state.x.topRows(6).transpose()); */
            /* } */
            /* if ( */
            /*     ((orig_state.P.diagonal().array() < 0).any()) || */
            /*     ((measurement.P.diagonal().array() < 0).any()) || */ 
            /*     ((filter_local.filter_state.P.diagonal().array() < 0).any()) */
            /*    ){ */
            /*   ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: NEGATIVE NUMBERS ON MAIN DIAGONAL!"); */
            /*   ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Meas. cov: " << std::endl << measurement.P); */
            /*   ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Fixed to: " << std::endl << P_local); */
            /*   ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Orig. state. cov: " << std::endl << orig_state.P); */
            /*   ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: New state. cov: " << std::endl << filter_local.filter_state.P); */
            /* } */
            /* } */
            fd_curr.filter_state = filter_local.filter_state;
            fd_curr.latest_measurement = meas_time;
            fd_curr.update_count++;
          }
        }
        return filter_local.filter_state;
      }
      //}

      /**
       * @brief Deletes on of the active filter states
       *
       * @param index The index of the filter state to remove
       */
      /* deleteFilter //{ */
      void deleteFilter(int &index){
        if (index < 0) return;
        fd.erase(fd.begin()+index);
        index--;
      }
      //}

      /**
       * @brief Returns a value between 0 and 1 representing the level of overlap between two probability distributions in the form of multivariate Gaussians. Multiplying two Gaussians produces another Gaussian, with the value of its peak roughly corresponding to the level of the overlap between the two inputs. The ouptut of this function is the value of such peak, given that the input Gaussians have been scaled s.t. their peaks have the value of 1. Therefore, the output is 1 if the means of both inputs are identical.
       *
       * @param si0 The covariance of the first distribution
       * @param si1 The covariance of the second distribution
       * @param mu0 The mean of the first distribution
       * @param mu1 The mean of the second distribution
       *
       * @return The level of overlap between the two distribution
       */
      /* gaussJointMaxVal //{ */
      double gaussJointMaxVal(e::MatrixXd si0,e::MatrixXd si1,e::VectorXd mu0,e::VectorXd mu1){
        bool scaled = true;
        int k=mu0.size();
        auto K=si0*(si0+si1).inverse();
        auto d0=K*(mu1-mu0);
        auto d1=(K-e::MatrixXd::Identity(K.rows(),K.rows()))*(mu1-mu0);
        double N;
        if (scaled){
          auto N_v = ((-0.5)*( (d0.transpose()*(si0).inverse()*d0) + (d1.transpose()*(si1).inverse()*d1) ));
          N=exp(N_v(0));
        }
        else{
          auto N_v = ((-0.5)*( (d0.transpose()*(si0).inverse()*d0) + (d1.transpose()*(si1).inverse()*d1) ));
          N=(1.0/pow((2*M_PI),k)*sqrt((si0).determinant()*(si1).determinant()))*exp(N_v(0));
        }
        if (isnan(N)){
          ROS_INFO("[UWB_UVDAR_Fuser]: Joint Gaussian value came out NaN");
          ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Sigma 1: " << std::endl << si0);
          ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Sigma 2: " << std::endl << si1);
        }

        return N;
      }
      //}

      bool isInFrontOfCamera(e::Vector3d mean, std::string camera_frame, ros::Time stamp){
        geometry_msgs::PoseStamped target_cam_view, target_filter;
        target_filter.header.frame_id = _output_frame_;
        target_filter.header.stamp = stamp;
        target_filter.pose.position.x = mean.x();
        target_filter.pose.position.y = mean.y();
        target_filter.pose.position.z = mean.z();

        auto ret = transformer_->transformSingle(target_filter, camera_frame);
        if (ret) {
          target_cam_view = ret.value();
        }
        else{
          ROS_ERROR_STREAM("[UWB_UVDAR_Fuser]: Could not transform filter state from "<< _output_frame_ << " to camera frame " << camera_frame);
          return false;
        }

        e::Vector3d target_cam_view_vector(target_cam_view.pose.position.x, target_cam_view.pose.position.y, target_cam_view.pose.position.z);
        double norm = target_cam_view_vector.norm();
        double cos_angle = target_cam_view_vector.normalized().dot(e::Vector3d(0,0,1));

        return ((norm > 1.5) && (cos_angle > -0.173648));//cos(100 deg) 

      }

      /**
       * @brief Applies relative position measurements to states that are the closest to their position component
       *
       * @param measurements A vector of measurements with error covariances
       * @param meas_time The time of the input measurements
       */
      /* applyMeasurementsAnonymous //{ */
      void applyMeasurementsAnonymous(std::vector<statecov_t> measurements, ros::Time meas_time, std::string camera_frame){
        std::scoped_lock lock(filter_mutex);
        if (fd.size() == 0){
          for (auto const& measurement_curr : measurements | indexed(0)){
            initiateNew(measurement_curr.value().x, measurement_curr.value().P, meas_time);
          }
          return;
        }

        e::MatrixXd match_matrix(measurements.size(),fd.size());
        std::vector<std::vector<FilterData>> tentative_states;

        for (auto const& measurement_curr : measurements | indexed(0)){
          tentative_states.push_back(std::vector<FilterData>());
          for (auto const& state_curr : fd | indexed(0)){

            [[ maybe_unused ]] double dt_from_last = std::fmin((meas_time-state_curr.value().latest_update).toSec(),(meas_time-state_curr.value().latest_measurement).toSec());
            

            tentative_states.back().push_back(
                state_curr.value()
                );
            double match_level;
            correctWithMeasurement(tentative_states.back().back(),measurement_curr.value(), match_level, meas_time, true, true, camera_frame);
            match_matrix(measurement_curr.index(),state_curr.index()) = match_level;
            double dt_s = 0.1;
            if ((ros::Time::now() - state_curr.value().latest_measurement).toSec() < dt_s){ // just in case - in simulation the camera outputs follow one another immediately, so no inflation happens in between
              tentative_states.back().back().filter_state = predictTillTime(tentative_states.back().back(), ros::Time::now()+ros::Duration(dt_s),false);
            }
          }
        }

        if (_debug_){
          ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: match_matrix: " << std::endl <<match_matrix);
        }

        std::vector<std::pair<int,int>> matches;

        int best_index;
        double best_match_level;
        double best_update_count;
        double curr_match_level;
        for (int f_i = 0; f_i < (int)(fd.size()); f_i++) {
          best_update_count = -1;
          best_index = -1;
          best_match_level = -1;
          for (int m_i = 0; m_i < (int)(measurements.size()); m_i++) {
            curr_match_level = match_matrix(m_i,f_i);  
            if ((curr_match_level > MATCH_LEVEL_THRESHOLD_ASSOCIATE) && ( (fd[f_i].update_count > best_update_count) || ((fd[f_i].update_count == best_update_count) && (curr_match_level > best_match_level)) ) ){
              best_match_level = curr_match_level;
              best_index = m_i;
              best_update_count = fd[f_i].update_count;
            }
          }

          if (best_index >= 0){
            matches.push_back({best_index,f_i});
            for (int f_j = 0; f_j < (int)(fd.size()); f_j++) {
              match_matrix(best_index,f_j) = std::nan("");
            }
          }
          for (int m_j = 0; m_j < (int)(measurements.size()); m_j++) {
            if (match_matrix(m_j,f_i) > MATCH_LEVEL_THRESHOLD_ASSOCIATE){
              match_matrix(m_j,f_i) = std::nan("");
            }
          }
        }

        for( auto& match_curr : matches){
          if (_debug_){
            ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Updating state: " << match_curr.second << " with " << measurements[match_curr.first].x.transpose());
            ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: This yelds state: " << tentative_states[match_curr.first][match_curr.second].filter_state.x.topRows(6).transpose());
          }
          fd[match_curr.second] = tentative_states[match_curr.first][match_curr.second];

        }

        if (_debug_){
          ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: match_matrix post: " << std::endl <<match_matrix);
        }
        int fd_size_orig = (int)(fd.size());
        for (int f_i = 0; f_i < fd_size_orig; f_i++) {
          for (int m_i = 0; m_i < (int)(measurements.size()); m_i++) {
            if (_debug_){
              ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: match_matrix at: [" << m_i << ":" << f_i << "] is: " << match_matrix(m_i,f_i));
            }
            if (!isnan(match_matrix(m_i,f_i))){
              initiateNew(measurements[m_i].x, measurements[m_i].P, meas_time);
              for (int f_j = 0; f_j < fd_size_orig; f_j++) {
                match_matrix(m_i,f_j) = std::nan("");
              }
              for (int m_j = 0; m_j < (int)(measurements.size()); m_j++) {
                match_matrix(m_j,f_i) = std::nan("");
              }
            }
          }
        }

      }
      //}

      /**
       * @brief Applies relative position measurements to states that match their IDs
       *
       * @param measurements A vector of measurements with error covariances
       * @param ids A vector of IDs associated with the measurements - deanonymize measurements of multiple targets
       * @param meas_time
       */
      /* applyMeasurementsWithIdentity //{ */
      void applyMeasurementsWithIdentity(std::vector<statecov_t> measurements, std::vector<int> ids, ros::Time meas_time, std::string camera_frame){
        std::scoped_lock lock(filter_mutex);

        if (measurements.size() != ids.size()){
            ROS_ERROR("[UWB_UVDAR_Fuser]: the sizes of the input measurement vector and of the vector of their identities do not match! Returning.");
            return;
            }

        for(auto const& measurement_curr : measurements | indexed(0)){
          int id_local = ids[measurement_curr.index()] % 1000;
          int target = -1;
          for (auto const& filter_curr : fd | indexed(0)){
            if (id_local == (int)filter_curr.value().id){
              target = filter_curr.index();
            }
          }
          if (target < 0){
            initiateNew(measurement_curr.value().x, measurement_curr.value().P, meas_time, id_local);
          }
          else {
            [[ maybe_unused ]] double match_level; //for future use with multiple measurements with the same ID
            correctWithMeasurement(fd.at(target),measurement_curr.value(), match_level,meas_time,true, true, camera_frame);
          }

        }


      }
      //}

      /**
       * @brief Publishes the current set of filter states to an output topic
       */
      /* publishStates //{ */
      void publishStates(){
        mrs_msgs::PoseWithCovarianceArrayStamped msg;
        mrs_msgs::PoseWithCovarianceArrayStamped msg_tent;
        msg.header.frame_id = _output_frame_;
        msg.header.stamp = ros::Time::now();
        msg_tent.header = msg.header;

        mrs_msgs::PoseWithCovarianceIdentified temp;
        e::Quaterniond qtemp;
        for (auto const& fd_curr : fd | indexed(0)){
          temp.id = fd_curr.value().id;

          temp.pose.position.x = fd_curr.value().filter_state.x[0];
          temp.pose.position.y = fd_curr.value().filter_state.x[1];
          temp.pose.position.z = fd_curr.value().filter_state.x[2];

          qtemp = e::AngleAxisd(fd_curr.value().filter_state.x[3], e::Vector3d::UnitX()) * e::AngleAxisd(fd_curr.value().filter_state.x[4], e::Vector3d::UnitY()) * e::AngleAxisd(fd_curr.value().filter_state.x[5], e::Vector3d::UnitZ());

          temp.pose.orientation.x = qtemp.x();
          temp.pose.orientation.y = qtemp.y();
          temp.pose.orientation.z = qtemp.z();
          temp.pose.orientation.w = qtemp.w();

          /* for (int m=0; m<6; m++){ */
          /*   for (int n=0; n<6; n++){ */
          /*     temp.covariance[6*n+m] =  fd_curr.value().filter_state.P(n,m); */
          /*   } */
          /* } */
          temp.covariance = eigenCovarianceToRos(fd_curr.value().filter_state.P);

          if (fd_curr.value().update_count < MIN_MEASUREMENTS_TO_VALIDATION){
            msg_tent.poses.push_back(temp);
          }
          else{
            msg.poses.push_back(temp);
          }
        }

        std_msgs::String msg_status;
        msg_status.data = std::string("UVDAR sees "+std::to_string(msg.poses.size())+" targets").c_str();
        pub_status_.publish(msg_status);

        pub_filter_.publish(msg);
        pub_filter_tent_.publish(msg_tent);
      }
      //}

      /**
       * @brief Removes any filter state that contains NaNs.
       */
      /* removeNANs //{ */
      void removeNANs(){
        for (int i=0; i<(int)(fd.size());i++){
          if ( (fd[i].filter_state.x.array().isNaN().any() ) ||
              (fd[i].filter_state.P.array().isNaN().any() )){
            fd.erase(fd.begin()+i);
            i--;
          }
        }
      }
      //}


      /**
       * @brief Remove filter states that overlap too much with others in terms of their state vectors and associated covariances. Of each overlapping pair, the one removed is either the one that was not yet validated with sufficient number of measurements, or the one with larger covariance (less precise knowledge).
       */
      /* removeOverlaps //{ */
      void removeOverlaps(){
        double curr_match_level;
        for (int i=0; i<((int)(fd.size())-1);i++){
          bool remove_first = false;
          for (int j=i+1; j<(int)(fd.size()); j++){
            curr_match_level = gaussJointMaxVal(
                fd[i].filter_state.P.topLeftCorner(3,3),
                fd[j].filter_state.P.topLeftCorner(3,3),
                fd[i].filter_state.x.topRows(3),
                fd[j].filter_state.x.topRows(3)
                );
            if (curr_match_level > MATCH_LEVEL_THRESHOLD_REMOVE){
              auto eigens = fd[i].filter_state.P.topLeftCorner(3,3).eigenvalues();
              double size_i = (eigens.topLeftCorner(3, 1)).norm();
              eigens = fd[j].filter_state.P.topLeftCorner(3,3).eigenvalues();
              double size_j = (eigens.topLeftCorner(3, 1)).norm();
              int n = -1;
              int m = -1;
              if ( (fd[i].update_count < MIN_MEASUREMENTS_TO_VALIDATION) == (fd[j].update_count < MIN_MEASUREMENTS_TO_VALIDATION) ){
                if ( size_j > size_i ){
                  n = j;
                  m = i;
                }
                else {
                  n = i;
                  m = j;
                }
              }
              else {
                if (fd[i].update_count < MIN_MEASUREMENTS_TO_VALIDATION){
                  n = i;
                  m = j;
                }
                else {
                  n = j;
                  m = i;
                }

              }
              fd.erase(fd.begin()+n);
              ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Removing state " << n << " (ID:" << fd[n].id << "): " << fd[n].filter_state.x.transpose() << " due to large overlap with state " << m << ": " << fd[m].filter_state.x.transpose());
              if (n<m){
                remove_first=true;
                break;
              }
              else {
                j--;
              }
            }
          }
          if (remove_first){
            i--;
          }

        }
      }
      //}

      /**
       * @brief Retrieves aviation Roll angle from a quaternion
       *
       * @param q The input quaternion
       *
       * @return The ouput angle
       */
      /* quatRotateRoll //{ */
      double quatToRoll(e::Quaterniond q){
        e::Matrix3d m = q.matrix();
        return atan2(m(2,1),m(2,2));
      }
      //}



      e::MatrixXd rosCovarianceToEigen(const boost::array<double,36> input){
        e::MatrixXd output(6,6);
        /* if ((int)(input.size()) != 36 ){ */
        /*   ROS_ERROR_STREAM("[UWB_UVDAR_Fuser]: Covariance to be converted to Eigen matrix has " << input.size() << " elements instead of the expected 36! Returning"); */
        /*   return e::MatrixXd(); */
        /* } */

        for (int i=0; i<6; i++){
          for (int j=0; j<6; j++){
            output(j,i) =  input[6*j+i];
          }
        }

        return output;
      }

      boost::array<double,36> eigenCovarianceToRos(const e::MatrixXd input){
        boost::array<double, 36> output;
        if (((int)(input.rows()) != 6 ) || ((int)(input.cols()) != 6 )){
          ROS_ERROR_STREAM("[UWB_UVDAR_Fuser]: Covariance to be converted to Ros has size of  " << input.rows() << "x" << input.cols() << " as opposed to the expected size of 6x6! Returning");
          /* return boost::array<double, 36>(std::nan("")); */
          double n = std::nan("");
          return {n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n};
        }

          for (int m=0; m<6; m++){
            for (int n=0; n<6; n++){
              output[6*n+m] = input(n,m);
            }
          }

        return output;
      }


      /**
       * @brief Retrieves aviation Pitch angle from a quaternion
       *
       * @param q The input quaternion
       *
       * @return The ouput angle
       */
      /* quatRotatePitch //{ */
      double quatToPitch(e::Quaterniond q){
        e::Matrix3d m = q.matrix();
        return atan2( -m(2,0), sqrt( m(2,1)*m(2,1) +m(2,2)*m(2,2) )  );
      }
      //}

      /**
       * @brief Retrieves aviation Yaw angle from a quaternion
       *
       * @param q The input quaternion
       *
       * @return The ouput angle
       */
      /* quatRotateYaw //{ */
      double quatToYaw(e::Quaterniond q){
        e::Matrix3d m = q.matrix();
        return atan2(m(1,0),m(0,0));
      }
      //}

      /**
       * @brief Changes the expression of the origAngle, such that if it is updated in a filtering process with newAngle, circularity issues will be avoided
       *
       * @param origAngle The angle, representing prior state, to be updated
       * @param newAngle The new angle, representing new measurement, that affects origAngle in filtering
       *
       * @return The new expression of origAngle
       */
      /* fixAngle //{ */
      double fixAngle(double origAngle, double newAngle){
        double fixedPre;
        if ( (origAngle>(2*M_PI)) || (origAngle<(-2*M_PI)) )  {
          fixedPre = fmod(origAngle,2*M_PI);
        }
        else {
          fixedPre = origAngle;
        }

        if (fixedPre>(M_PI))
          fixedPre = fixedPre - (2.0*M_PI);
        if (fixedPre < (-M_PI))
          fixedPre = fixedPre + (2.0*M_PI);



        if (fabs(newAngle-fixedPre)<M_PI)
          return fixedPre;

        if (fixedPre>newAngle)
          return (fixedPre - (2.0*M_PI));
        else
          return (fixedPre + (2.0*M_PI));
      }
      //}


      /**
       * @brief Updates the shared system matrix in case that it is time step-dependent. This is useful for non-uniform time-steps
       *
       * @param dt - The current time step of the kalman filter process
       *
       * @return The new system matrix
       */
      /* A_dt //{ */
      A_t A_dt(double dt){
        if (_anonymous_measurements_){
          filter_matrices.A <<
            1,0,0, 0,0,0,
            0,1,0, 0,0,0,
            0,0,1, 0,0,0,
            0,0,0, 1,0,0,
            0,0,0, 0,1,0,
            0,0,0, 0,0,1;
        } else {
          if (_use_velocity_){
            filter_matrices.A <<
              1,0,0,dt,0, 0, 0,0,0,
              0,1,0,0, dt,0, 0,0,0,
              0,0,1,0, 0, dt,0,0,0,
              0,0,0,1, 0, 0, 0,0,0,
              0,0,0,0, 1, 0, 0,0,0,
              0,0,0,0, 0, 1, 0,0,0,
              0,0,0,0, 0, 0, 1,0,0,
              0,0,0,0, 0, 0, 0,1,0,
              0,0,0,0, 0, 0, 0,0,1;
          }
          else{
            filter_matrices.A <<
              1,0,0, 0,0,0,
              0,1,0, 0,0,0,
              0,0,1, 0,0,0,
              0,0,0, 1,0,0,
              0,0,0, 0,1,0,
              0,0,0, 0,0,1;
          }
        }
        return filter_matrices.A;
      }
      //}

      /**
       * @brief Updates the shared process noise covariance matrix in case that it is time step-dependent. This is useful for non-uniform time-steps
       *
       * @param dt - The current time step of the kalman filter process
       *
       * @return The new process noise matrix
       */
      // Q_dt //{ */
      Q_t Q_dt(double dt){
        if (_anonymous_measurements_){
          //simplified to the process described in detail below. These two approaches will be compared
          filter_matrices.Q <<
            vl, 0 ,0, 0, 0 ,0,
            0, vl, 0, 0, 0 ,0,
            0, 0, vv, 0, 0 ,0,
            0, 0, 0,  1, 0 ,0,
            0, 0, 0,  0, 1 ,0,
            0, 0, 0,  0, 0 ,1;
        } else {
          if (_use_velocity_){
            filter_matrices.Q <<
              sn*sn, 0,     0,     0,  0,  0,  0, 0, 0,
              0,     sn*sn, 0,     0,  0,  0,  0, 0, 0,
              0,     0,     sn*sn, 0,  0,  0,  0, 0, 0,
              0,     0,     0,     vl, 0,  0,  0, 0, 0,
              0,     0,     0,     0,  vl, 0,  0, 0, 0,
              0,     0,     0,     0,  0,  vv, 0, 0, 0,
              0,     0,     0,     0,  0,  0,  1, 0, 0,
              0,     0,     0,     0,  0,  0,  0, 1, 0,
              0,     0,     0,     0,  0,  0,  0, 0, 1;
          }
          else{
            // This is an unorthodox approach.
            // I wanted for the process noise covariance Q (expressing a multivariate gaussian) to cover not only the noise in static position estimate, but to also take into account the mean of expected relative velocity, which is not a state variable in this case.
            // The noise should inflate the error covariance of the state approximately linearly. This can be demonstrated in the following example:
            //  In this filter, we can receive measurement with time-stamp after a previous step of prediction. We therefore first need to predict the effects of process noise up to the measurement time, we then correct the state with the measurement and in the next prediction step we need to expand the error covariance from the measurement time to the next step in the regular process.
            //  If the expansion was not to be linear (or at least close to linear), the two prediction steps, splitting a normal time step into two parts, would not add up to the same result as a single prediction with time step equal to the sum of the two.
            // To approach linearity without drastically changing the Kalman filter process (it must be based on multivariate Gaussians) while including an unknown velocity of uniform distribution, we need a Q that represents the sum of the influences of position estimate noise and of the drift due to unknown velocity.
            // The function used here was obtained thusly:
            // We want a normal distribution to approximate an original normal distribution (accounting only for static position noise sn) expanded by uniform (since we don't have any information on the velocity) distribution f_u(x) limited by maximum velocity v_max*dt=s_max (maximum position shift caused by unknown velocity)
            // This uniform distribution has center at mu, and its values are {f_u(x) = 1/(2*s_max) for abs(x-mu) < s_max} and {f_u(x) = 0 otherwise}, so that the integral of the distribudion is 1.
            // A normal distribution with the same standard deviation as such a sum of a normal and uniform distributions resolves (after some calculations) to one with a new standard deviation:
            //  sn_new^2 = (1/2)*sn^2 + (1/6)*v_max^2*dt^2
            // This is not linear. We may be overcomplicating the issue here, and this is a subject to testing in the future.
            filter_matrices.Q <<
              0.5*sn*sn+0.16667*vl*vl*dt*dt,0,0, 0,0,0,
              0,0.5*sn*sn+0.16667*vl*vl*dt*dt,0, 0,0,0,
              0,0,0.5*sn*sn+0.16667*vv*vv*dt*dt, 0,0,0,
              0,0,0, 0.5,0,0,
              0,0,0, 0,0.5,0,
              0,0,0, 0,0,0.5;
          }
        }
        return filter_matrices.Q;
      }
      //}
      
      //TODO: description
      e::Vector3d directionFromCamPoint(cv::Point2d point, int image_index){
        double v_i[2] = {(double)(point.y), (double)(point.x)};
        double v_w_raw[3];
        cam2world(v_w_raw, v_i, &(cameras_[image_index].oc_models));
        return e::Vector3d(v_w_raw[1], v_w_raw[0], -v_w_raw[2]);
      };
      //}

  };


} //uvdar

int main(int argc, char** argv) {
  ros::init(argc, argv, "uwb_uvdar_fuser");
  ros::NodeHandle nh("~");
  uvdar::UWB_UVDAR_Fuser        uuf(nh);

  ROS_INFO("[UWB_UVDAR_Fuser]: UWB-UVDAR fuser node initiated");

  ros::spin();

  return 0;
}
