#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

#include <mrs_lib/dkf.h>

#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/RangeWithCovarianceArrayStamped.h>
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
#define LINE_MAH_THRESH 3.0
#define YAW_THRESH 1.5
#define MATCH_LEVEL_THRESHOLD_ASSOCIATE 0.3
#define MATCH_LEVEL_THRESHOLD_REMOVE 0.5

#define LINE_VARIANCE 1 //meters^2
#define RANGE_MIN 1.0 //meters
#define RANGE_MAX 50.0 //meters

#define MAX_TARGET_COUNT 20

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

namespace Eigen
{
  typedef Matrix< double, 6, 6 > 	Matrix6d;
  typedef Matrix< double, 1, 6 > 	Vector6d;
}

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

      std::unique_ptr<mrs_lib::dkf_t> filter_;

      struct td_t{
        A_t A;
        B_t B;
        /* H_t H; */
        Q_t Q;
      };
      td_t filter_matrices;

      struct FilterData{
        statecov_t filter_state;
        int update_count;
        ros::Time latest_update;
        ros::Time latest_measurement;
        int id;
        bool received_bearing = false;
      };

      std::vector<FilterData> fd_;
      // | ----------------------- subscribers ---------------------- |

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
      
      // | --------------------- UWB ranger data ------------------- |
      std::string _uwb_frame_;
      //

      // }

      // | ---------------------- line prperties ------------------- |
      struct LineProperties{
        e::Vector3d origin;
        e::Vector3d direction;
      };
      //}
      
      // | -------------------- target identifiers ----------------- |
      struct TargetIdentifiers{
        int id; //ID of the target itself
        std::vector<int> uvdar_ids = {};
        int uwb_id = -1;
      };
      std::vector<TargetIdentifiers> _targets_;
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


          cameras_.back().image_size = cv::Size(-1,-1);


          if (_calib_files.at(i) == "default"){
            cameras_.back().calibration_file = ros::package::getPath("uvdar_core")+"/config/ocamcalib/calib_results_bf_uv_fe.txt";
          }
          else {
            cameras_.back().calibration_file = _calib_files.at(i);
          }

          get_ocam_model(&cameras_.back().oc_model, (char*)(cameras_.back().calibration_file.c_str()));
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Calibration parameters for camera " << i << " came from the file " <<  cameras_.back().calibration_file);
          for (int j=0; j<cameras_.back().oc_model.length_pol; j++){
            if (isnan(cameras_.back().oc_model.pol[j])){
              ROS_ERROR("[UVDARPoseCalculator]: Calibration polynomial containts NaNs! Returning.");
              return;
            }
          }

          ROS_INFO_STREAM("[UVDARPoseCalculator]: Subscribing to " << _blinkers_seen_topics[i]);
          cameras_.back().subscriber = nh.subscribe(_blinkers_seen_topics[i], 1, cameras_.back().callback);

        }

        param_loader.loadParam("uwb_frame", _uwb_frame_);

        sub_range_ = nh.subscribe<mrs_msgs::RangeWithCovarianceArrayStamped>("ranges_in", 1, &UWB_UVDAR_Fuser::ProcessRanges, this);


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


        }
        else {
          if (_use_velocity_){
            filter_matrices.A.resize(9,9);
            filter_matrices.Q.resize(9,9);
          }
          else {
            filter_matrices.A.resize(6,6);
            filter_matrices.Q.resize(6,6);
          }
        }

        filter_matrices.B = B_t();


        filter_ = std::make_unique<mrs_lib::dkf_t>(filter_matrices.A, filter_matrices.B);


        if (param_loader.loadedSuccessfully()) {
          initialized_ = true;
          ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: initiated");
        } else {
          ROS_ERROR_ONCE("[UWB_UVDAR_Fuser]: PARAMS not loaded correctly, shutting down.");
          nh.shutdown();
        }

        loadTargetIdentifiers(param_loader);

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

          auto associated_points = associateImagePointsToTargets(msg_local, image_index); //each element is a vector of image points with associated target id

          for (auto target_points : associated_points) {
          int target = target_points.first;
            for (auto& pt: target_points.second){
              /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Point: [" << pt.x << "," << pt.y << "]."); */
              e::Vector3d curr_direction_local = directionFromCamPoint(cv::Point2d(pt.x, pt.y), image_index);
              /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Line direction in camera: [" << curr_direction_local.transpose() << "]."); */
              /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Camera resolution: [" << cameras_[image_index].oc_model.width << "," << cameras_[image_index].oc_model.height << "]"); */
                    

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
              //TODO: initialize fd_[target]
              //TODO: mutex fd_[target]

              // Apply the correction step for line
              /* fd_[target].filter_state = filter_.correctLine(fd_[target].filter_state, camera_origin, curr_direction, LINE_VARIANCE, true); //true should activate Masreliez uniform filtering */
              /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Line direction in output: [" << curr_direction.transpose() << "]."); */
              fd_[target].filter_state = filter_->correctLine(fd_[target].filter_state, camera_origin, curr_direction, LINE_VARIANCE);

              // Restrict state to be in front of the camera
              double dist_range_span = (RANGE_MAX-RANGE_MIN)/2.0;
              double dist_range_center = RANGE_MIN+dist_range_span;
              /* fd_[target].filter_state = filter_.correctPlane(fd_[target].filter_state, camera_origin+(curr_direction*dist_range_center), curr_direction, dist_range_span, true); //true should activate Masreliez uniform filtering */
              fd_[target].filter_state = filter_->correctPlane(fd_[target].filter_state, camera_origin+(curr_direction*dist_range_center), curr_direction, dist_range_span);


              fd_[target].received_bearing = true;
            }
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
      std::vector<std::pair<int,std::vector<uvdar_core::Point2DWithFloat>>> associateImagePointsToTargets(uvdar_core::ImagePointsWithFloatStamped msg, int image_index){
        std::vector< std::pair<int, std::vector<uvdar_core::Point2DWithFloat>>> output;
        for (auto &pt : msg.points){
          int ID = targetIDFromUVDAR(pt.value);
          if (ID < 0){
            ROS_WARN_STREAM_THROTTLE(1.0,"[UWB_UVDAR_Fuser]: Observed point [" << pt.x << ", " << pt.y << "] with ID: " << pt.value << " did not match any target!");
            continue;
          }
          int i=0;
          bool found_filter = false;
          for (auto &f: fd_){
            if (f.id == ID){
              if (measurementCompatible(f, pt, image_index))
              {
                found_filter = true;
                bool found_association_group = false;
                for (auto &o : output){
                  if (o.first == i){
                    o.second.push_back(pt);
                    found_association_group = true;
                  }
                }
                if (!found_association_group){
                  output.push_back({i,{pt}});
                }

              }
            }
            i++;
          }
          if (!found_filter){
            if ( initiateNew(pt, msg.stamp))
              output.push_back({i, {pt}});
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
        for (auto tg : _targets_){
          for (auto mid : tg.uvdar_ids){
            if (mid == uvdar_id){
              return tg.id;
            }
          }
        }
        return -1; //not found
      }
      //}

      /**
       * @brief Associates the blinking sequence ID retrieved by UWB with a specific target
       *
       * @param uvdar_id - the UWB ID provided by receiver
       *
       * @return An ID of the target that is expected to carry the uvdar_id
       *
       * targetIDFromUWB //{ */
      int targetIDFromUWB(double uwb_id){
        for (auto tg : _targets_){
          if (uwb_id == tg.uwb_id){
            return tg.id;
          }
        }
        return -1; //not found
      }
      //}

      /**
       * @brief Callback to process input range measurement messges
       *
       * @param msg
       */
      /* ProcessRanges //{ */
      void ProcessRanges(const mrs_msgs::RangeWithCovarianceArrayStampedConstPtr& msg) {
        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Getting ranges.");
        std::scoped_lock lock(filter_mutex);

        e::Vector3d receiver_origin;
        {
          std::scoped_lock lock(transformer_mutex);

          auto fromrec_tmp = transformer_.getTransform(_uwb_frame_,_uav_name_+"/"+_output_frame_, msg->header.stamp);
          if (!fromrec_tmp){
            ROS_ERROR_STREAM("[UWB_UVDAR_Fuser]: Could not obtain transform from " << _uwb_frame_  << " to " << _uav_name_+"/"+_output_frame_ << "!");
            return;
          }

          auto receiver_origin_tmp = transformer_.transform(e::Vector3d(0,0,0), fromrec_tmp.value());
          if (!receiver_origin_tmp){
            ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: Failed to transform UWB device origin! Returning.");
            return;
          }
          else
            receiver_origin = receiver_origin_tmp.value();
        }

        for (auto r: msg->ranges){
          int uwb_id = r.id;
          int ID = targetIDFromUWB(uwb_id);
          ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Getting range: " << r.range << ", ID: " << r.id);
          if (ID < 0){
            ROS_WARN_STREAM("[UWB_UVDAR_Fuser]: Observed range [" << r.range.range << "] with ID: " << r.id << " did not match any target!");
            continue;
          }
          double range = r.range.range;
          double variance = r.variance;
          bool found_filter = false;
          for (auto &f: fd_){
            if (f.id == ID){
              if (f.received_bearing){
                e::Vector3d direction = f.filter_state.x.topLeftCorner(3,1).normalized();
                ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Fusing distance measurement with distance of " << range << " with assumed direction of [ " << direction.transpose() << "].");
                f.filter_state = filter_->correctPlane(f.filter_state, receiver_origin+(direction*range), direction, variance);
              }
            }
          }
          if (!found_filter){
            initiateNew(r, msg->header.stamp);
          }


        }
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
        for (int target=0; target<(int)(fd_.size());target++){
          /* int targetsSeen = 0; */
          double age = (ros::Time::now() - fd_[target].latest_measurement).toSec();
          if (_debug_)
            ROS_INFO("[UWB_UVDAR_Fuser]: Age of %d is %f", target, age);
          double decay_age;
          if (fd_[target].update_count > MIN_MEASUREMENTS_TO_VALIDATION){
            decay_age = DECAY_AGE_NORMAL;
          }
          else {
            decay_age = DECAY_AGE_UNVALIDATED;
          }
          if (age>decay_age){
            ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Removing state " << target << " (ID:" << fd_[target].id << ") : " << fd_[target].filter_state.x.transpose() << " due to old age of " << age << " s." );
            fd_.erase(fd_.begin()+target);
            target--;
            continue;
          }
          predictTillTime(fd_.at(target), ros::Time::now(), true);

          if (_debug_){
            ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: State: ");
            ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: \n" << fd_[target].filter_state.x);
          }


          /* if ( */
          /*     ((fd_[target].filter_state.P.diagonal().array() < 0).any()) */
          /*    ){ */
          /*   ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: SPIN: NEGATIVE NUMBERS ON MAIN DIAGONAL!"); */
          /*   ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: State. cov: " << std::endl << fd_[target].filter_state.P); */
          /* } */
        }

        publishStates();

      }
      //}

      /**
       * @brief Initiates a new filter based on a blinker point measurement
       *
       * @param pt - a single blinker point extracted from image
       * @param stamp Time of the measurement
       */
      /* initiateNew //{ */
      bool initiateNew(uvdar_core::Point2DWithFloat pt, ros::Time stamp){
        if (fd_.size() > MAX_TARGET_COUNT)
          return false;

        int ID = targetIDFromUVDAR(pt.value);
        if (ID < 0){
          ROS_ERROR_STREAM("[UWB_UVDAR_Fuser]: Observed point [" << pt.x << ", " << pt.y << "] with ID: " << pt.value << " did not match any target!");
          return false ;
        }


        if (!_use_velocity_){
          int index = (int)(fd_.size());
          ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Initiating state " << index << " (ID:" << ID << ").");
          ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: The source input of the state is " << (ros::Time::now() - stamp).toSec() << "s old.");

          e::Matrix6d C_large;
          C_large << e::Matrix3d::Identity()*666, e::Matrix3d::Zero(), e::Matrix3d::Zero(), e::Matrix3d::Identity()*666;
          e::Vector6d zero_state = e::Vector6d::Zero();


          fd_.push_back({.filter_state = {.x = zero_state, .P = C_large}, .update_count = 0, .latest_update = stamp, .latest_measurement = stamp, .id = ID});
          return true;

        }
        else{
          ROS_WARN("[UWB_UVDAR_Fuser]: Initialization of states with velocity is not implemented. Returning.");
          return false;
        }

        return false;
      }
      //}

      /**
       * @brief Initiates a new filter based on a blinker point measurement
       *
       * @param r - range message from which to initialize
       * @param stamp Time of the measurement
       */
      /* initiateNew //{ */
      bool initiateNew(mrs_msgs::RangeWithCovarianceIdentified r, ros::Time stamp){
        if (fd_.size() > MAX_TARGET_COUNT)
          return false;

        int ID = targetIDFromUWB(r.id);
        if (ID < 0){
          ROS_ERROR_STREAM_THROTTLE(1.0,"[UWB_UVDAR_Fuser]: Observed range [" << r.range.range << "] with ID: " << r.id << " did not match any target!");
          return false ;
        }


        if (!_use_velocity_){
          int index = (int)(fd_.size());
          ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Initiating state " << index << " (ID:" << ID << ").");
          ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: The source input of the state is " << (ros::Time::now() - stamp).toSec() << "s old.");

          e::Matrix6d C_large;
          C_large << e::Matrix3d::Identity()*666, e::Matrix3d::Zero(), e::Matrix3d::Zero(), e::Matrix3d::Identity()*666;
          e::Vector6d zero_state = e::Vector6d::Zero();

          fd_.push_back({.filter_state = {.x = zero_state, .P = C_large}, .update_count = 0, .latest_update = stamp, .latest_measurement = stamp, .id = ID});
          return true;

        }
        else{
          ROS_WARN("[UWB_UVDAR_Fuser]: Initialization of states with velocity is not implemented. Returning.");
          return false;
        }

        return false;
      }
      //}
      
      bool measurementCompatible(FilterData filter, uvdar_core::Point2DWithFloat pt, int image_index){
        std::scoped_lock lock(transformer_mutex);


        LineProperties projection_line;

        auto camera_origin_tmp = transformer_.transform(e::Vector3d(0,0,0), cameras_[image_index].fromcam_tf);
        if (!camera_origin_tmp){
          ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: Failed to transform camera origin! Returning.");
          return false;
        }
        else
          projection_line.origin = camera_origin_tmp.value();

        e::Vector3d line_direction_local = directionFromCamPoint(cv::Point2d(pt.x, pt.y), image_index);
        e::Vector3d curr_direction; //global
        auto line_direction_tmp = transformer_.transformAsVector(line_direction_local, cameras_[image_index].fromcam_tf);
        if (!line_direction_tmp){
          ROS_ERROR_STREAM("[" << ros::this_node::getName().c_str() << "]: Failed to transform direction vector! Returning.");
          return false;
        }
        else
          projection_line.direction = line_direction_tmp.value();

        double mahalanobis_distance = mahalanobisDistance(filter.filter_state, projection_line);

        /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Mean: " << filter.filter_state.x.transpose()); */
        /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Mahalanobis distance: " << mahalanobis_distance); */

        return (mahalanobis_distance < LINE_MAH_THRESH);
      }

      double mahalanobisDistance(statecov_t X, LineProperties line){
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(X.P.topLeftCorner(3,3));
        Eigen::Matrix3d sphere_transform_inverse = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal(); 
        Eigen::Matrix3d sphere_transform = sphere_transform_inverse.inverse(); //transforms the Covariance to unit spherical - in the resulting space we compute the distance of the line to the estimate, corresponding to the Mahalanobis distance of the closest point on the line to the estimate mean
        e::Vector3d d = (sphere_transform*line.direction).normalized();
        e::Vector3d o = sphere_transform*line.origin;
        e::Vector3d m = sphere_transform*X.x.topLeftCorner(3,1);
        e::Vector3d W = m-o;
        e::Vector3d closest_point = o+d*(W.dot(d));
        /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Origin: " << o.transpose()); */
        /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Mean: " << m.transpose()); */
        /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Dist: " << W.transpose()); */
        /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Direction: " << d.transpose()); */
        /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Scale: " << (W.dot(d))); */
        /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Closest point: " << closest_point.transpose()); */
        return (m-closest_point).norm();
      }

      double mahalanobisDistance(statecov_t X, e::Vector3d p){
        e::Matrix3d Sinv = X.P.topLeftCorner(3,3).inverse();
        e::Vector3d diff = p-X.x.topLeftCorner(3,1);
        return sqrt(diff.transpose()*Sinv*diff);
      }


      /**
       * @brief Predicts the filter state at target_time, given that target_time is newer than the last update or measurement
       *
       * @param fd_ The filter data structure to start from (if apply_update is true, this structure will be updated with the result)
       * @param target_time The time to which the state is to be predicted
       * @param apply_update If true, the output will be applied to the structure fd_. This provides more data than the return value, such as the update time
       *
       * @return The resulting state and its output covariance
       */
      /* predictTillTime //{ */
      statecov_t predictTillTime(struct FilterData &fd_, ros::Time target_time, bool apply_update = false){
        auto orig_state = fd_;
        double dt_from_last = std::fmax(std::fmin((target_time-fd_.latest_update).toSec(),(target_time-fd_.latest_measurement).toSec()),0.0);
        filter_->A = A_dt(dt_from_last);
        /* ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Filter pred. orig: " << std::endl << fd_.filter_state.P); */
        auto new_state =  filter_->predict(fd_.filter_state, u_t(), Q_dt(dt_from_last), dt_from_last);
        /* ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Filter predicted: " << std::endl << fd_.filter_state.P); */
        if (apply_update){
          fd_.filter_state = new_state;
          fd_.latest_update = target_time;
        }

        if (
            ((orig_state.filter_state.P.diagonal().array() < 0).any()) ||
            ((fd_.filter_state.P.diagonal().array() < 0).any())
           ){
          ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: NEGATIVE NUMBERS ON MAIN DIAGONAL!");
          ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Orig. state. cov: " << std::endl << orig_state.filter_state.P);
          /* ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: dt: " << dt_from_last); */
          /* ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Q_dt: " << Q_dt(dt_from_last)); */
          ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: New state. cov: " << std::endl << fd_.filter_state.P);
        }
        return new_state;
      }
      //}

      /**
       * @brief Predicts the change of the state and error covariance till meas_time, and then corrects the state with measurement
       *
       * @param fd_ The filter data structure to start from (if apply_update is true, this structure will be updated with the result)
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
          filter_local.filter_state = filter_->correct(filter_local.filter_state, measurement.x, P_local);
        }
        catch ([[maybe_unused]] std::exception& e) {
          ROS_ERROR_STREAM("[UWB_UVDAR_Fuser]: Attempted to correct with bad covariance (match_level_pos = " << match_level_pos << "). Will replace with big, but manageable one.");
          /* P_local = e::MatrixXd(6,6); */
          P_local.setIdentity();
          P_local *= 10000;
          filter_local.filter_state = filter_->correct(filter_local.filter_state, measurement.x, P_local);
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
        fd_.erase(fd_.begin()+index);
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

        auto ret = transformer_.transformSingle(target_filter, camera_frame);
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
        for (auto const& fd_curr : fd_ ){
          temp.id = fd_curr.id;

          temp.pose.position.x = fd_curr.filter_state.x[0];
          temp.pose.position.y = fd_curr.filter_state.x[1];
          temp.pose.position.z = fd_curr.filter_state.x[2];

          qtemp = e::AngleAxisd(fd_curr.filter_state.x[3], e::Vector3d::UnitX()) * e::AngleAxisd(fd_curr.filter_state.x[4], e::Vector3d::UnitY()) * e::AngleAxisd(fd_curr.filter_state.x[5], e::Vector3d::UnitZ());

          temp.pose.orientation.x = qtemp.x();
          temp.pose.orientation.y = qtemp.y();
          temp.pose.orientation.z = qtemp.z();
          temp.pose.orientation.w = qtemp.w();

          /* for (int m=0; m<6; m++){ */
          /*   for (int n=0; n<6; n++){ */
          /*     temp.covariance[6*n+m] =  fd_curr.value().filter_state.P(n,m); */
          /*   } */
          /* } */
          temp.covariance = eigenCovarianceToRos(fd_curr.filter_state.P);

          if (fd_curr.update_count < MIN_MEASUREMENTS_TO_VALIDATION){
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
        for (int i=0; i<(int)(fd_.size());i++){
          if ( (fd_[i].filter_state.x.array().isNaN().any() ) ||
              (fd_[i].filter_state.P.array().isNaN().any() )){
            fd_.erase(fd_.begin()+i);
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
        for (int i=0; i<((int)(fd_.size())-1);i++){
          bool remove_first = false;
          for (int j=i+1; j<(int)(fd_.size()); j++){
            curr_match_level = gaussJointMaxVal(
                fd_[i].filter_state.P.topLeftCorner(3,3),
                fd_[j].filter_state.P.topLeftCorner(3,3),
                fd_[i].filter_state.x.topRows(3),
                fd_[j].filter_state.x.topRows(3)
                );
            if (curr_match_level > MATCH_LEVEL_THRESHOLD_REMOVE){
              auto eigens = fd_[i].filter_state.P.topLeftCorner(3,3).eigenvalues();
              double size_i = (eigens.topLeftCorner(3, 1)).norm();
              eigens = fd_[j].filter_state.P.topLeftCorner(3,3).eigenvalues();
              double size_j = (eigens.topLeftCorner(3, 1)).norm();
              int n = -1;
              int m = -1;
              if ( (fd_[i].update_count < MIN_MEASUREMENTS_TO_VALIDATION) == (fd_[j].update_count < MIN_MEASUREMENTS_TO_VALIDATION) ){
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
                if (fd_[i].update_count < MIN_MEASUREMENTS_TO_VALIDATION){
                  n = i;
                  m = j;
                }
                else {
                  n = j;
                  m = i;
                }

              }
              fd_.erase(fd_.begin()+n);
              ROS_INFO_STREAM("[UWB_UVDAR_Fuser]: Removing state " << n << " (ID:" << fd_[n].id << "): " << fd_[n].filter_state.x.transpose() << " due to large overlap with state " << m << ": " << fd_[m].filter_state.x.transpose());
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
        cam2world(v_w_raw, v_i, &(cameras_[image_index].oc_model));
        return e::Vector3d(v_w_raw[1], v_w_raw[0], -v_w_raw[2]);
      }
      //}
      
      bool loadTargetIdentifiers(mrs_lib::ParamLoader &pl){
        const auto tgids_xml = pl.loadParam2<XmlRpc::XmlRpcValue>("targets");
        const auto tgids_opt = parse_target_ids(tgids_xml);
        
        if (!tgids_opt) {
          ROS_ERROR("[UWB_UVDAR_Fuser]: Some target identifiers could not be obtained successfully!");
          return false;
        }
        _targets_ = tgids_opt.value();
        return true;
      }

      /* parse_target_ids() method //{ */
      std::optional<std::vector<TargetIdentifiers>> parse_target_ids(const XmlRpc::XmlRpcValue& xmlarr) const {
        const static std::string root_frame_xmlname  = "root_frame_id";
        const static std::string equal_frame_xmlname = "equal_frame_id";
        const static std::string offsets_xmlname     = "offsets";
        if (xmlarr.getType() != XmlRpc::XmlRpcValue::TypeArray) {
          ROS_ERROR("[UWB_UVDAR_Fuser]: The 'targets' parameter has to be array, but it's not. Cannot parse.");
          return std::nullopt;
        }

        std::vector<TargetIdentifiers> output;
        output.reserve(xmlarr.size());

        int i = 0;
        for (size_t it = 0; it < (size_t)(xmlarr.size()); it++) {
          const auto& tgid_xml = xmlarr[it];
          if (tgid_xml.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR_STREAM("[UWB_UVDAR_Fuser]: Invalid type of the " << it << ". member of 'targets'. Cannot parse.");
            return std::nullopt;
          }

          if (!tgid_xml.hasMember("uvdar_ids") || !tgid_xml.hasMember("uwb_id")) {
            ROS_ERROR_STREAM("[UWB_UVDAR_Fuser]: The " << it << ". member of 'targets' is missing either the 'uvdar_ids' or 'uwb_id' member. Cannot parse.");
            return std::nullopt;
          }

          bool parsed_uwb_id  = false;
          bool parsed_uvdar_ids = false;

          int uwb_id = -1;
          std::vector<int> uvdar_ids = {};

          for (auto mem_it = std::cbegin(tgid_xml); mem_it != std::cend(tgid_xml); ++mem_it) {
            const auto& mem_name = mem_it->first;
            const auto& mem      = mem_it->second;
            // firstly, check types
            if ((mem_name == "uvdar_ids" && mem.getType() != XmlRpc::XmlRpcValue::TypeArray) ||
                (mem_name == "uwb_id" && mem.getType() != XmlRpc::XmlRpcValue::TypeInt)) {
              ROS_ERROR_STREAM("[UWB_UVDAR_Fuser]: The " << it << ". member of 'targets' has a wrong type of the '" << mem_name
                  << "' member. Cannot parse.");
              return std::nullopt;
            }


            if (mem_name == "uwb_id") {
              uwb_id = num(mem);
              parsed_uwb_id = true;
            }
            else if (mem_name == "uvdar_ids") {
              if (mem.size() < 1){
                continue;
              }
              for (int i = 0; i < (int)(mem.size()); i++){
                uvdar_ids.push_back(num(mem[i]));
              }
              parsed_uvdar_ids = true;
            } else {
              ROS_ERROR_STREAM("[UWB_UVDAR_Fuser]: The " << it << ". member of 'targets' has an unexpected member '" << mem_name << "'. Aborting parse.");
              return std::nullopt;
            }
          }

          if (!parsed_uwb_id || !parsed_uvdar_ids) {
            ROS_ERROR_STREAM("[UWB_UVDAR_Fuser]: The " << it << ". member of 'targets' misses a compulsory member 'uvdar_ids' or 'uwb_id'. Aborting parse.");
            return std::nullopt;
          }


          output.push_back({.id=i,.uvdar_ids=uvdar_ids, .uwb_id=uwb_id});
          i++;
        }

        return output;
      }
//}

      double num(const XmlRpc::XmlRpcValue& xml) const {
        switch (xml.getType()) {
          case XmlRpc::XmlRpcValue::TypeInt:
            return (int)xml;
          case XmlRpc::XmlRpcValue::TypeDouble:
            return (double)xml;
          default:
            return std::numeric_limits<double>::quiet_NaN();
    }
  }
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
