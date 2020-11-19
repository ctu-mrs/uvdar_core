#include <ros/ros.h>
#include <ros/package.h>

#include <thread>
#include <mutex>
#include <numeric>
#include <fstream>
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

#define MAX_DIST_INIT 100.0
#define BRACKET_STEP 10
#define QPIX 1 //pixel std. dev
#define TUBE_LENGTH (_beacon_?10:1000)


#define sqr(X) ((X) * (X))
#define cot(X) (cos(X)/sin(X))
#define deg2rad(X) ((X)*0.01745329251)
#define rad2deg(X) ((X)*57.2957795131)

#define DIRECTIONAL_LED_VIEW_ANGLE (deg2rad(120))
#define RING_LED_VERT_ANGLE (deg2rad(120))

#define UNMATCHED_OBSERVED_POINT_PENALTY sqr(10)

#define LED_GROUP_DISTANCE 0.03

#define ERROR_THRESHOLD 60

namespace e = Eigen;

namespace uvdar {


  /**
   * @brief A processing class for converting retrieved blinking markers from a UV camera image into relative poses of observed UAV that carry these markers, as well as the error covariances of these estimates
   */
  class UVDARPoseCalculator {
    struct LEDMarker {
      e::Vector3d position;
      e::Quaterniond orientation;
      int type; // 0 - directional, 1 - omni ring, 2 - full omni
      int freq_id;
    };

    class LEDModel {
      private:
      std::vector<LEDMarker> markers;
      std::vector<std::vector<int>> groups;

      public:
      LEDModel(){
      }

      LEDModel(std::string model_file){
        parseModelFile(model_file);

        prepareGroups();
      }

      LEDModel translate(e::Vector3d position){
        LEDModel output(this);
        for (auto &marker : output.markers){
          marker.position += position;
          /* output.push_back(marker); */
        }
        return output;
      }

      LEDModel rotate(e::Vector3d center, e::Vector3d axis, double angle){
        e::Quaterniond rotation(e::AngleAxisd(angle, axis));
        return rotate(center, rotation);
      }

      LEDModel rotate(e::Vector3d center, e::Quaterniond orientation){
        LEDModel output(this);
        for (auto &marker : output.markers){
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: pos a: " << marker.position); */
          marker.position -= center;
          marker.position = orientation*marker.position;
          marker.position += center;
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: pos b: " << marker.position); */
          marker.orientation *= orientation;
          /* output.push_back(marker); */
        }
        return output;
      }

      std::pair<double,double> getMaxMinVisibleDiameter(){
        if ((int)(groups.size()) == 0){
          return {-1,-1};
        }

        if ((int)(groups.size()) == 1){
          return {std::numeric_limits<double>::max(), 0};
        }

        double max_dist = 0;
        double min_dist = std::numeric_limits<double>::max();
        for (int i=0; i<((int)(groups.size())-1); i++){
          for (int j=i+1; j<(int)(groups.size()); j++){
            if (areSimultaneouslyVisible(markers[groups[i][0]],markers[groups[j][0]])){
              double tent_dist = (markers[groups[i][0]].position - markers[groups[j][0]].position).norm();
              /* ROS_ERROR_STREAM("[UVDARPoseCalculator]: visible dist: " << tent_dist); */
              if (tent_dist > max_dist){
                max_dist = tent_dist;
              }
              if (tent_dist < min_dist){
                min_dist = tent_dist;
              }
            }
          }
        }
        /* if (max_dist < 0.0001){ */
        /*   ROS_ERROR_STREAM("[UVDARPoseCalculator]: Greatest visible diameter of the model evaluated as ZERO!"); */
        /* } */
        /* if (min_dist < 0.0001){ */
        /*   ROS_ERROR_STREAM("[UVDARPoseCalculator]: Smallest visible diameter of the model evaluated as ZERO!"); */
        /* } */
        return {max_dist,min_dist};
      }

      bool areSimultaneouslyVisible(LEDMarker a, LEDMarker b){
        if ((a.type == 0) && (b.type == 0)){
          double angle_between = a.orientation.angularDistance(b.orientation);
          if (angle_between < DIRECTIONAL_LED_VIEW_ANGLE){
            return true;
          }
          else{
            return false;
          }
        }
        else{
          ROS_ERROR_STREAM("[UVDARPoseCalculator]: Non-directional markers not yet implemented!");
          return false;
        }
      }


      std::size_t size(){
        return markers.size();
      }
      void push_back(LEDMarker marker){
        markers.push_back(marker);
      };
      const LEDMarker operator[](std::size_t idx) const { return markers.at(idx); }
      inline std::vector<LEDMarker>::iterator begin() noexcept { return markers.begin(); }
      inline std::vector<LEDMarker>::iterator end() noexcept { return markers.end(); }

      private:

      LEDModel(LEDModel* input){
        markers = input->markers;
        groups = input->groups;

      }

      void prepareGroups(){
        std::vector<bool> flagged(markers.size(),false);
        int i = 0;
        for (auto marker : markers){
          bool found = false;
          for (auto &group : groups){
            if ((marker.position - markers.at(group.at(0)).position).norm() < LED_GROUP_DISTANCE){
              group.push_back(i);
              found = true;
            }
          }
          if (!found){
            groups.push_back(std::vector<int>(1,i));
          }

          i++;
        }


      }

      bool parseModelFile(std::string model_file){
            ROS_INFO_STREAM("[UVDARPoseCalculator]: Loading model from file: [ " + model_file + " ]");
        std::ifstream ifs;
        ifs.open(model_file);
        std::string word;
        std::string line;

        LEDMarker curr_lm;

        if (ifs.good()) {
          while (getline( ifs, line )){
            if (line[0] == '#'){
              continue;
            }
            std::stringstream iss(line); 
            double X, Y, Z;
            iss >> X;
            iss >> Y;
            iss >> Z;
            curr_lm.position = e::Vector3d(X,Y,Z);
            int type;
            iss >> type;
            curr_lm.type = type;
            double pitch, yaw;
            iss >> pitch;
            iss >> yaw;
            tf::Quaternion qtemp;
            qtemp.setRPY(0, pitch, yaw);
            curr_lm.orientation.x() = qtemp.x();
            curr_lm.orientation.y() = qtemp.y();
            curr_lm.orientation.z() = qtemp.z();
            curr_lm.orientation.w() = qtemp.w();
            int freq_id;
            iss >> freq_id;
            curr_lm.freq_id = freq_id;

            markers.push_back(curr_lm);
            ROS_INFO_STREAM("[UVDARPoseCalculator]: Loaded Model: [ X: " << X <<" Y: "  << Y << " Z: "  << Z << " type: " << type << " pitch: " << pitch << " yaw: " << yaw << " freq_id: " << freq_id << " ]");
          }
        ifs.close();
        }
        else {
          ROS_ERROR_STREAM("[UVDARPoseCalculator]: Failed to load model file " << model_file << "! Returning.");
        ifs.close();
          return false;
        }
        return true;
      }
    };

    public:

      /**
       * @brief Constructor - loads parameters and initializes necessary structures
       *
       * @param nh Private NodeHandle of this ROS node
       */
      /* Constructor //{ */
      UVDARPoseCalculator(ros::NodeHandle& nh) {
        ROS_INFO("[UVDARPoseCalculator]: Initializing pose calculator...");

        mrs_lib::ParamLoader param_loader(nh, "UVDARPoseCalculator");

        param_loader.loadParam("uav_name", _uav_name_, std::string());

        param_loader.loadParam("debug", _debug_, bool(false));

        param_loader.loadParam("gui", _gui_, bool(false));
        param_loader.loadParam("publish_visualization", _publish_visualization_, bool(false));

        param_loader.loadParam("quadrotor",_quadrotor_,bool(false));

        param_loader.loadParam("beacon",_beacon_,bool(false));

        param_loader.loadParam("custom_model",_custom_model_,bool(false));
        param_loader.loadParam("model_file",_model_file_, std::string(""));
        
        prepareModel();


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
              nh.subscribe(_blinkers_seen_topics[i], 1, cals_blinkers_seen_[i]));

          ROS_INFO_STREAM("[UVDARPoseCalculator]: Advertising measured poses " << i+1);
          pub_measured_poses_.push_back(nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("measuredPoses"+std::to_string(i+1), 1)); 


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
              nh.subscribe(_estimated_framerate_topics[i], 1, cals_estimated_framerate_[i]));
        }
        //}

        /* X2 = Eigen::VectorXd(9,9); */
        /* X2q = Eigen::VectorXd(8,8); */
        /* X2qb = Eigen::VectorXd(8,8); */
        /* X3 = Eigen::VectorXd(10,10); */
        /* X3qb = Eigen::VectorXd(9,9); */
        /* X4qb = Eigen::VectorXd(12,12); */
        /* Px2 = Eigen::MatrixXd(9,9); */
        /* Px2q = Eigen::MatrixXd(8,8); */
        /* Px2qb = Eigen::MatrixXd(8,8); */
        /* Px3 = Eigen::MatrixXd(10,10); */
        /* Px3qb = Eigen::MatrixXd(9,9); */
        /* Px4qb = Eigen::MatrixXd(12,12); */


        /* Set transformation frames for cameras //{ */
        param_loader.loadParam("camera_frames", _camera_frames_, _camera_frames_);
        if (_camera_frames_.size() != _blinkers_seen_topics.size()) {
          ROS_ERROR_STREAM("The size of camera_frames (" << _camera_frames_.size() << 
              ") is different from blinkers_seen_topics size (" << _blinkers_seen_topics.size() << ")");
        }
        //}

        if (_gui_ || _publish_visualization_){
          if (_publish_visualization_){
            pub_visualization_ = std::make_unique<mrs_lib::ImagePublisher>(boost::make_shared<ros::NodeHandle>(nh));
          }
          timer_visualization_ = nh.createTimer(ros::Rate(1), &UVDARPoseCalculator::VisualizationThread, this, false);
        }


        initialized_ = true;
      }
      //}

      /* Destructor //{ */
      ~UVDARPoseCalculator() {
      }
      //}

    private:

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
            file_name = ros::package::getPath("uvdar_core")+"/include/OCamCalib/config/calib_results_bf_uv_fe.txt";
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
       * @brief Load the target model files used for calculating relative poses
       *
       * @return success
       */
      /* prepareModel //{ */
      bool prepareModel(){
        std::string file_name;
        if (!_custom_model_){
          if (_quadrotor_){
            if (_beacon_){
              file_name = ros::package::getPath("uvdar_core")+"/config/models/quadrotor_beacon.txt";
            }
            else {
              file_name = ros::package::getPath("uvdar_core")+"/config/models/quadrotor.txt";
            }
          }
          else {
            file_name = ros::package::getPath("uvdar_core")+"/config/models/hexarotor.txt";
          }
        }
        else {
            file_name = _model_file_;
        }
        model_ = LEDModel(file_name);
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
        if (!initialized_){
          return;
        }
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


      /* Specific calculations used for individual cases of UAV models detection of various numbers of their markers//{ */

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
        /* separated_points.resize(_target_count_); */


        for (int i = 0; i < (int)(points.size()); i++) {
          if (points[i].z > 1) {
            int mid = ufc_->findMatch(points[i].z);
            int tid = classifyMatch(mid);
            if (_debug_)
              ROS_INFO("[%s]: FR: %d, MID: %d, TID: %d", ros::this_node::getName().c_str(),(int)points[i].z, mid, tid);
            if (tid>=0){
              int index = -1;
              int j=0;
              for (auto& sep_pts : separated_points){
                if (sep_pts.first == tid){
                  index = j;
                  break;
                }
                j++;
              }
              if (index < 0){
                separated_points.push_back({tid,{points[i]}});
              }
              else {
                separated_points[index].second.push_back(points[i]);
              }

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

        /* double leftF; */
        /* double rightF; */
        /* if (_beacon_){ */
        /*   leftF = _frequencies_[1]; */
        /*   if (_frequencies_.size() == 3) */
        /*     rightF = _frequencies_[2]; */
        /*   else */
        /*     rightF = _frequencies_[1]; */
        /* } */
        /* else { */
        /*   leftF = _frequencies_[target*2]; */
        /*   rightF = _frequencies_[target*2+1]; */
        /* } */
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

                  /* if (_debug_) */
                  /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: Distance: " <<cv::norm(cv::Point2i(points[i].x,points[i].y) - cv::Point2i(points[j].x,points[j].y)) << " max_dist: " << max_dist); */

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

        auto furthest_position = getRoughInit(model_, points, image_index);
        if (_debug_){
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Furhtest possible distance: " << furthest_position);
        }

        auto [hypotheses, errors] = getViableInitialHyptheses(model_, points, furthest_position, target, image_index);

        /* auto fitted_position = iterFitPosition(model_, points, rough_initialization, target,  image_index); */
        /* if (_debug_){ */
        /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: Fitted position: " << fitted_position.transpose()); */
        /* } */

          ROS_INFO_STREAM("[UVDARPoseCalculator]: Rough hypotheses for target " << target << " in image " << image_index << ": ");
          int i = 0;
        for (auto h: hypotheses){
          ROS_INFO_STREAM("x: [" << h.first.transpose() << "] rot: [" << rad2deg(rotmatToYaw(h.second.toRotationMatrix())) << "] with error of " << errors.at(i++));
        }

        auto fitted_pose = iterFitFull(model_, points, hypotheses, target,  image_index);
        if (_debug_){
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Fitted pose: " << fitted_pose.first.transpose());
        }
        e::MatrixXd covariance = getCovarianceEstimate(model_, points, fitted_pose, target,  image_index);
        if (_debug_){
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Covariance: [\n" << covariance << "\n]");
        }
      
      /*   ms.x.topLeftCorner(3,1) = fitted_pose.first; */
      /*   ms.x.bottomLeftCorner(4,1) = fitted_pose.second; */
      /*   ms.C = covariance; */

        /* covariance += e::MatrixXd::Identity(6,6)*0.0001; */
        if (fitted_pose.first.norm() < 1.5) { //We don't expect to be able to measure relative poses of UAVs this close - the markers would be too bright and too far apart
          return;
        }

        e::Vector3d unit_vec;
        unit_vec << 0,0,1.0;
        if (acos(fitted_pose.first.normalized().dot(unit_vec)) > rad2deg(190)) //our lenses only allow us to see UAVs ~92.5 degrees away from the optical axis of the camera
          return;

        /* tf::Quaternion qtemp; */
        /* qtemp.setRPY((ms.x(3)), (ms.x(4)), (ms.x(5))); */
        /* qtemp=tf::Quaternion(-0.5,0.5,-0.5,-0.5)*qtemp; //bring relative orientations to the optical frame of the camera (Roll, Pitch and Yaw were estimated in the more intuitive sensor frame (X forward, Y to the left, Z up) */
        /* qtemp.normalize();//just in case */
        auto fitted_pose_optical = opticalFromBase(fitted_pose,covariance);

        output_pose.id = target;
        output_pose.pose.position.x = fitted_pose_optical.first.first.x();
        output_pose.pose.position.y = fitted_pose_optical.first.first.y();
        output_pose.pose.position.z = fitted_pose_optical.first.first.z();
        output_pose.pose.orientation.x = fitted_pose_optical.first.second.x();
        output_pose.pose.orientation.y = fitted_pose_optical.first.second.y();
        output_pose.pose.orientation.z = fitted_pose_optical.first.second.z();
        output_pose.pose.orientation.w = fitted_pose_optical.first.second.w();
        for (int i=0; i<fitted_pose_optical.second.cols(); i++){
          for (int j=0; j<fitted_pose_optical.second.rows(); j++){
            output_pose.covariance[fitted_pose_optical.second.cols()*j+i] =  fitted_pose_optical.second(j,i);
          }
        }

        if (_debug_){
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Y: \n" << fitted_pose_optical.first.first.transpose() << " : [" << fitted_pose_optical.first.second.x() << "," << fitted_pose_optical.first.second.y() << "," <<fitted_pose_optical.first.second.z() << "," <<fitted_pose_optical.first.second.w() << "]" );
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Py: \n" << fitted_pose_optical.second );
        }


      }
      //}

      /* std::vector<e::Vector3d> getRoughInit(LEDModel model, std::vector<cv::Point3d> observed_points, int image_index){ */
      e::Vector3d getRoughInit(LEDModel model, std::vector<cv::Point3d> observed_points, int image_index){

        std::vector<e::Vector3d> v_w;
        e::Vector3d v_avg = {0,0,0};
        for (auto& point: observed_points){
          v_w.push_back(directionFromCamPoint(point, image_index));
          v_avg += v_w.back();
        }
        v_avg /= (double)(v_avg.size());
        v_avg = v_avg.normalized();
        /* ROS_ERROR_STREAM("[UVDARPoseCalculator]: model size: " << model.size()); */
        /* for (sub_model : getSubModels(model, (int)(observed_points.size()))) { */
          /* double d_model = getRoughVisibleDiameter(sub_model); */
          auto [d_max, d_min] = model.getMaxMinVisibleDiameter();
          double alpha_max = getLargestAngle(v_w);
          double l_max = (d_max/2.0)/tan(alpha_max/2.0);
          ROS_INFO_STREAM("[UVDARPoseCalculator]: d_max: " << d_max << "; alpha_max: " << alpha_max << "; l_rough: " << l_max);
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: d_min: " << d_min << "; alpha_min: " << alpha_min << "; l_rough: " << l_max); */
          return baseFromOptical(v_avg*l_max);
        /* } */
      }


      double getLargestAngle(std::vector<e::Vector3d> directions){
        double max_angle = 0;
        for (int i = 0; i < ((int)(directions.size()) - 1); i++){
          for (int j = i+1; j < (int)(directions.size()); j++){
            double tent_angle = acos(directions[i].normalized().dot(directions[j].normalized()));
            if (tent_angle > max_angle){
              max_angle = tent_angle;
            }
          }
        }

        return max_angle;
      }

      /* std::vector<LEDModel> getSubModels(LEDModel full_model, int min_markers){ */
      /*   std::vector<LEDModel> output; */

      /* } */


      e::Vector3d iterFitPosition(LEDModel model, std::vector<cv::Point3d> observed_points, e::Vector3d rough_initialization, int target, int image_index){
        e::Vector3d position_curr = rough_initialization;
        auto model_curr = model.translate(position_curr);
        double step_init = 0.1;
        double step = step_init;
        /* double y_step = step_init; */
        /* double z_step = step_init; */
        double error_total = totalError(model_curr, observed_points, target, image_index);
        LEDModel shape_top, shape_bottom;
        LEDModel shape_left, shape_right;
        LEDModel shape_front, shape_back;
        double top_error, bottom_error;
        double left_error, right_error;
        double front_error, back_error;
        auto gradient = e::Vector3d (
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max()
            );
        double threshold = (int)(observed_points.size())*0.001;
        int iters = 0;
        ROS_INFO_STREAM("[UVDARPoseCalculator]: total error init: " << error_total);
        while ((error_total > threshold) && ((gradient.norm()) > 0.0001) && (iters < 50)){
          step = step_init;
          while (true){
            shape_top     = model_curr.translate(e::Vector3d(0,0,step));
            shape_bottom  = model_curr.translate(e::Vector3d(0,0,-step));
            shape_left    = model_curr.translate(e::Vector3d(0,step,0));
            shape_right   = model_curr.translate(e::Vector3d(0,-step,0));
            shape_front   = model_curr.translate(e::Vector3d(step,0,0));
            shape_back    = model_curr.translate(e::Vector3d(-step,0,0));
            top_error = totalError(shape_top, observed_points, target, image_index);
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: top: " << top_error); */
            bottom_error = totalError(shape_bottom, observed_points, target, image_index);
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: bottom: " << bottom_error); */
            if ( (step > 0.0001) && (sgn(bottom_error - error_total) == sgn(top_error - error_total))) {
              step /= 2;
              continue;
            }
            left_error = totalError(shape_left, observed_points, target, image_index);
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: left: " << left_error); */
            right_error = totalError(shape_right, observed_points, target, image_index);
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: right:" << right_error); */
            if ( (step > 0.0001) && (sgn(left_error - error_total) == sgn(right_error - error_total))) {
              step /= 2;
              continue;
            }
            front_error = totalError(shape_front, observed_points, target, image_index);
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: front: " << front_error); */
            back_error = totalError(shape_back, observed_points, target, image_index);
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: back: " << back_error); */
            if ( (step > 0.0001) && (sgn(front_error - error_total) == sgn(back_error - error_total))) {
              step /= 2;
              continue;
            }

            break;
          }
          gradient.z() = ((top_error-bottom_error)/(2*step))/((double)(observed_points.size()));
          gradient.y() = ((left_error-right_error)/(2*step))/((double)(observed_points.size()));
          gradient.x() = ((front_error-back_error)/(2*step))/((double)(observed_points.size()));

          /* gradient.x() = gradient.x()*x_step; */
          /* gradient.y() = gradient.y()*y_step; */
          /* gradient.z() = gradient.z()*z_step; */

          /* if ( */ 
          /*     (gradient.x() > x_step) || */
          /*     (gradient.y() > y_step) || */
          /*     (gradient.z() > z_step) */
          /*    ){ */
            gradient = gradient.normalized()*step;
          /* } */

          double x_diff, y_diff, z_diff;
          if (abs(gradient.x()) > 1E-9)
            x_diff = -gradient.x();
          else
            x_diff = 0;
          if (abs(gradient.y()) > 1E-9)
            y_diff = -gradient.y();
          else
            y_diff = 0;
          if (abs(gradient.z()) > 1E-9)
            z_diff = -gradient.z();
          else
            z_diff = 0;


          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: gradient: " << gradient); */
          position_curr = position_curr + e::Vector3d(x_diff, y_diff, z_diff);
          model_curr = model_curr.translate(e::Vector3d(x_diff, y_diff, z_diff));

          error_total = totalError(model_curr, observed_points, target, image_index);
          if (_debug_){
            ROS_INFO_STREAM("[UVDARPoseCalculator]: total error: " << error_total;);
          }
          iters++;
        }
        //final gradient check
        double mean_error = error_total/((double)(observed_points.size()));
        step = 0.10;
        /* y_step = 0.10; */
        /* z_step = 0.10; */
        shape_top   = model_curr.translate(e::Vector3d(0,0,step));
        shape_bottom= model_curr.translate(e::Vector3d(0,0,-step));
        shape_left  = model_curr.translate(e::Vector3d(0,step,0));
        shape_right = model_curr.translate(e::Vector3d(0,-step,0));
        shape_front = model_curr.translate(e::Vector3d(step,0,0));
        shape_back  = model_curr.translate(e::Vector3d(-step,0,0));
        gradient.x() = ((((abs(front_error)+abs(back_error))/2)-mean_error)/(step))/((double)(observed_points.size()));
        gradient.y() = ((((abs(left_error)+abs(right_error))/2)-mean_error)/(step))/((double)(observed_points.size()));
        gradient.z() = ((((abs(top_error)+abs(bottom_error))/2)-mean_error)/(step))/((double)(observed_points.size()));

        return position_curr;
      }

      std::pair<std::vector<std::pair<e::Vector3d, e::Quaterniond>>,std::vector<double>> getViableInitialHyptheses(LEDModel model, std::vector<cv::Point3d> observed_points, e::Vector3d furthest_position, int target, int image_index, double init_dist_step_meters=2.0, int orientation_step_count=24){
        e::Vector3d first_position = 1.0*furthest_position.normalized();
        ROS_INFO_STREAM("[UVDARPoseCalculator]: Range: " << (furthest_position-first_position).norm());
        int dist_step_count = round((furthest_position-first_position).norm()/init_dist_step_meters);
        auto position_step = (furthest_position-first_position)/dist_step_count;

        double angle_step = 2.0*M_PI/(double)(orientation_step_count);

        std::vector<std::pair<e::Vector3d, e::Quaterniond>> acceptable_hypotheses;
        std::vector<double> errors;

        e::Vector3d position_curr = first_position;
        std::pair<double,double> best_orientation;
        for (int i=0; i<=dist_step_count; i++){
          best_orientation = {std::numeric_limits<double>::max(), -1};
          for (int j=0; j<orientation_step_count; j++){
            double error_total = totalError(model.rotate(e::Vector3d(0,0,0), e::Vector3d::UnitZ(), j*angle_step).translate(position_curr), observed_points, target, image_index);
            if (best_orientation.first > error_total){
              best_orientation.first = error_total;
              best_orientation.second = j*angle_step;
            }
          }

          /* if (best_orientation.first < (ERROR_THRESHOLD*(int)(observed_points.size()))){ */
          if (true){
            acceptable_hypotheses.push_back(std::pair<e::Vector3d, e::Quaterniond>(position_curr, e::AngleAxisd(best_orientation.second, e::Vector3d::UnitZ())));
            errors.push_back(best_orientation.first);
          }

          position_curr+=position_step;
        }

        return {acceptable_hypotheses, errors};
      }

      std::pair<e::Vector3d, e::Quaterniond> iterFitFull(LEDModel model, std::vector<cv::Point3d> observed_points, std::vector<std::pair<e::Vector3d, e::Quaterniond>> hypotheses, int target, int image_index){
        std::pair<e::Vector3d, e::Quaterniond> best_result;
        double best_error = std::numeric_limits<double>::max();

        ROS_INFO_STREAM("[UVDARPoseCalculator]: Refined hypotheses for target " << target << " in image " << image_index << ": ");
        int h = 0;
        for (auto hypothesis : hypotheses){
          e::Vector3d position_curr = hypothesis.first;
          e::Vector3d RPY_curr = e::Vector3d(0,0,0);
          e::Quaterniond orientation_start  = hypothesis.second;
          e::Quaterniond orientation_curr  = orientation_start;
          auto model_curr = model.translate(position_curr);

          double pos_step_init = 0.1;
          double angle_step_init = 0.1;
          double x_step     = 0.1;
          double y_step     = 0.1;
          double z_step     = 0.1;
          double angle_step   = 0.1;//rad
          /* double pitch_step = 0.1;//rad */
          /* double roll_step  = 0.1;//rad */

          double error_total = totalError(model_curr, observed_points, target, image_index);
          LEDModel shape_top, shape_bottom;
          LEDModel shape_left, shape_right;
          LEDModel shape_front, shape_back;
          LEDModel shape_ccw, shape_cw;
          LEDModel shape_pd, shape_pu;
          LEDModel shape_rr, shape_rl;
          double top_error, bottom_error;
          double left_error, right_error;
          double front_error, back_error;
          double ccw_error, cw_error; //yaw
          double pd_error, pu_error; //pitch
          double rr_error, rl_error; //roll
          e::VectorXd gradient = Eigen::VectorXd(6);
          gradient <<
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max();
          double threshold = (double)(observed_points.size())*sqr(QPIX);
          int iters = 0;
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: total error init: " << error_total); */
          while ((error_total > threshold) && ((gradient.norm()) > 0.0001) && (iters < 50)){
            x_step = y_step = z_step = pos_step_init;
            angle_step = angle_step_init;
            while (true){ //get local position gradient
              shape_top     = model_curr.translate(e::Vector3d(0,0,z_step));
              shape_bottom  = model_curr.translate(e::Vector3d(0,0,-z_step));
              shape_left    = model_curr.translate(e::Vector3d(0,y_step,0));
              shape_right   = model_curr.translate(e::Vector3d(0,-y_step,0));
              shape_front   = model_curr.translate(e::Vector3d(x_step,0,0));
              shape_back    = model_curr.translate(e::Vector3d(-x_step,0,0));
              /* shape_pd      = rotateModel(model_curr, position_curr,  e::Vector3d(-sin(RPY_curr.z()),cos(RPY_curr.z()),0), pitch_step); */
              /* shape_pu      = rotateModel(model_curr, position_curr,  e::Vector3d(-sin(RPY_curr.z()),cos(RPY_curr.z()),0), -pitch_step); */
              /* shape_rr      = rotateModel(model_curr, position_curr,  e::Vector3d(cos(RPY_curr.z()),sin(RPY_curr.z()),0), roll_step); */
              /* shape_rl      = rotateModel(model_curr, position_curr,  e::Vector3d(cos(RPY_curr.z()),sin(RPY_curr.z()),0), -roll_step); */

              top_error = totalError(shape_top, observed_points, target, image_index);
              /* ROS_INFO_STREAM("[UVDARPoseCalculator]: top: " << top_error); */
              bottom_error = totalError(shape_bottom, observed_points, target, image_index);
              /* ROS_INFO_STREAM("[UVDARPoseCalculator]: bottom: " << bottom_error); */
              if ( (z_step > 0.0001) && (sgn(bottom_error - error_total) == sgn(top_error - error_total))) {
                z_step /= 2;
                continue;
              }
              left_error = totalError(shape_left, observed_points, target, image_index);
              /* ROS_INFO_STREAM("[UVDARPoseCalculator]: left: " << left_error); */
              right_error = totalError(shape_right, observed_points, target, image_index);
              /* ROS_INFO_STREAM("[UVDARPoseCalculator]: right:" << right_error); */
              if ( (y_step > 0.0001) && (sgn(left_error - error_total) == sgn(right_error - error_total))) {
                y_step /= 2;
                continue;
              }
              front_error = totalError(shape_front, observed_points, target, image_index);
              /* ROS_INFO_STREAM("[UVDARPoseCalculator]: front: " << front_error); */
              back_error = totalError(shape_back, observed_points, target, image_index);
              /* ROS_INFO_STREAM("[UVDARPoseCalculator]: back: " << back_error); */
              if ( (x_step > 0.0001) && (sgn(front_error - error_total) == sgn(back_error - error_total))) {
                x_step /= 2;
                continue;
              }

              break;
            }
            gradient(0) = ((top_error-bottom_error)/(2*x_step))/((double)(observed_points.size()));
            gradient(1) = ((left_error-right_error)/(2*y_step))/((double)(observed_points.size()));
            gradient(2) = ((front_error-back_error)/(2*z_step))/((double)(observed_points.size()));


            /* double lin_gradient = (gradient.topRightCorner(3,1).norm()); */
            double lin_gradient;
            double dist = 0.0;
            double lin_step_init = 1.0;
            double lin_diff_step = lin_step_init;
            double lin_step = lin_step_init;
            /* double lin_step = gradient.topRightCorner(3,1).norm(); */
            e::Vector3d step_dir = -(gradient.topRightCorner(3,1).normalized());
            double error_shift_prev = error_total;
            double error_shift_curr = error_total;
            auto model_shifted_curr = model_curr;
            do {
              error_shift_prev = error_shift_curr;
              lin_diff_step = lin_step_init;
              while (true){ //approach minimum along gradient
                double dist_a = dist+lin_diff_step;
                double dist_b = dist-lin_diff_step;
                auto model_shifted_a     = model_curr.translate(step_dir*dist_a);
                auto model_shifted_b     = model_curr.translate(step_dir*dist_b);
                double error_shift_a = totalError(model_shifted_a, observed_points, target, image_index);
                double error_shift_b = totalError(model_shifted_b, observed_points, target, image_index);
                if ( (lin_diff_step > 0.0001) && (sgn(error_shift_a - error_total) == sgn(error_shift_b - error_total))) {
                  lin_diff_step /= 2;
                  continue;
                }
                lin_gradient = ((error_shift_a-error_shift_b)/(2*lin_diff_step))/((double)(observed_points.size()));
                break;
              }
              lin_step = lin_step_init;
              while (true){
                double dist_tent = dist-(sgn(lin_gradient)*lin_step);
                /* lin_step = lin_step_init; */
                model_shifted_curr = model_curr.translate(step_dir*dist_tent);
                error_shift_curr = totalError(model_shifted_curr, observed_points, target, image_index);
                if (error_shift_prev-error_shift_curr < 0.0){
                  lin_step /= 2;
                  continue;
                }
                /* ROS_INFO_STREAM("[UVDARPoseCalculator]: error_shifted: " << error_shift_curr); */
                dist = dist_tent;
                break;
              }
            } while (error_shift_prev-error_shift_curr > 0.001);
            position_curr += step_dir*dist;
            model_curr = model_shifted_curr;
            error_total = error_shift_curr;
            /* if (gradient.topRightCorner(3,1).norm() > 1){ */
            /*   gradient.topRightCorner(3,1) = gradient.topRightCorner(3,1).normalized(); */
            /* } */
            /* gradient.topRightCorner(3,1) = gradient.topRightCorner(3,1).normalized()*pos_step; */

            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: position_shifted: " << position_curr); */
            while (true){

              shape_ccw     = model_curr.rotate(position_curr,  orientation_curr*e::Vector3d(0,0,1), angle_step);
              shape_cw      = model_curr.rotate(position_curr,  orientation_curr*e::Vector3d(0,0,1), -angle_step);
              ccw_error = totalError(shape_ccw, observed_points, target, image_index);
              /* ROS_INFO_STREAM("[UVDARPoseCalculator]: ccw: " << ccw_error); */
              cw_error = totalError(shape_cw, observed_points, target, image_index);
              /* ROS_INFO_STREAM("[UVDARPoseCalculator]: cw: " << cw_error); */
              if ( (angle_step > 0.0001) && (sgn(ccw_error - error_total) == sgn(cw_error - error_total))) {
                angle_step /= 2;
                continue;
              }
              shape_pd      = model_curr.rotate(position_curr,  orientation_curr*e::Vector3d(0,1,0), angle_step);
              shape_pu      = model_curr.rotate(position_curr,  orientation_curr*e::Vector3d(0,1,0), -angle_step);
              pd_error = totalError(shape_pd, observed_points, target, image_index);
              /* ROS_INFO_STREAM("[UVDARPoseCalculator]: pd: " << pd_error); */
              pu_error = totalError(shape_pu, observed_points, target, image_index);
              /* ROS_INFO_STREAM("[UVDARPoseCalculator]: pu: " << pu_error); */
              if ( (angle_step > 0.0001) && (sgn(pd_error - error_total) == sgn(pu_error - error_total))) {
                angle_step /= 2;
                continue;
              }
              shape_rr      = model_curr.rotate(position_curr,  orientation_curr*e::Vector3d(1,0,0), angle_step);
              shape_rl      = model_curr.rotate(position_curr,  orientation_curr*e::Vector3d(1,0,0), -angle_step);
              rr_error = totalError(shape_rr, observed_points, target, image_index);
              /* ROS_INFO_STREAM("[UVDARPoseCalculator]: rr: " << rr_error); */
              rl_error = totalError(shape_rl, observed_points, target, image_index);
              /* ROS_INFO_STREAM("[UVDARPoseCalculator]: rl: " << rl_error); */
              if ( (angle_step > 0.0001) && (sgn(rr_error - error_total) == sgn(rl_error - error_total))) {
                angle_step /= 2;
                continue;
              }

              break;
            }

            gradient(3) = ((ccw_error-cw_error)/(2*angle_step))/((double)(observed_points.size()));
            gradient(4) = ((pd_error-pu_error)/(2*angle_step))/((double)(observed_points.size()));
            gradient(5) = ((rr_error-rl_error)/(2*angle_step))/((double)(observed_points.size()));
            /* gradient(4) = 0; */
            /* gradient(5) = 0; */

            if (gradient.bottomRightCorner(3,1).norm() > (angle_step*0.5)){
              gradient.bottomRightCorner(3,1) = gradient.bottomRightCorner(3,1).normalized()*angle_step*0.5;
            }


            double rot_gradient;
            double angle = 0.0;
            double rot_step_init = 1.0;
            double rot_diff_step = rot_step_init;
            double rot_step = rot_step_init;
            e::Vector3d step_axis =
              e::AngleAxisd(
              e::AngleAxisd(RPY_curr.x(), e::Vector3d::UnitX()) *
              e::AngleAxisd(RPY_curr.y(), e::Vector3d::UnitY()) *
              e::AngleAxisd(RPY_curr.z(), e::Vector3d::UnitZ())
              ).axis();

              /* -(gradient.topRightCorner(3,1).normalized()); */
            double error_rot_prev = error_total;
            double error_rot_curr = error_total;
            auto model_rotated_curr = model_curr;
            do {
              error_rot_prev = error_rot_curr;
              rot_diff_step = rot_step_init;
              while (true){ //approach minimum along gradient
                double angle_a = angle+rot_diff_step;
                double angle_b = angle-rot_diff_step;
                auto model_rotated_a     = model_curr.rotate(position_curr, step_axis, angle_a);
                auto model_rotated_b     = model_curr.rotate(position_curr, step_axis, angle_b);
                double error_rot_a = totalError(model_rotated_a, observed_points, target, image_index);
                double error_rot_b = totalError(model_rotated_b, observed_points, target, image_index);
                if ( (rot_diff_step > 0.0001) && (sgn(error_rot_a - error_total) == sgn(error_rot_b - error_total))) {
                  rot_diff_step /= 2;
                  continue;
                }
                rot_gradient = ((error_rot_a-error_rot_b)/(2*rot_diff_step))/((double)(observed_points.size()));
                break;
              }
              rot_step = rot_step_init;
              while (true){
                double angle_tent = angle-(sgn(rot_gradient)*rot_step);
                /* rot_step = rot_step_init; */
                model_rotated_curr = model_curr.translate(step_dir*angle_tent);
                error_rot_curr = totalError(model_rotated_curr, observed_points, target, image_index);
                if (error_rot_prev-error_rot_curr < 0.0){
                  rot_step /= 2;
                  continue;
                }
                /* ROS_INFO_STREAM("[UVDARPoseCalculator]: error_shifted: " << error_shift_curr); */
                angle = angle_tent;
                break;
              }
            } while (error_rot_prev-error_rot_curr > 0.001);
            orientation_curr = e::AngleAxisd(angle,step_axis)*orientation_curr;
            model_curr = model_rotated_curr;
            error_total = error_rot_curr;

            /* /1* gradient.bottomRightCorner(3,1) = gradient.bottomRightCorner(3,1).normalized(); *1/ */
            /* double yaw_diff, pitch_diff, roll_diff; */

            /* if (abs(gradient(3)) > angle_step){ */
            /*   /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: Saturating yaw gradient"); *1/ */
            /*   yaw_diff = -sgn(gradient(3)); */
            /* } */
            /* else if (abs(gradient(3)) > 1E-9) */
            /*   yaw_diff = -gradient(3); */
            /* else */
            /*   yaw_diff = 0; */

            /* if (abs(gradient(4)) > angle_step){ */
            /*   /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: Saturating pitch gradient"); *1/ */
            /*   pitch_diff = -sgn(gradient(4)); */
            /* } */
            /* else if (abs(gradient(4)) > 1E-9) */
            /*   pitch_diff = -gradient(4); */
            /* else */
            /*   pitch_diff = 0; */

            /* if (abs(gradient(5)) > angle_step){ */
            /*   /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: Saturating roll gradient"); *1/ */
            /*   roll_diff = -sgn(gradient(5)); */
            /* } */
            /* else if (abs(gradient(5)) > 1E-9) */
            /*   roll_diff = -gradient(5); */
            /* else */
            /*   roll_diff = 0; */

            /* /1* position_curr = position_curr + e::Vector3d(x_diff, y_diff, z_diff); *1/ */
            /* RPY_curr = RPY_curr + e::Vector3d(roll_diff, pitch_diff, yaw_diff); */
            /* orientation_curr = */
            /*   e::AngleAxisd(RPY_curr.x(), e::Vector3d::UnitX()) */
            /*   * e::AngleAxisd(RPY_curr.y(), e::Vector3d::UnitY()) */
            /*   * e::AngleAxisd(RPY_curr.z(), e::Vector3d::UnitZ())*orientation_start; */
            /* model_curr = model.rotate(e::Vector3d(0,0,0), orientation_curr).translate(position_curr); */
            /* error_total = totalError(model_curr, observed_points, target, image_index); */

            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: gradient: " << gradient.transpose()); */
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: position XYZ: " << position_curr.transpose()); */
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: rotation RPY: " << RPY_curr.transpose()); */
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: total error: " << error_total << " : iters: " << iters); */
            /* exit(3); */
            iters++;
          }
          //final gradient check
          double mean_error = error_total/((double)(observed_points.size()));
          x_step = y_step = z_step = pos_step_init;
          angle_step = angle_step_init;
          /* y_step = 0.10; */
          /* z_step = 0.10; */
          shape_top   = model_curr.translate(e::Vector3d(0,0,z_step));
          shape_bottom= model_curr.translate(e::Vector3d(0,0,-z_step));
          shape_left  = model_curr.translate(e::Vector3d(0,y_step,0));
          shape_right = model_curr.translate(e::Vector3d(0,-y_step,0));
          shape_front = model_curr.translate(e::Vector3d(z_step,0,0));
          shape_back  = model_curr.translate(e::Vector3d(-z_step,0,0));
          gradient.x() = ((((abs(front_error)+abs(back_error))/2)-mean_error)/(x_step))/((double)(observed_points.size()));
          gradient.y() = ((((abs(left_error)+abs(right_error))/2)-mean_error)/(y_step))/((double)(observed_points.size()));
          gradient.z() = ((((abs(top_error)+abs(bottom_error))/2)-mean_error)/(z_step))/((double)(observed_points.size()));


          ROS_INFO_STREAM("x: [" << position_curr.transpose() << "] rot: [" << rad2deg(rotmatToYaw(orientation_curr.toRotationMatrix())) << "] with error of " << error_total);
          if (best_error > error_total){
            best_error = error_total;
            best_result = {position_curr, orientation_curr};
            ROS_INFO_STREAM("selected");

          }
        }

        /* return {position_curr, orientation_curr}; */
        return best_result;
        /* return {e::Vector3d::UnitX(), e::Quaterniond(1,0,0,0)}; */
      }

      double totalError(LEDModel model, std::vector<cv::Point3d> observed_points, int target, int image_index, bool discrete_pixels=false){
        struct ProjectedMarker {
          cv::Point2d position;
          int freq_id;
          double view_angle;
          double distance;
        };
        double total_error = 0;

        std::vector<ProjectedMarker> projected_markers;
        for (auto marker : model){
          auto curr_projected =  camPointFromModelPoint(marker, image_index);
          projected_markers.push_back({
              .position = curr_projected.first,
              .freq_id = marker.freq_id,
              .view_angle = curr_projected.second,
              .distance = marker.position.norm()
              });

          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: projected marker:  " << projected_markers.back().position << " : " << _frequencies_[(target*_frequencies_per_target_)+projected_markers.back().freq_id]); */
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: fid:  " << (target*_frequencies_per_target_)+projected_markers.back().freq_id  << "; target: " << target << "; lfid: " << projected_markers.back().freq_id); */
        }

        std::vector<ProjectedMarker> selected_markers;
        for (auto marker : projected_markers){
          double distance = marker.distance;
          double cos_angle =  cos(marker.view_angle);
          double led_intensity =
            round(std::max(.0, cos_angle) * (led_projection_coefs_[0] + (led_projection_coefs_[1] / ((distance + led_projection_coefs_[2]) * (distance + led_projection_coefs_[2])))));
          if (led_intensity > 0) { // otherwise they will probably not be visible
            selected_markers.push_back(marker);
          }
        }

        for (int i = 0; i < ((int)(selected_markers.size()) - 1); i++){
          for (int j = i+1; j < (int)(selected_markers.size()); j++){
            double tent_dist = cv::norm(selected_markers[i].position-selected_markers[j].position);
            if (tent_dist < 3){
              if (selected_markers[i].freq_id == selected_markers[j].freq_id){ // if the frequencies are the same, they tend to merge. Otherwise, the result varies
                selected_markers[i].position = (selected_markers[i].position + selected_markers[j].position)/2; //average them
                selected_markers.erase(selected_markers.begin()+j); //remove the other
                j--; //we are not expecting many 2+ clusters, so we will not define special case for this
              }
            }
          }
        }

        for (auto& obs_point : observed_points){

          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: observed marker:  " << obs_point); */
          ProjectedMarker closest_projection;
          double closest_distance = std::numeric_limits<double>::max();
          bool any_match_found = false;
          for (auto& proj_point : projected_markers){
            double tent_signal_distance = abs( (1.0/(_frequencies_[(target*_frequencies_per_target_)+proj_point.freq_id])) - (1.0/(obs_point.z)) );
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: signal distance:  " << tent_signal_distance); */
            if (tent_signal_distance < (2.0/estimated_framerate_[image_index])){
              double tent_image_distance;
              if (!discrete_pixels){
                tent_image_distance = cv::norm(proj_point.position - cv::Point2d(obs_point.x, obs_point.y));
              }
              else {
                tent_image_distance = cv::norm(cv::Point2i(proj_point.position.x - obs_point.x, proj_point.position.y - obs_point.y));
              }
              if (tent_image_distance > 100){
                /* ROS_INFO_STREAM("[UVDARPoseCalculator]: obs_point: " << cv::Point2d(obs_point.x, obs_point.y) << " : proj_point: " << proj_point.position); */
                /* exit(3); */
              }
              if (tent_image_distance < closest_distance){
                closest_distance = tent_image_distance;
                any_match_found = true;
              }
            }
          }
          if (any_match_found){
            total_error += sqr(closest_distance);
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: closest_distance squared: " << sqr(closest_distance)); */
          }
          else {
            total_error += UNMATCHED_OBSERVED_POINT_PENALTY;
          }
        }


        return total_error;
      }

      e::MatrixXd getCovarianceEstimate(LEDModel model, std::vector<cv::Point3d> observed_points, std::pair<e::Vector3d, e::Quaterniond> pose, int target, int image_index){
        e::MatrixXd output;

        double trans_scale = 0.1;
        /* rot_scale = 0.05; */
        double rot_scale = 0.1;
        auto Y0 = pose;
        e::MatrixXd Y(6,(3*3*3*3*3*3));
        std::vector<int> j = {-1,0,1};
        int k = 0;
        std::vector<double> Xe;
        for (auto x_s : j){
          for (auto y_s : j){
            for (auto z_s : j){
              for (auto r_s : j){
                for (auto p_s : j){
                  for (auto y_s : j){
                    if (
                        (x_s == 0) &&
                        (y_s == 0) &&
                        (z_s == 0) &&
                        (r_s == 0) &&
                        (p_s == 0) &&
                        (y_s == 0)
                        ){
                      Xe.push_back(std::numeric_limits<double>::max()); //to avoid singularities
                      Y(0,k) = Y0.first.x();
                      Y(1,k) = Y0.first.y();
                      Y(2,k) = Y0.first.z();
                      Y(3,k) = rotmatToRoll(Y0.second.toRotationMatrix());
                      Y(4,k) = rotmatToPitch(Y0.second.toRotationMatrix());
                      Y(5,k) = rotmatToYaw(Y0.second.toRotationMatrix());
                    }
                    else {
                      e::Quaterniond rotation(
                          e::AngleAxisd(y_s, Y0.second*e::Vector3d(0,0,1)) *
                          e::AngleAxisd(p_s, Y0.second*e::Vector3d(0,1,0)) *
                          e::AngleAxisd(r_s, Y0.second*e::Vector3d(1,0,0))
                          );
                      auto model_curr = model.rotate(Y0.first,  rotation);
                      model_curr = model_curr.translate(e::Vector3d(x_s, y_s, z_s));

                      Xe.push_back(totalError(model_curr, observed_points, target, image_index, false));
                      Y(0,k) = Y0.first.x()+x_s*trans_scale;
                      Y(1,k) = Y0.first.y()+y_s*trans_scale;
                      Y(2,k) = Y0.first.z()+z_s*trans_scale;
                      Y(3,k) = rotmatToRoll(Y0.second.toRotationMatrix()) + r_s*rot_scale;
                      Y(4,k) = rotmatToPitch(Y0.second.toRotationMatrix()) + p_s*rot_scale;
                      Y(5,k) = rotmatToYaw(Y0.second.toRotationMatrix()) + y_s*rot_scale;
                    }
                    k++;
                  }
                }
              }
            }
          }
        }
        e::VectorXd W(Xe.size());
        int i = 0;
        double Wsum = 0;
        for (auto xe : Xe){
          W(i) = (1.0/xe);
          Wsum += W(i);
          i++;
        }
        /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Wsum: [\n" << Wsum << "\n]"); */
        /* ROS_INFO_STREAM("[UVDARPoseCalculator]: W_orig: [\n" << W << "\n]"); */
        /* W /= (Wsum); */
        W *= sqr(QPIX)*observed_points.size();
        /* ROS_INFO_STREAM("[UVDARPoseCalculator]: W: [\n" << W << "\n]"); */
        auto y = Y*W;
        /* ROS_INFO_STREAM("[UVDARPoseCalculator]: y: [\n" << y << "\n]"); */
        auto Ye = Y-y.replicate(1,W.size());
        /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Ye: [\n" << Ye << "\n]"); */

        auto P = Ye*W.asDiagonal()*Ye.transpose();
        e::JacobiSVD<e::MatrixXd> svd(P, e::ComputeThinU | e::ComputeThinV);
        /* if (P.topLeftCorner(3,3).determinant() > 0.001){ */

          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: P: [\n" << P << "\n]"); */
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: P determinant: [\n" << P.topLeftCorner(3,3).determinant() << "\n]"); */
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Singular values: [\n" << svd.singularValues() << "\n]"); */
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Matrix V: [\n" << svd.matrixV() << "\n]"); */
        /* } */
          

        /* output.setIdentity(6,6); */
        output = P;
        return output;
      }

      e::Vector3d directionFromCamPoint(cv::Point3d point, int image_index){
          double v_i[2] = {(double)(point.y), (double)(point.x)};
          double v_w_raw[3];
          cam2world(v_w_raw, v_i, &(_oc_models_[image_index]));
          return e::Vector3d(v_w_raw[1], v_w_raw[0], -v_w_raw[2]);
      }

      std::pair<cv::Point2d, double> camPointFromModelPoint(LEDMarker marker, int image_index){
        auto marker_cam_pos = opticalFromMarker(marker);
        auto output_position = camPointFromObjectPoint(marker_cam_pos.first, image_index);
        e::Vector3d view_vector = -(marker_cam_pos.first.normalized());
        e::Vector3d led_vector = marker_cam_pos.second*e::Vector3d(0,0,1);
        double view_angle = acos(view_vector.dot(led_vector));
        return {output_position, view_angle};
      }

      std::pair<e::Vector3d, e::Quaterniond> opticalFromMarker(LEDMarker marker){
        e::Quaterniond rotator(0.5,0.5,-0.5,0.5);
        /* ROS_INFO_STREAM("[UVDARPoseCalculator]: marker_pos: " << marker.position << "; rotated: " << rotator*marker.position); */
        return {rotator*marker.position, rotator*marker.orientation};
      }

      std::pair<std::pair<e::Vector3d,e::Quaterniond>,e::MatrixXd> opticalFromBase(std::pair<e::Vector3d,e::Quaterniond> pose, e::MatrixXd covariance){
        auto output_pose = pose;
        auto output_covariance = covariance;
        e::Quaterniond rotator(0.5,0.5,-0.5,0.5);
        output_pose.first = rotator*pose.first;
        output_pose.second = rotator*pose.second;
        output_covariance.topLeftCorner(3,3) = rotator.toRotationMatrix()*output_covariance.topLeftCorner(3,3)*rotator.toRotationMatrix().transpose();
        return {output_pose, output_covariance};
      }

      e::Vector3d baseFromOptical(e::Vector3d input){
        return e::Vector3d(input.z(), -input.x(), -input.y());
      }

      cv::Point2d camPointFromObjectPoint(e::Vector3d point, int image_index){
          double v_w[3] = {point.y(), point.x(),-point.z()};
          double v_i_raw[2];
          world2cam(v_i_raw, v_w, &(_oc_models_[image_index]));
          return cv::Point2d(v_i_raw[1], v_i_raw[0]);
      }

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
      
      template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
      }

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


      /* Eigen::MatrixXd Px2,Px3,Px2q; */
      /* Eigen::MatrixXd Px2qb,Px3qb,Px4qb; */
      /* Eigen::VectorXd X2,X3,X2q; */
      /* Eigen::VectorXd X2qb,X3qb,X4qb; */

      std::vector<ros::Publisher> pub_measured_poses_;

      std::vector<double> _frequencies_;
      int _frequencies_per_target_;
      int _target_count_;
      std::unique_ptr<UVDARFrequencyClassifier> ufc_;


      std::vector<cv::Rect> bracket_set;

      std::vector<std::string> _calib_files_;

      std::string _model_file_;

      bool _use_masks_;
      std::vector<std::string> _mask_file_names_;
      std::vector<cv::Mat> _masks_;

      double _arm_length_;
      double _beacon_height_;

      bool _quadrotor_;
      bool _beacon_;
      bool _custom_model_;


      LEDModel model_;

      bool initialized_ = false;

      std::vector<std::shared_ptr<std::mutex>>  mutex_separated_points_;
      std::vector<std::vector<std::pair<int,std::vector<cv::Point3d>>>> separated_points_;

      double led_projection_coefs_[3] = {1.3398, 31.4704, 0.0154}; //empirically measured coefficients of the decay of blob radius wrt. distance of a LED in our UVDAR cameras.
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

