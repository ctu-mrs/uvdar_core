#include <ros/ros.h>
#include <ros/package.h>

#include <thread>
#include <mutex>
#include <numeric>
#include <fstream>
#include <boost/filesystem/operations.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
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
/* #include <frequency_classifier/frequency_classifier.h> */

#define MAX_DIST_INIT 100.0
#define BRACKET_STEP 10
#define QPIX 2 //pixel std. dev
#define TUBE_LENGTH (_beacon_?10:1000)


#define sqr(X) ((X) * (X))
#define cot(X) (cos(X)/sin(X))
#define deg2rad(X) ((X)*0.01745329251)
#define rad2deg(X) ((X)*57.2957795131)

#define DIRECTIONAL_LED_VIEW_ANGLE (deg2rad(120))
#define RING_LED_VERT_ANGLE (deg2rad(120))

#define UNMATCHED_OBSERVED_POINT_PENALTY sqr(15)
/* #define UNMATCHED_PROJECTED_POINT_PENALTY sqr(5) */
#define UNMATCHED_PROJECTED_POINT_PENALTY sqr(0)

#define LED_GROUP_DISTANCE 0.03

#define ERROR_THRESHOLD_INITIAL sqr(10)
#define ERROR_THRESHOLD_FITTED sqr(QPIX)

#define SIMILAR_ERRORS_THRESHOLD sqr(1)

#define UVDAR_RANGE 15.0
#define MAX_HYPOTHESIS_SPREAD 8.0

#define REJECT_UPSIDE_DOWN true

#define EDGE_DETECTION_MARGIN 10

#define SINGLE_HYPOTHESIS_COVARIANCE sqr(0.1)

namespace e = Eigen;

namespace Eigen
{
  typedef Matrix< double, 6, 6 > 	Matrix6d;
  typedef Matrix< double, 1, 6 > 	Vector6d;
  /* typedef Matrix< double, 4, 4 > 	Matrix4d; */
}

namespace uvdar {

  class Profiler{

    public:
      Profiler(){
        latest_times.push_back(getTime());
      }

      void addValue(std::string preamble){
        if (active){
          auto now_time = getTime();
          elapsed_time.push_back({currDepthIndent() + preamble,std::chrono::duration_cast<std::chrono::microseconds>(now_time - latest_times.back()).count()});
          latest_times.back() = now_time;
        }
      }

      void addValueBetween(std::string preamble, std::chrono::time_point<std::chrono::high_resolution_clock> start, std::chrono::time_point<std::chrono::high_resolution_clock> stop){
        if (active){
          elapsed_time.push_back({currDepthIndent() + preamble,std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()});
        }
      }

      void addValueSince(std::string preamble, std::chrono::time_point<std::chrono::high_resolution_clock> start){

        if (active){
          auto now_time = getTime();
          elapsed_time.push_back({currDepthIndent() + preamble,std::chrono::duration_cast<std::chrono::microseconds>(now_time - start).count()});
          latest_times.back() = now_time;
        }
      }

      static std::chrono::time_point<std::chrono::high_resolution_clock> getTime(){
        return std::chrono::high_resolution_clock::now();
      }

      void indent(){
        if (active){
          latest_times.push_back(getTime());
          curr_depth_indent++;
        }
      }

      void unindent(){
        if (active){
          latest_times.pop_back();
          curr_depth_indent--;
        }
      }

      void printAll(std::string preamble){
        for (auto data : elapsed_time){
          std::cout << preamble << " "  << data.first << " took : " << double(data.second)/1000.0 << " ms" << std::endl;
        }
      }

      void clear(){
        elapsed_time.clear();
        latest_times.clear();
      }

      void stop(){
        active = false;
      }

      void start(){
        active = true;
      }

    private:
      std::string currDepthIndent(){
        if (curr_depth_indent == 0)
          return "";
        else if (curr_depth_indent < 0)
          return "? ";
        else
          return std::string(2*curr_depth_indent, ' ');
      }

      bool active = false;
      int curr_depth_indent = 0;

      std::vector<std::pair<std::string,int>>  elapsed_time;

      std::vector<std::chrono::time_point<std::chrono::high_resolution_clock>> latest_times;

  };

  /**
   * @brief A processing class for converting retrieved blinking markers from a UV camera image into relative poses of observed UAV that carry these markers, as well as the error covariances of these estimates
   */
  class UVDARPoseCalculator {
    struct LEDMarker {
      e::Vector3d position;
      e::Quaterniond orientation;
      int type; // 0 - directional, 1 - omni ring, 2 - full omni
      /* int freq_id; */
      int signal_id;
    };

    class LEDModel {
      private:
      std::vector<LEDMarker> markers;
      std::vector<std::vector<int>> groups;

      public:
      LEDModel() = default;

      LEDModel(std::string model_file){
        if (!parseModelFile(model_file)){
          ros::shutdown();
          return;
        }

        prepareGroups();
      }

      LEDModel translate(e::Vector3d position) const {
        LEDModel output = *this;
        for (auto &marker : output.markers){
          marker.position += position;
          /* output.push_back(marker); */
        }
        return output;
      }

      std::vector<LEDMarker> getMarkers(){
        std::vector<LEDMarker> output = markers;
        return output;
      }


      LEDModel rotate(e::Vector3d center, e::Vector3d axis, double angle) const {
        e::Quaterniond rotation(e::AngleAxisd(angle, axis));
        return rotate(center, rotation);
      }

      LEDModel rotate(e::Vector3d center, e::Quaterniond orientation) const {
        LEDModel output = *this;
        for (auto &marker : output.markers){
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: pos a: " << marker.position); */
          marker.position -= center;
          marker.position = orientation*marker.position;
          marker.position += center;
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: pos b: " << marker.position); */
          marker.orientation = orientation * marker.orientation;
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

      int maxSignalID(){
        int max_signal_id = -1;
        for (auto& m : markers){
          if (m.signal_id > max_signal_id ){
            max_signal_id = m.signal_id;
          }
        }
        return max_signal_id;
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

      inline std::vector<LEDMarker>::const_iterator begin() const noexcept { return markers.begin(); }
      inline std::vector<LEDMarker>::const_iterator end() const noexcept { return markers.end(); }

      private:

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
            int signal_id;
            iss >> signal_id;
            curr_lm.signal_id = signal_id;

            markers.push_back(curr_lm);
            ROS_INFO_STREAM("[UVDARPoseCalculator]: Loaded Model: [ X: " << X <<" Y: "  << Y << " Z: "  << Z << " type: " << type << " pitch: " << pitch << " yaw: " << yaw << " signal_id: " << signal_id << " ]");
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
        param_loader.loadParam("profiling", _profiling_, bool(false));

        if (_profiling_){
          profiler.start();
        }
        else {
          profiler.stop();
        }

        param_loader.loadParam("gui", _gui_, bool(false));
        param_loader.loadParam("publish_visualization", _publish_visualization_, bool(false));

        param_loader.loadParam("publish_constituents", _publish_constituents_, bool(false));

        param_loader.loadParam("quadrotor",_quadrotor_,bool(false));

        param_loader.loadParam("beacon",_beacon_,bool(false));

        param_loader.loadParam("custom_model",_custom_model_,bool(false));
        param_loader.loadParam("model_file",_model_file_, std::string(""));
        
        param_loader.loadParam("separate_by_distance",_separate_by_distance_,bool(true));

        prepareModel();


        /* load the signals //{ */
        param_loader.loadParam("signal_ids", _signal_ids_);
        if (_signal_ids_.empty()){
          ROS_WARN("[UVDARPoseCalculator]: No signal IDs were supplied, using the default sequence set.");
          _signal_ids_ = {0, 1, 2, 3, 4, 5, 6, 7, 8};
        }

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

        separated_points_.resize(_blinkers_seen_topics.size());
        for (size_t i = 0; i < _blinkers_seen_topics.size(); ++i) {
          blinkers_seen_callback_t callback = [image_index=i,this] (const mrs_msgs::ImagePointsWithFloatStampedConstPtr& pointsMessage) { 
            ProcessPoints(pointsMessage, image_index);
          };
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Subscribing to " << _blinkers_seen_topics[i]);
          sub_blinkers_seen_.push_back(
              nh.subscribe(_blinkers_seen_topics[i], 1, callback));

          ROS_INFO_STREAM("[UVDARPoseCalculator]: Advertising measured poses " << i+1);
          pub_measured_poses_.push_back(nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("measuredPoses"+std::to_string(i+1), 1)); 

          if (_publish_constituents_){
            pub_constituent_poses_.push_back(nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("constituentPoses"+std::to_string(i+1), 1)); 
            pub_constituent_hypo_poses_.push_back(nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("constituentHypoPoses"+std::to_string(i+1), 1)); 
          }

          camera_image_sizes_.push_back(cv::Size(-1,-1));

          mutex_separated_points_.push_back(std::make_shared<std::mutex>());
        }
        //}

        /* Load calibration files //{ */
        param_loader.loadParam("calib_files", _calib_files_, _calib_files_);
        if (_calib_files_.empty()) {
          ROS_ERROR("[UVDARPoseCalculator]: No camera calibration files were supplied. You can even use \"default\" for the cameras, but no calibration is not permissible. Returning.");
          ros::shutdown();
          return;
        }
        if (!loadCalibrations()){
          ROS_ERROR("[UVDARPoseCalculator The camera calibration files could not be loaded!");
          ros::shutdown();
          return;
        }

        /* prepare masks if necessary //{ */
        param_loader.loadParam("use_masks", _use_masks_, bool(false));
        if (_use_masks_){
          param_loader.loadParam("mask_file_names", _mask_file_names_, _mask_file_names_);

          if (_mask_file_names_.size() != _camera_count_){
            ROS_ERROR_STREAM("[UVDARPoseCalculator Masks are enabled, but the number of mask filenames provided does not match the number of camera topics (" << _camera_count_ << ")!");
            ros::shutdown();
            return;
          }

          if (!loadMasks()){
            ROS_ERROR("[UVDARPoseCalculator Masks are enabled, but the mask files could not be loaded!");
            ros::shutdown();
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
          ros::shutdown();
          return;
        }
        for (size_t i = 0; i < _estimated_framerate_topics.size(); ++i) {
          estimated_framerate_callback_t callback = [image_index=i,this] (const std_msgs::Float32ConstPtr& framerateMessage) { 
            estimated_framerate_[image_index] = framerateMessage->data;
          };

          ROS_INFO_STREAM("[UVDARPoseCalculator]: Subscribing to " << _estimated_framerate_topics[i]);
          sub_estimated_framerate_.push_back(
              nh.subscribe(_estimated_framerate_topics[i], 1, callback));
        }
        //}

        /* Set transformation frames for cameras //{ */
        param_loader.loadParam("camera_frames", _camera_frames_, _camera_frames_);
        if (_camera_frames_.size() != _blinkers_seen_topics.size()) {
          ROS_ERROR_STREAM("The size of camera_frames (" << _camera_frames_.size() << 
              ") is different from blinkers_seen_topics size (" << _blinkers_seen_topics.size() << ")");
          ros::shutdown();
          return;
        }
        //}

        if (_gui_ || _publish_visualization_){
          if (_publish_visualization_){
            pub_visualization_ = std::make_unique<mrs_lib::ImagePublisher>(boost::make_shared<ros::NodeHandle>(nh));
          }
          timer_visualization_ = nh.createTimer(ros::Rate(1), &UVDARPoseCalculator::VisualizationThread, this, false);
        }

        transformer_ = mrs_lib::Transformer("UVDARPoseCalculator");
        transformer_.setDefaultPrefix(_uav_name_);

        tf_fcu_to_cam.resize(_blinkers_seen_topics.size(), std::nullopt);
        camera_view_.resize(_blinkers_seen_topics.size(), e::Quaterniond());

        initModelsAndAxisVectors();

        initialized_ = true;
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

          ROS_INFO_STREAM("[UVDARPoseCalculator]: Camera resolution  is: " << _oc_models_.at(i).width << "x" << _oc_models_.at(i).height);

          /* auto center_dir = directionFromCamPoint(cv::Point3d(_oc_models_[i].yc,_oc_models_[i].xc,0),i); */
          auto center_dir = directionFromCamPoint(cv::Point3d(_oc_models_.at(i).width/2,_oc_models_.at(i).height/2,0),i);
          auto zero_dir = e::Vector3d(0,0,1);

          double x_ang = acos(e::Vector3d(center_dir.x(),0,center_dir.z()).normalized().dot(zero_dir));
          if (center_dir.x()<0){
            x_ang = -x_ang;
          }

          double y_ang = acos(e::Vector3d(0,center_dir.y(),center_dir.z()).normalized().dot(zero_dir));
          if (center_dir.y()<0){
            y_ang = -y_ang;
          }

          ROS_INFO_STREAM("[UVDARPoseCalculator]: Central direction is: " << center_dir.transpose());
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Offset in angles from central direction: X=" << rad2deg(x_ang) << ": Y=" << rad2deg(y_ang) );

          _center_fix_.push_back(e::Quaterniond(e::AngleAxisd(-x_ang*0.5, e::Vector3d(0,-1,0))*e::AngleAxisd(-y_ang*0.5, e::Vector3d(1,0,0))));

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

        /* param_loader.loadParam("signals_per_target", signals_per_target_, int(1)); */
        signals_per_target_ = model_.maxSignalID()+1;
        _target_count_ = (int)(_signal_ids_.size())/signals_per_target_;

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

        if (!tf_fcu_to_cam[image_index]){
          ROS_INFO_THROTTLE(1.0,"[UVDARPoseCalculator]: Camera TF not yet obatined. Attempting to retrieve it...");
          {
            std::scoped_lock lock(transformer_mutex);
            tf_fcu_to_cam[image_index] = transformer_.getTransform(_uav_name_+"/fcu", _camera_frames_[image_index], msg->stamp);
          }
          if (!tf_fcu_to_cam[image_index]) {
            ROS_ERROR_STREAM_THROTTLE(1.0,"[UVDARPoseCalculator]: Could not obtain transform from " << _uav_name_+"/fcu" << " to " << _camera_frames_[image_index] << "!");
            return;
          }
          else {
            const bool prepared_rotation_axes = prepareModelsAndAxisVectors(image_index);
            if (!prepared_rotation_axes)
              tf_fcu_to_cam[image_index] = std::nullopt;
          }
        }

        camera_image_sizes_[image_index].width = msg->image_width;
        camera_image_sizes_[image_index].height = msg->image_height;

        for (const auto& point : msg->points) {
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

        mrs_msgs::PoseWithCovarianceArrayStamped msg_constuents_array;
        mrs_msgs::PoseWithCovarianceArrayStamped msg_constuents_hypo_array;
        msg_constuents_array.header.frame_id = _camera_frames_[image_index];
        msg_constuents_array.header.stamp = last_blink_time_;
        msg_constuents_hypo_array.header.frame_id = _camera_frames_[image_index];
        msg_constuents_hypo_array.header.stamp = last_blink_time_;

        if ((int)(points.size()) > 0) {
          if (_beacon_){
            separated_points_[image_index] = separateByBeacon(points);
          }
          else {
            /* separated_points_[image_index] = separateByFrequency(points); */
            separated_points_[image_index] = separateBySignals(points);
          }

          auto start_target_cycle = profiler.getTime();
          profiler.indent();
          for (int i = 0; i < ((int)(separated_points_[image_index].size())); i++) {

            auto start_target_iteration = profiler.getTime();
            if (_debug_){
              ROS_INFO_STREAM("[UVDARPoseCalculator]: target [" << separated_points_[image_index][i].first << "]: ");
              ROS_INFO_STREAM("[UVDARPoseCalculator]: p: " << std::endl << separated_points_[image_index][i].second);
            }
            mrs_msgs::PoseWithCovarianceIdentified pose;
            std::vector<mrs_msgs::PoseWithCovarianceIdentified> constituents;
            std::vector<mrs_msgs::PoseWithCovarianceIdentified> constituents_hypo;
            bool res = extractSingleRelative(separated_points_[image_index][i].second, separated_points_[image_index][i].first, image_index, pose, constituents, constituents_hypo);

            if (res){
              msg_measurement_array.poses.push_back(pose);

              if (_publish_constituents_){
                for (auto &constituent : constituents){
                  msg_constuents_array.poses.push_back(constituent);
                }
                for (auto &constituent : constituents_hypo){
                  msg_constuents_hypo_array.poses.push_back(constituent);
                }
              }


              profiler.addValueSince("Target "+std::to_string(separated_points_[image_index][i].first),start_target_iteration);
              if (_profiling_){
                profiler.printAll("[UVDARPoseCalculator]: [cam:"+std::to_string(image_index)+"]-[tg:"+std::to_string(separated_points_[image_index][i].first)+"]:");
              }
              profiler.clear();
            }

            profiler.unindent();
            profiler.addValueSince("All targets",start_target_cycle);

            if (_profiling_){
              profiler.printAll("[UVDARPoseCalculator]: [cam:"+std::to_string(image_index)+"]:");
            }
            profiler.clear();
          }
          pub_measured_poses_[image_index].publish(msg_measurement_array);

          if (_publish_constituents_){
            pub_constituent_poses_[image_index].publish(msg_constuents_array);
            pub_constituent_hypo_poses_[image_index].publish(msg_constuents_hypo_array);
          }
        }
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
        pos_cov(0, 0) = pos_cov(1, 1) = sqr(xy_covariance_coeff);
        pos_cov(2, 2) = sqr(z_covariance_coeff);

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
      /* separateBySignals //{ */
      std::vector<std::pair<int,std::vector<cv::Point3d>>> separateBySignals(std::vector< cv::Point3d > points){
        std::vector<std::pair<int,std::vector<cv::Point3d>>> separated_points;
        /* separated_points.resize(_target_count_); */


        for (int i = 0; i < (int)(points.size()); i++) {
          /* if (points[i].z > 1) { */
          if (points[i].z >= 0) {
            /* int mid = ufc_->findMatch(points[i].z); */
            int mid = points[i].z;
            int tid = classifyMatch(mid);
            if (_debug_)
              ROS_INFO("[%s]: SIG: %d, MID: %d, TID: %d", ros::this_node::getName().c_str(),(int)points[i].z, mid, tid);
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

        if (_separate_by_distance_){
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
                it = separated_points.begin()+s;
                cl_n++;
              }
              s+=cl_n++;
            }
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
            /* int mid = ufc_->findMatch(points[i].z); */
            int mid = points[i].z;
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
      bool extractSingleRelative(std::vector< cv::Point3d > points, int target, size_t image_index, mrs_msgs::PoseWithCovarianceIdentified& output_pose, std::vector<mrs_msgs::PoseWithCovarianceIdentified> &constituents, std::vector<mrs_msgs::PoseWithCovarianceIdentified> &constituents_hypo) {

        std::pair<e::Vector3d, e::Quaterniond> final_mean =
        {
          e::Vector3d( std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
          e::Quaterniond(1,0,0,0)
        };
        e::MatrixXd final_covariance;

        if (_debug_){
          ROS_INFO_STREAM("[UVDARPoseCalculator]: framerateEstim: " << estimated_framerate_[image_index]);
        }

        auto start = profiler.getTime();

        double alpha_max;
        std::vector<e::Vector3d> v_w;
        if (points.size() != 1){
          for (auto& point: points){
            v_w.push_back(directionFromCamPoint(point, image_index));
          }
          alpha_max = getLargestAngle(v_w);
        }
          
        if ((points.size() == 1) || (alpha_max < 0.01) || (avgIsNearEdge(points,EDGE_DETECTION_MARGIN, image_index))){
          auto v_w_s = baseFromOptical(directionFromCamPoint(points.at(0), image_index));
          v_w_s *= 15.0; //max range
          final_mean.first << v_w_s.x(),v_w_s.y(),v_w_s.z();
          final_mean.second = e::Quaterniond(1,0,0,0);
          final_covariance.setIdentity(6,6);
          final_covariance *= M_PI;//large covariance for angles in radians
          final_covariance.topLeftCorner(3, 3) = getLongCovariance(v_w_s,(model_.getMaxMinVisibleDiameter().first*1.0),1000.0);
        }
        else {
          auto furthest_position = getRoughInit(model_, v_w, image_index);
          if (_debug_){
            ROS_INFO_STREAM("[UVDARPoseCalculator]: Furthest possible distance: " << furthest_position.norm());
          }
          /* auto rough_init = std::chrono::high_resolution_clock::now(); */
          /* elapsedTime.push_back({currDepthIndent() + ,std::chrono::duration_cast<std::chrono::microseconds>(rough_init - start).count()}); */
          profiler.addValueSince("Rough initialization", start);

          auto [hypotheses, errors] = getViableInitialHyptheses(model_, points, furthest_position, target, image_index);

          int initial_hypothesis_count = (int)(hypotheses.size());

          /* auto fitted_position = iterFitPosition(model_, points, rough_initialization, target,  image_index); */
          /* if (_debug_){ */
          /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: Fitted position: " << fitted_position.transpose()); */
          /* } */

          if (_debug_)
            ROS_INFO_STREAM("[UVDARPoseCalculator]: Rough hypotheses for target " << target << " in image " << image_index << ": ");
          int i = 0;
          if (_debug_)
            for (auto h: hypotheses){
              ROS_INFO_STREAM("x: [" << h.first.transpose() << "] rot: [" << rad2deg(camera_view_[image_index].inverse()*quaternionToRPY(h.second)).transpose() << "] with error of " << errors.at(i++));
            }


          /* auto viable_hypotheses = std::chrono::high_resolution_clock::now(); */
          /* elapsedTime.push_back({currDepthIndent() + "Viable initial hypotheses",std::chrono::duration_cast<std::chrono::microseconds>(viable_hypotheses - rough_init).count()}); */
          profiler.addValue("Viable initial hypotheses");

          std::vector<std::pair<e::Vector3d, e::Quaterniond>> selected_poses;
          std::vector<double> projection_errors;
          /* projection_errors.push_back(std::numeric_limits<double>::max()); */
          double smallest_error_in_set = std::numeric_limits<double>::max();
          for (auto h: hypotheses){
            /* profiler.stop(); */
            profiler.indent();

            
            auto [fitted_pose, error] = iterFitFull(model_, points, h, target,  image_index);

            /* double error = 1.0; */
            /* auto fitted_pose = h; */


            profiler.unindent();

            bool found_equivalent = false;
            for (auto &prev_fitted : selected_poses){ 
              if ((fitted_pose.first - prev_fitted.first).norm() < 0.1){

                auto q_diff = fitted_pose.second*prev_fitted.second.inverse();
                double ang_diff = e::AngleAxisd(q_diff).angle();
                if (ang_diff < 0.2){
                  if (_debug_){
                    ROS_INFO_STREAM("[UVDARPoseCalculator]: Discarding duplicate hypothesis");
                  }
                  found_equivalent = true;// we don't need duplicate initial hypoteheses
                  break;

                }
              }
            }
            if (found_equivalent){
              continue;
            }



            if (error < 0){ //discarded fitting;
              continue;
            }
            /* profiler.start(); */
            if (_debug_){
              ROS_INFO_STREAM("[UVDARPoseCalculator]: Fitted pose: [" << fitted_pose.first.transpose() << "] rot: [" << rad2deg(quaternionToRPY(camera_view_[image_index].inverse()*fitted_pose.second)).transpose() << "] with error of " << error);
            }

          if (_publish_constituents_){
            e::MatrixXd hypo_covar = e::MatrixXd::Identity(6,6)*0.01;
                auto hypothesis_optical = opticalFromBase(fitted_pose,hypo_covar);
                mrs_msgs::PoseWithCovarianceIdentified constituent;

                constituent.id = target;
                constituent.pose.position.x = hypothesis_optical.first.first.x();
                constituent.pose.position.y = hypothesis_optical.first.first.y();
                constituent.pose.position.z = hypothesis_optical.first.first.z();
                constituent.pose.orientation.x = hypothesis_optical.first.second.x();
                constituent.pose.orientation.y = hypothesis_optical.first.second.y();
                constituent.pose.orientation.z = hypothesis_optical.first.second.z();
                constituent.pose.orientation.w = hypothesis_optical.first.second.w();
                for (int i=0; i<hypothesis_optical.second.cols(); i++){
                  for (int j=0; j<hypothesis_optical.second.rows(); j++){
                    constituent.covariance[hypothesis_optical.second.cols()*j+i] =  hypothesis_optical.second(j,i);
                  }
                }
                constituents_hypo.push_back(constituent);
          }

            if (fitted_pose.first.norm() < 0.5) { //We don't expect to be able to measure relative poses of UAVs this close - the markers would be too bright and too far apart
              continue;
            }

            e::Vector3d unit_vec;
            unit_vec << 1.0,0,0;
            if (acos(fitted_pose.first.normalized().dot(unit_vec)) > rad2deg(95)) //our lenses only allow us to see UAVs ~92.5 degrees away from the optical axis of the camera
              continue;


            /* if (projection_errors.back() > (error+(SIMILAR_ERRORS_THRESHOLD*(int)(points.size())))){ */
            projection_errors.push_back(error);
            selected_poses.push_back(fitted_pose);

            if ((error) < (smallest_error_in_set)){
              smallest_error_in_set = error;
            }
            /* else if (projection_errors.back() > (error-(SIMILAR_ERRORS_THRESHOLD*(int)(points.size())))){ */
          }
          int fitted_hypothesis_count = (int)(selected_poses.size());


          auto projection_errors_backup = projection_errors;

          double threshold = ERROR_THRESHOLD_FITTED*(int)(points.size());

          for (int i = 0; i<(int)(selected_poses.size()); i++){

            /* ROS_ERROR_STREAM("[UVDARPoseCalculator]: error: " << projection_errors[i]); */

            if (projection_errors[i] > threshold){ // if the frequencies are the same, they tend to merge. Otherwise, the result varies
              selected_poses.erase(selected_poses.begin()+i); //remove the other
              projection_errors.erase(projection_errors.begin()+i);
              i--; 
            }
          }

          /* auto precise_fitting = std::chrono::high_resolution_clock::now(); */
          /* elapsedTime.push_back({currDepthIndent() + "Precise fitting",std::chrono::duration_cast<std::chrono::microseconds>(precise_fitting - viable_hypotheses).count()}); */
          profiler.addValue("Precise fitting");

          if ((int)(selected_poses.size()) == 0){
            ROS_ERROR_STREAM("[UVDARPoseCalculator]: No suitable hypothesis found!");
            ROS_ERROR_STREAM("[UVDARPoseCalculator]: Initial hypothesis count: "<< initial_hypothesis_count << ", fitted hypothesis count: " << fitted_hypothesis_count);
            /* ROS_ERROR_STREAM("[UVDARPoseCalculator]: Points:"); */
            /* for (auto pt : points){ */
            /* ROS_ERROR_STREAM("[UVDARPoseCalculator]: " << pt); */
            /* } */
            /* ROS_ERROR_STREAM("[UVDARPoseCalculator]: Projection errors were: " ); */
            /* for (auto pre : projection_errors_backup){ */
            /* ROS_ERROR_STREAM("[UVDARPoseCalculator]: " << pre); */
            /* } */
            return false;
          }


          if (_publish_constituents_){
            std::vector<e::MatrixXd> covariances;
            for ([[ maybe_unused ]] auto &p : selected_poses){
              profiler.indent();
              /* auto [new_pose, covariance] = getCovarianceEstimate(model_, points, p, target,  image_index); */

              e::MatrixXd covariance(6,6);
              covariance.setIdentity();
              double pos_cov = 0.002;
              double rot_cov = 0.05;
              covariance(0,0) = pos_cov;
              covariance(1,1) = pos_cov;
              covariance(2,2) = pos_cov;

              covariance(3,3) = rot_cov;
              covariance(4,4) = rot_cov;
              covariance(5,5) = rot_cov;

              covariances.push_back(covariance);
              /* p=new_pose; */
              profiler.unindent();
              /* if (_debug_){ */
              /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Covariance: [\n" << covariances.back() << "\n]"); */
              /* } */
            }

            /* auto covariance_estimation = std::chrono::high_resolution_clock::now(); */
            /* elapsedTime.push_back({currDepthIndent() + "Covariance estimation",std::chrono::duration_cast<std::chrono::microseconds>(covariance_estimation - precise_fitting).count()}); */
            /* profiler.addValue("Covariance estimation"); */

            for (int i = 0; i<(int)(selected_poses.size()); i++){
              /* if (_debug_){ */
              /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: Covariance base: [\n" << covariances[i] << "\n]"); */
              /* } */
              auto constituent_pose_optical = opticalFromBase(selected_poses[i],covariances[i]);
              /* if (_debug_){ */
              /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: Covariance opt:: [\n" << constituent_pose_optical.second << "\n]"); */
              /* } */
              mrs_msgs::PoseWithCovarianceIdentified constituent;

              constituent.id = target;
              constituent.pose.position.x = constituent_pose_optical.first.first.x();
              constituent.pose.position.y = constituent_pose_optical.first.first.y();
              constituent.pose.position.z = constituent_pose_optical.first.first.z();
              constituent.pose.orientation.x = constituent_pose_optical.first.second.x();
              constituent.pose.orientation.y = constituent_pose_optical.first.second.y();
              constituent.pose.orientation.z = constituent_pose_optical.first.second.z();
              constituent.pose.orientation.w = constituent_pose_optical.first.second.w();
              for (int i=0; i<constituent_pose_optical.second.cols(); i++){
                for (int j=0; j<constituent_pose_optical.second.rows(); j++){
                  constituent.covariance[constituent_pose_optical.second.cols()*j+i] =  constituent_pose_optical.second(j,i);
                }
              }
              constituents.push_back(constituent);
            }
          }



          if ((int)(selected_poses.size()) == 1){
            final_mean = selected_poses.at(0);
            final_covariance = e::Matrix6d::Identity()*SINGLE_HYPOTHESIS_COVARIANCE;
          }
          else if ((int)(selected_poses.size()) > 1){
            /* if (((selected_poses.front().first - selected_poses.back().first).norm() < MAX_HYPOTHESIS_SPREAD) || (selected_poses.size() < 100)) */
            { if (_debug_)
              ROS_INFO_STREAM("[UVDARPoseCalculator]: " << selected_poses.size() << " equivalent hypotheses found! I will attempt to smear them together into a unified measurement.");
              /* std::tie(final_mean, final_covariance) = getMeasurementUnion(selected_poses, covariances); */
              /* std::tie(final_mean, final_covariance) = getMeasurementUnionSimple(selected_poses); */
              std::tie(final_mean, final_covariance) = getMeasurementElipsoidHull(selected_poses);
              final_covariance += e::Matrix6d::Identity()*SINGLE_HYPOTHESIS_COVARIANCE;
            }
            /* else{ */
            /*   auto v_w_s = baseFromOptical(directionFromCamPoint(points.at(0), image_index)); */
            /*   v_w_s *= 15.0; //max range */
            /*   final_mean.first << v_w_s.x(),v_w_s.y(),v_w_s.z(); */
            /*   final_mean.second = e::Quaterniond(1,0,0,0); */
            /*   final_covariance.setIdentity(6,6); */
            /*   final_covariance *= M_PI;//large covariance for angles in radians */
            /*   final_covariance.topLeftCorner(3, 3) = getLongCovariance(v_w_s,(model_.getMaxMinVisibleDiameter().first*1.0),1000.0); */

            /* } */
          }


          /* auto measurement_union = std::chrono::high_resolution_clock::now(); */
          /* elapsedTime.push_back({currDepthIndent() + "Measurement union",std::chrono::duration_cast<std::chrono::microseconds>(measurement_union - covariance_estimation).count()}); */
          profiler.addValue("Measurement union");


          /* covariance += e::MatrixXd::Identity(6,6)*0.0001; */


          /* tf::Quaternion qtemp; */
          /* qtemp.setRPY((ms.x(3)), (ms.x(4)), (ms.x(5))); */
          /* qtemp=tf::Quaternion(-0.5,0.5,-0.5,-0.5)*qtemp; //bring relative orientations to the optical frame of the camera (Roll, Pitch and Yaw were estimated in the more intuitive sensor frame (X forward, Y to the left, Z up) */
          /* qtemp.normalize();//just in case */
          }

          auto fitted_pose_optical = opticalFromBase(final_mean,final_covariance);

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

          return true;
      }
      //}

          e::Vector3d getRoughInit(LEDModel model, std::vector<e::Vector3d> v_w, [[ maybe_unused ]] int image_index){

            /* std::vector<e::Vector3d> v_w; */
            e::Vector3d v_avg = {0,0,0};
            for (auto& v: v_w){
              /* v_w.push_back(directionFromCamPoint(point, image_index)); */
              v_avg += v;
            }
            v_avg /= (double)(v_avg.size());
            v_avg = v_avg.normalized();
            /* ROS_ERROR_STREAM("[UVDARPoseCalculator]: model size: " << model.size()); */
            /* for (sub_model : getSubModels(model, (int)(observed_points.size()))) { */
            /* double d_model = getRoughVisibleDiameter(sub_model); */
            double l_max = UVDAR_RANGE;
            if ((int)(v_w.size()) > 1){
              auto [d_max, d_min] = model.getMaxMinVisibleDiameter();
              double alpha_max = getLargestAngle(v_w);

              l_max = (d_max/2.0)/tan(alpha_max/2.0);
              /* if (_debug_) */
              /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: d_max: " << d_max << "; alpha_max: " << alpha_max << "; l_rough: " << l_max); */
            }

            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: d_min: " << d_min << "; alpha_min: " << alpha_min << "; l_rough: " << l_max); */
            return baseFromOptical(v_avg*l_max)*1.25;
            /* } */
          }


          double getLargestAngle(std::vector<e::Vector3d> directions){
            double max_angle = 0;
            std::pair<int,int> sel_indices = {-1,-1};
            for (int i = 0; i < ((int)(directions.size()) - 1); i++){
              for (int j = i+1; j < (int)(directions.size()); j++){
                double tent_angle = acos(directions[i].normalized().dot(directions[j].normalized()));
                /* if (_debug_) */
                /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: tent_angle: "<< tent_angle); */
                if (tent_angle > max_angle){
                  max_angle = tent_angle;
                  sel_indices = {i,j};
                }
              }
            }

            if ((sel_indices.first != -1) && (sel_indices.second != -1)){
              if (_debug_)
                ROS_INFO_STREAM("[UVDARPoseCalculator]: selected points: " << sel_indices.first << ": " << directions[sel_indices.first].transpose() << " and " << sel_indices.second << ": " << directions[sel_indices.second].transpose());
            }
            else{
              if (_debug_)
                ROS_INFO_STREAM("[UVDARPoseCalculator]: No points selected as closest");
            }

            return max_angle;
          }

          /* std::vector<LEDModel> getSubModels(LEDModel full_model, int min_markers){ */
          /*   std::vector<LEDModel> output; */

          /* } */

          void initModelsAndAxisVectors(){ // spread out the axis of initial hypotheses - presume that Z is roughly upwards
            for (unsigned int i=0; i<_camera_count_; i++){
              axis_vectors_.push_back(std::vector<e::Vector3d>());
            }
          }

          bool prepareModelsAndAxisVectors(unsigned int image_index){ // spread out the axis of initial hypotheses - presume that Z is roughly upwards
            if (!tf_fcu_to_cam[image_index]){
              ROS_INFO_STREAM("[UVDARPoseCalculator]: Transformation for camera " << image_index << " is missing, returning.");
              return false;
            }
            const auto tf = tf_fcu_to_cam[image_index].value();

            axis_vectors_[image_index].clear();

            axis_vectors_[image_index].push_back(e::Vector3d::UnitZ());

            axis_vectors_[image_index].push_back(e::Vector3d::UnitX());
            /* axis_vectors_[image_index].push_back(-e::Vector3d::UnitX()); */
            axis_vectors_[image_index].push_back(e::Vector3d::UnitY());
            /* axis_vectors_[image_index].push_back(-e::Vector3d::UnitY()); */

            axis_vectors_[image_index].push_back(e::Vector3d(sqrt(0.5), sqrt(0.5), 0.0));
            axis_vectors_[image_index].push_back(e::Vector3d(-sqrt(0.5), sqrt(0.5), 0.0));
            /* axis_vectors_[image_index].push_back(e::Vector3d(-sqrt(0.5), -sqrt(0.5), 0.0)); */
            /* axis_vectors_[image_index].push_back(e::Vector3d(sqrt(0.5), -sqrt(0.5), 0.0)); */

            axis_vectors_[image_index].push_back(e::Vector3d(0.0, sqrt(0.5), sqrt(0.5)));
            axis_vectors_[image_index].push_back(e::Vector3d(0.0, -sqrt(0.5), sqrt(0.5)));
            axis_vectors_[image_index].push_back(e::Vector3d(sqrt(0.5), 0.0, sqrt(0.5)));
            axis_vectors_[image_index].push_back(e::Vector3d(-sqrt(0.5), 0.0, sqrt(0.5)));

            ROS_INFO_STREAM("[UVDARPoseCalculator]: camera " << image_index << " optical rotation: " << std::endl <<  tf2::transformToEigen(tf.transform).rotation());
            camera_view_[image_index] = rot_optical_to_base*tf2::transformToEigen(tf.transform).rotation();

            ROS_INFO_STREAM("[UVDARPoseCalculator]: Composed rotation matrix");
            ROS_INFO_STREAM("[UVDARPoseCalculator]: \n" << camera_view_[image_index].toRotationMatrix());

            ROS_INFO_STREAM("[UVDARPoseCalculator]: Initializing sample rotation axes for camera " << image_index);
            for (auto& av : axis_vectors_[image_index]){
              av = camera_view_[image_index]*av;
              ROS_INFO_STREAM("[UVDARPoseCalculator]:   - " << av.transpose());
            }

            return true;
          }


          /* iterFitPosition //{ */
          /* e::Vector3d iterFitPosition(LEDModel model, std::vector<cv::Point3d> observed_points, e::Vector3d rough_initialization, int target, int image_index){ */
          /*   e::Vector3d position_curr = rough_initialization; */
          /*   auto model_curr = model.translate(position_curr); */
          /*   double step_init = 0.1; */
          /*   double step = step_init; */
          /*   /1* double y_step = step_init; *1/ */
          /*   /1* double z_step = step_init; *1/ */
          /*   double error_total = totalError(model_curr, observed_points, target, image_index); */
          /*   LEDModel shape_top, shape_bottom; */
          /*   LEDModel shape_left, shape_right; */
          /*   LEDModel shape_front, shape_back; */
          /*   double top_error, bottom_error; */
          /*   double left_error, right_error; */
          /*   double front_error, back_error; */
          /*   auto gradient = e::Vector3d ( */
          /*       std::numeric_limits<double>::max(), */
          /*       std::numeric_limits<double>::max(), */
          /*       std::numeric_limits<double>::max() */
          /*       ); */
          /*   double threshold = (int)(observed_points.size())*0.001; */
          /*   int iters = 0; */
          /*   if (_debug_) */
          /*     ROS_INFO_STREAM("[UVDARPoseCalculator]: total error init: " << error_total); */
          /*   while ((error_total > threshold) && ((gradient.norm()) > 0.0001) && (iters < 100)){ */
          /*     step = step_init; */
          /*     int it=0; */
          /*     while (it<1000){ */
          /*       it++; */
          /*       shape_top     = model_curr.translate(e::Vector3d(0,0,step)); */
          /*       shape_bottom  = model_curr.translate(e::Vector3d(0,0,-step)); */
          /*       shape_left    = model_curr.translate(e::Vector3d(0,step,0)); */
          /*       shape_right   = model_curr.translate(e::Vector3d(0,-step,0)); */
          /*       shape_front   = model_curr.translate(e::Vector3d(step,0,0)); */
          /*       shape_back    = model_curr.translate(e::Vector3d(-step,0,0)); */
          /*       top_error = totalError(shape_top, observed_points, target, image_index); */
          /*       /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: top: " << top_error); *1/ */
          /*       bottom_error = totalError(shape_bottom, observed_points, target, image_index); */
          /*       /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: bottom: " << bottom_error); *1/ */
          /*       if ( (step > 0.0001) && (sgn(bottom_error - error_total) == sgn(top_error - error_total))) { */
          /*         step /= 2; */
          /*         continue; */
          /*       } */
          /*       left_error = totalError(shape_left, observed_points, target, image_index); */
          /*       /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: left: " << left_error); *1/ */
          /*       right_error = totalError(shape_right, observed_points, target, image_index); */
          /*       /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: right:" << right_error); *1/ */
          /*       if ( (step > 0.0001) && (sgn(left_error - error_total) == sgn(right_error - error_total))) { */
          /*         step /= 2; */
          /*         continue; */
          /*       } */
          /*       front_error = totalError(shape_front, observed_points, target, image_index); */
          /*       /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: front: " << front_error); *1/ */
          /*       back_error = totalError(shape_back, observed_points, target, image_index); */
          /*       /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: back: " << back_error); *1/ */
          /*       if ( (step > 0.0001) && (sgn(front_error - error_total) == sgn(back_error - error_total))) { */
          /*         step /= 2; */
          /*         continue; */
          /*       } */

          /*       break; */
          /*     } */
          /*     gradient.z() = ((top_error-bottom_error)/(2*step))/((double)(observed_points.size())); */
          /*     gradient.y() = ((left_error-right_error)/(2*step))/((double)(observed_points.size())); */
          /*     gradient.x() = ((front_error-back_error)/(2*step))/((double)(observed_points.size())); */

          /*     /1* gradient.x() = gradient.x()*x_step; *1/ */
          /*     /1* gradient.y() = gradient.y()*y_step; *1/ */
          /*     /1* gradient.z() = gradient.z()*z_step; *1/ */

          /*     /1* if ( *1/ */ 
          /*     /1*     (gradient.x() > x_step) || *1/ */
          /*     /1*     (gradient.y() > y_step) || *1/ */
          /*     /1*     (gradient.z() > z_step) *1/ */
          /*     /1*    ){ *1/ */
          /*       gradient = gradient.normalized()*step; */
          /*     /1* } *1/ */

          /*     double x_diff, y_diff, z_diff; */
          /*     if (abs(gradient.x()) > 1E-9) */
          /*       x_diff = -gradient.x(); */
          /*     else */
          /*       x_diff = 0; */
          /*     if (abs(gradient.y()) > 1E-9) */
          /*       y_diff = -gradient.y(); */
          /*     else */
          /*       y_diff = 0; */
          /*     if (abs(gradient.z()) > 1E-9) */
          /*       z_diff = -gradient.z(); */
          /*     else */
          /*       z_diff = 0; */


          /*     /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: gradient: " << gradient); *1/ */
          /*     position_curr = position_curr + e::Vector3d(x_diff, y_diff, z_diff); */
          /*     model_curr = model_curr.translate(e::Vector3d(x_diff, y_diff, z_diff)); */

          /*     error_total = totalError(model_curr, observed_points, target, image_index); */
          /*     if (_debug_){ */
          /*       ROS_INFO_STREAM("[UVDARPoseCalculator]: total error: " << error_total;); */
          /*     } */
          /*     iters++; */
          /*   } */
          /*   //final gradient check */
          /*   double mean_error = error_total/((double)(observed_points.size())); */
          /*   step = 0.10; */
          /*   /1* y_step = 0.10; *1/ */
          /*   /1* z_step = 0.10; *1/ */
          /*   shape_top   = model_curr.translate(e::Vector3d(0,0,step)); */
          /*   shape_bottom= model_curr.translate(e::Vector3d(0,0,-step)); */
          /*   shape_left  = model_curr.translate(e::Vector3d(0,step,0)); */
          /*   shape_right = model_curr.translate(e::Vector3d(0,-step,0)); */
          /*   shape_front = model_curr.translate(e::Vector3d(step,0,0)); */
          /*   shape_back  = model_curr.translate(e::Vector3d(-step,0,0)); */
          /*   gradient.x() = ((((abs(front_error)+abs(back_error))/2)-mean_error)/(step))/((double)(observed_points.size())); */
          /*   gradient.y() = ((((abs(left_error)+abs(right_error))/2)-mean_error)/(step))/((double)(observed_points.size())); */
          /*   gradient.z() = ((((abs(top_error)+abs(bottom_error))/2)-mean_error)/(step))/((double)(observed_points.size())); */

          /*   return position_curr; */
          /* } */
          //}

          std::pair<std::vector<std::pair<e::Vector3d, e::Quaterniond>>,std::vector<double>> getViableInitialHyptheses(LEDModel model, std::vector<cv::Point3d> observed_points, e::Vector3d furthest_position, int target, int image_index, double dist_step_ratio=0.1, int orientation_step_count=12){
            e::Vector3d first_position = 1.0*furthest_position.normalized();
            if (_debug_)
              ROS_INFO_STREAM("[UVDARPoseCalculator]: Range: " << (furthest_position-first_position).norm());

            LEDModel model_local = model.rotate(e::Vector3d::Zero(),camera_view_[image_index]);

            /* int dist_step_count = round((furthest_position-first_position).norm()/init_dist_step_meters); */
            [[maybe_unused]] bool is_close = false;
            /* if (dist_step_count == 0){ */
            /*   position_step = (furthest_position-first_position).normalized(); */
            /*   first_position = furthest_position; */
            /*   is_close = true; */
            /* } */
            /* else{ */ 
            /*   position_step = (furthest_position-first_position)/dist_step_count; */
            /* } */
            /* position_step = furthest_position*dist_step_ratio */

            double angle_step = 2.0*M_PI/(double)(orientation_step_count);

            std::vector<std::pair<e::Vector3d, e::Quaterniond>> acceptable_hypotheses;
            std::vector<double> errors;

            e::Vector3d position_curr = first_position;
            std::vector<std::vector<std::tuple<double,e::Vector3d,double>>> orientation_errors;
            std::vector<std::tuple<double,e::Vector3d,double>> best_orientations;

            /* for (int i=0; i<=dist_step_count; i++){ */
            while (position_curr.norm() < (furthest_position.norm()*1.2)){
              /* e::Vector3d position_step = position_curr*dist_step_ratio; */

              best_orientations.clear();
              orientation_errors.clear();
              /* best_orientation.push_back({std::numeric_limits<double>::max(), -1}); */
              for (auto v : axis_vectors_[image_index]){
                orientation_errors.push_back(std::vector<std::tuple<double,e::Vector3d,double>>());
                for (int j=0; j<orientation_step_count; j++){
                  /* use is close */

                  /* ROS_INFO_STREAM("[UVDARPoseCalculator]: pos curr: " << position_curr.transpose() ); */

                  /* double error_total = totalError(model_local.rotate(e::Vector3d(0,0,0), e::Vector3d::UnitZ(), j*angle_step).rotate(e::Vector3d(0,0,0), position_curr.normalized(), -img_rotator[image_index]).translate(position_curr), observed_points, target, image_index); */
                  /* double error_total = totalError(model_local.rotate(e::Vector3d(0,0,0), v, j*angle_step).rotate(e::Vector3d(0,0,0), position_curr.normalized(), -img_rotator[image_index]).translate(position_curr), observed_points, target, image_index); */
                  double error_total = totalError(model_local.rotate(e::Vector3d(0,0,0), v, j*angle_step).translate(position_curr), observed_points, target, image_index);
                  orientation_errors.back().push_back({error_total,v,j*angle_step});
                }
              }

              //find local orientation minima
              for (auto &orr_err : orientation_errors){
                auto orig_back = orr_err.back();
                orr_err.push_back(orr_err.front());
                orr_err.insert(orr_err.begin(), orig_back);
                for (int j = 1; j < (int)(orr_err.size())-1; j++){
                  /* double threshold = ((ERROR_THRESHOLD/position_curr.norm())*(int)(observed_points.size())); */
                  double threshold = ((ERROR_THRESHOLD_INITIAL)*(int)(observed_points.size()));
                  /* double threshold = ((ERROR_THRESHOLD)*(double)(observed_points.size()))/sqr(position_curr.norm()); */
                  /* ROS_INFO_STREAM("[UVDARPoseCalculator]: orientation error: " << std::get<0>(orr_err.at(j)) << " vs. threshold of: " << threshold << "..."); */
                  if (std::get<0>(orr_err.at(j)) < threshold){
                    /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Pass" ); */

                    /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Checking if local minimum..."); */
                    /* if ((std::get<0>(orr_err.at(j)) < std::get<0>(orr_err.at(j-1))) && (std::get<0>(orr_err.at(j)) < std::get<0>(orr_err.at(j+1)))){ */
                    {
                      /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Pass" ); */
                      best_orientations.push_back(orr_err.at(j));
                    }
                    /* else */
                    /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Fail" ); */
                  }
                  /* else */
                  /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Fail" ); */
                }
              }

              /* if ((best_orientation.back()).first > error_total){ */
              /*   best_orientation.back().first = error_total; */
              /*   best_orientation.back().second = j*angle_step; */
              /* } */
              /* } */

              /* /1* if (true){ *1/ */
              for (auto& bor : best_orientations){
                /* double threshold = ((ERROR_THRESHOLD)*(int)(observed_points.size())); */
                /* if (std::get<0>(bor) < threshold){ */
                /* } */
                /* acceptable_hypotheses.push_back(std::pair<e::Vector3d, e::Quaterniond>(position_curr, e::AngleAxisd(-img_rotator[image_index],position_curr.normalized())*e::AngleAxisd(bor.second, e::Vector3d::UnitZ()))); */
                /* acceptable_hypotheses.push_back(std::pair<e::Vector3d, e::Quaterniond>(position_curr, e::AngleAxisd(-img_rotator[image_index],position_curr.normalized())*e::AngleAxisd(std::get<2>(bor), std::get<1>(bor)))); */



                auto orientation_world = e::AngleAxisd(std::get<2>(bor), std::get<1>(bor));
                e::Quaterniond orientation_total = orientation_world * camera_view_[image_index];

                bool upside_down_check = true;
                if (REJECT_UPSIDE_DOWN){
                  if (_debug_){
                    ROS_INFO_STREAM("[UVDARPoseCalculator]: camera " << image_index << " rotation: " << std::endl <<  camera_view_[image_index].toRotationMatrix());

                    ROS_INFO_STREAM("[UVDARPoseCalculator]: transformed Z in camera link: " << (((camera_view_[image_index] * e::Vector3d::UnitZ())) ).transpose());
                    ROS_INFO_STREAM("[UVDARPoseCalculator]: transformed Z in the current orientation: " << ((orientation_world * (camera_view_[image_index] * e::Vector3d::UnitZ())) ).transpose());
                    ROS_INFO_STREAM("[UVDARPoseCalculator]: transformed Z in FCU: " << (((camera_view_[image_index].inverse())*(orientation_world * (camera_view_[image_index] * e::Vector3d::UnitZ())) ).transpose()));
                  }
                  if ((((camera_view_[image_index].inverse())*(orientation_world * (camera_view_[image_index] * e::Vector3d::UnitZ())) ).z()) < 0){
                    upside_down_check = false;
                    if (_debug_){
                      ROS_INFO_STREAM("[UVDARPoseCalculator]: Small Z: " << (camera_view_[image_index].inverse())*(orientation_world * (camera_view_[image_index] * e::Vector3d::UnitZ())) );
                    }
                  }
                }

                if ( upside_down_check ){
                  acceptable_hypotheses.push_back(std::pair<e::Vector3d, e::Quaterniond>(position_curr, orientation_total));
                  errors.push_back(std::get<0>(bor));

                }
              }

              /* position_curr+=position_step; */
              position_curr*=(1+dist_step_ratio);
            }

            return {acceptable_hypotheses, errors};
          }

          std::pair<std::pair<e::Vector3d, e::Quaterniond>, double> iterFitFull(const LEDModel& model, const std::vector<cv::Point3d>& observed_points, const std::pair<e::Vector3d, e::Quaterniond>& hypothesis, int target, int image_index)
          {
            const auto start = profiler.getTime();

            /* if (_debug_) */
            /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: Refined hypotheses for target " << target << " in image " << image_index << ": "); */
            e::Vector3d position_curr = hypothesis.first;
            const e::Quaterniond orientation_start  = hypothesis.second;
            e::Quaterniond orientation_curr  = orientation_start;
            auto model_curr = model.rotate(e::Vector3d(0,0,0), orientation_curr).translate(position_curr);

            const double pos_step_init = 1.0;
            const double angle_step_init = 0.1;

            std::shared_ptr<std::vector<cv::Point3d>> projected_points = std::make_shared<std::vector<cv::Point3d>>();
            double error_total = totalError(model_curr, observed_points, target, image_index, projected_points, _debug_);
            if (_debug_){
              ROS_INFO_STREAM("[UVDARPoseCalculator]: Proj. points:");
              for (const auto& p : *projected_points){
                ROS_INFO_STREAM("[UVDARPoseCalculator]: " << p.x << ":\t" << p.y);
              }
            }
            using vec6_t = e::Matrix<double, 6, 1>;
            vec6_t gradient = std::numeric_limits<double>::max()*vec6_t::Ones();
            const double threshold = (double)(observed_points.size())*sqr(QPIX);
            int iters = 0;
            if (_debug_)
              ROS_INFO_STREAM("[UVDARPoseCalculator]: total error init: " << error_total);
            profiler.addValueSince("Variable initialization, initial e="+std::to_string(error_total),start);


            profiler.indent();
            double prev_error_total = error_total+(threshold);
            while ((error_total > (threshold*0.1)) && ((prev_error_total - error_total) > (prev_error_total*0.1)) && (iters < 50)){
              const auto loop_start = profiler.getTime();
              prev_error_total = error_total;
              int grad_iter = 0;
              for (int dim = 0; dim < 3; dim++){
                double pos_step = 0.1;
                bool extreme = false;
                int it=0;
                double a_error = 0.0, b_error = 0.0;
                while (it<1000){//get local position gradient
                  it++;
                  grad_iter++;
                  e::Vector3d grad_vec = e::Vector3d::Zero();
                  grad_vec(dim) = pos_step;
                  const LEDModel shape_a = model_curr.translate(grad_vec);
                  const LEDModel shape_b = model_curr.translate(-grad_vec);

                  a_error = totalError(shape_a, observed_points, target, image_index);
                  b_error = totalError(shape_b, observed_points, target, image_index);
                  /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  1"); */
                  if ( (pos_step > (0.01*pos_step_init)) && (sgn(b_error - error_total) == sgn(a_error - error_total))) {
                  /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  2"); */
                    pos_step /= 2;
                    if (pos_step <= (0.01*pos_step_init)){
                  /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  3"); */
                      extreme = true;
                      break;
                    }
                    continue;
                  }
                  if (pos_step <= (0.01*pos_step_init)){
                  /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  4"); */
                    extreme = true;
                  }
                  /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  5"); */
                  break;
                }

                if (!extreme)
                  gradient(dim) = ((a_error-b_error)/(2*pos_step))/((double)(observed_points.size()));
                else 
                  gradient(dim) = 0;
              }

              profiler.addValueSince("Position gradient (it:"+std::to_string(grad_iter)+",g="+std::to_string(gradient.head<3>().norm())+")",loop_start);

              if ((gradient.head<3>().norm()) > (0.01)){//to check for extremes
                double grad_norm = gradient.head<3>().norm();
                const double m_lin_gradient = grad_norm;
                /* const double t_parameter = -0.5*m_lin_gradient; */
                const double t_parameter = 0.5*m_lin_gradient;
                double dist = 0.0;
                const double lin_step_init = 0.01;
                double alpha_lin_step = lin_step_init;
                const e::Vector3d p_step_dir = -(gradient.head<3>().normalized());
                /* const e::Vector3d p_step_dir = -(gradient.head<3>()); */
                const double error_shift_prev = error_total;
                double error_shift_curr = error_total;
                auto model_shifted_curr = model_curr;
                const auto loop_start_gradient_descent = profiler.getTime();

                int j=0;
                /* double condition = std::numeric_limits<double>::max(); */

                bool shrinking;
                model_shifted_curr = model_curr.translate(p_step_dir*alpha_lin_step);
                error_shift_curr = totalError(model_shifted_curr, observed_points, target, image_index);
                if ((error_shift_prev-error_shift_curr) >= (alpha_lin_step*t_parameter)){
                  shrinking = false;
                }
                else{
                  shrinking = true;
                }

                bool backtracking_failed = false;
                /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  Starting Armijo"); */
                /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  Initial error: " << error_total); */
                /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  Initial direction: " << p_step_dir.transpose()); */
                /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  Gradient: " << gradient.head<3>().transpose()); */
                while (j < 100){
                  j++;
                  model_shifted_curr = model_curr.translate(p_step_dir*alpha_lin_step);
                  error_shift_curr = totalError(model_shifted_curr, observed_points, target, image_index);

                  if (!shrinking){
                    /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  grow?"); */
                    /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  current erorr: " << error_shift_curr); */
                    /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  compare: " << (error_shift_prev-error_shift_curr) << " vs " << (alpha_lin_step*t_parameter)); */
                    /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  condition: " << ((error_shift_prev-error_shift_curr) - (alpha_lin_step*t_parameter))); */
                    if ((error_shift_prev-error_shift_curr) >= (alpha_lin_step*t_parameter)){
                      /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  growing"); */
                      alpha_lin_step *= 2.0;
                    }
                    else {
                      /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  terminating"); */
                      dist = alpha_lin_step*0.5;
                      break;
                    }
                  }
                  else {
                    /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  shrink?"); */
                    /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  current erorr: " << error_shift_curr); */
                    /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  compare: " << (error_shift_prev-error_shift_curr) << " vs " << (alpha_lin_step*t_parameter)); */
                    /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  condition: " << ((error_shift_prev-error_shift_curr) - (alpha_lin_step*t_parameter))); */
                    if ((error_shift_prev-error_shift_curr) < (alpha_lin_step*t_parameter)){
                      /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  shrinking"); */
                      alpha_lin_step *= 0.5;
                    }
                    else {
                      /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  terminating"); */
                      dist = alpha_lin_step*2.0;
                      break;
                    }
                  }

                  if ((alpha_lin_step < 0.01) || (alpha_lin_step > 20)){
                    backtracking_failed = true;
                    /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  Breaking Armijo, step is extreme."); */
                    break;
                  }
                }
                if (!backtracking_failed){
                  position_curr += p_step_dir*dist;
                  model_curr = model_curr.translate(p_step_dir*dist);
                  error_total = totalError(model_curr, observed_points, target, image_index);;
                  /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  Moved the hypothesis by "<< dist << "m"); */
                }
                /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  Final error: " << error_total); */

                profiler.addValueSince("Position fitting - iter. "+std::to_string(j)+" (e="+std::to_string(error_total)+")",loop_start_gradient_descent);
            }

            iters++;
            profiler.addValueSince("Iteration loop position "+std::to_string(iters),loop_start);
            }

            iters = 0;

            while ((error_total > (threshold*0.1)) && ((prev_error_total - error_total) > (prev_error_total*0.1)) && (iters < 50)){
              const auto loop_start = profiler.getTime();
              const auto rot_steps = profiler.getTime();
              prev_error_total = error_total;
              int grad_iter = 0;
              for (int dim = 0; dim < 3; dim++){
                double angle_step = 0.1;
                bool extreme = false;
                int it=0;
                double a_error = 0.0, b_error = 0.0;
                while (it<1000){
                  it++;
                  grad_iter++;

                  e::Vector3d grad_axis_vec = e::Vector3d::Zero();
                  grad_axis_vec(dim) = 1;
                  const LEDModel shape_a = model_curr.rotate(position_curr,  orientation_curr*grad_axis_vec, angle_step);
                  const LEDModel shape_b = model_curr.rotate(position_curr,  orientation_curr*grad_axis_vec, -angle_step);
                  a_error = totalError(shape_a, observed_points, target, image_index);
                  /* ROS_INFO_STREAM("[UVDARPoseCalculator]: ccw: " << a_error); */
                  b_error = totalError(shape_b, observed_points, target, image_index);
                  /* ROS_INFO_STREAM("[UVDARPoseCalculator]: cw: " << b_error); */

                  if ( (angle_step > (0.1*angle_step_init)) && (sgn(b_error - error_total) == sgn(a_error - error_total))) {
                    angle_step /= 2;
                    if (angle_step <= (0.1*angle_step_init)){
                      extreme = true;
                      break;
                    }
                    continue;
                  }
                  break;
                }

                if (!extreme)
                  gradient(3+dim) = ((a_error-b_error)/(2*angle_step))/((double)(observed_points.size()));
                else
                  gradient(3+dim) = 0;
              }

              profiler.addValueSince("Orientation gradient (it:"+std::to_string(grad_iter)+",g="+std::to_string(gradient.head<3>().norm())+")", rot_steps);

              if ((gradient.tail<3>().norm()) > (0.1)){//to check for extremes
                const double rot_step_init = angle_step_init/2;
                double alpha_rot_step = rot_step_init;
                const e::Vector3d norm_gradient = gradient.tail<3>().normalized()*0.01;//small, to avoid cross-axis influence
                const e::Vector3d p_step_axis =
                  e::AngleAxisd(
                      e::AngleAxisd(-norm_gradient(0), e::Vector3d::UnitX()) *
                      e::AngleAxisd(-norm_gradient(1), e::Vector3d::UnitY()) *
                      e::AngleAxisd(-norm_gradient(2), e::Vector3d::UnitZ())
                      ).axis();

                /* -(gradient.topRightCorner(3,1).normalized()); */
                const double error_rot_prev = error_total;
                double error_rot_curr = error_total;
                auto model_rotated_curr = model_curr;

                const double m_rot_gradient = gradient.tail<3>().norm();
                const double t_parameter = 0.5*m_rot_gradient;
                double angle = 0.0;
                int j=0;
                /* double condition = std::numeric_limits<double>::max(); */

                bool shrinking;
                model_rotated_curr = model_curr.rotate(position_curr, p_step_axis,alpha_rot_step);
                error_rot_curr = totalError(model_rotated_curr, observed_points, target, image_index);
                if ((error_rot_prev-error_rot_curr) >= (alpha_rot_step*t_parameter)){
                  shrinking = false;
                }
                else{
                  shrinking = true;
                }

                bool backtracking_failed = false;
                while (j < 10){
                  j++;
                  model_rotated_curr = model_curr.rotate(position_curr, p_step_axis,alpha_rot_step);
                  error_rot_curr = totalError(model_rotated_curr, observed_points, target, image_index);
                  
                  if (!shrinking){
                    if ((error_rot_prev-error_rot_curr) >= (alpha_rot_step*t_parameter)){
                      /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  growing"); */
                      alpha_rot_step *= 2.0;
                    }
                    else {
                      /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  terminating"); */
                      angle = alpha_rot_step*0.5;
                      break;
                    }
                  }
                  else {
                    if ((error_rot_prev-error_rot_curr) < (alpha_rot_step*t_parameter)){
                      /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  shrinking"); */
                      alpha_rot_step *= 0.5;
                    }
                    else {
                      /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  terminating"); */
                      angle = alpha_rot_step*2.0;
                      break;
                    }
                  }

                  if ((alpha_rot_step < 0.01) || (alpha_rot_step > 0.2)){
                    backtracking_failed = true;
                    /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  Breaking Armijo, step is extreme."); */
                    break;
                  }
                }
                if (!backtracking_failed){
                  orientation_curr = e::AngleAxisd(angle,p_step_axis)*orientation_curr;
                  model_curr = model_curr.rotate(position_curr, p_step_axis,angle);
                  error_total = totalError(model_curr, observed_points, target, image_index);;
                  /* ROS_INFO_STREAM("[UVDARPoseCalculator]:  Moved the hypothesis by "<< rad2deg(angle) << "deg"); */
                }

                profiler.addValue("Orientation fitting - iter. "+std::to_string(j)+" (e="+std::to_string(error_total)+")");
              }

              if ((orientation_curr.angularDistance(orientation_start)) > (deg2rad(90))){
                error_total = -1;
                ROS_INFO_STREAM("[UVDARPoseCalculator]: Skipping hypothesis - fitting went too far.");
                break;
              }

              iters++;
              profiler.addValueSince("Iteration loop orientation "+std::to_string(iters),loop_start);
              }
              profiler.unindent();

              profiler.addValue("Main fitting loop (e="+std::to_string(error_total)+")");

              profiler.addValue("Final operations");

              return {{position_curr, orientation_curr}, error_total};
      }


      double totalError(const LEDModel& model, std::vector<cv::Point3d> observed_points, int target, int image_index, std::shared_ptr<std::vector<cv::Point3d>> projected_points={}, bool return_projections=false, bool discrete_pixels=false){
        struct ProjectedMarker {
          cv::Point2d position;
          /* int freq_id; */
          int signal_id;
          double cos_view_angle;
          double distance;
        };
        double total_error = 0;

        /* if (return_projections){ */
        /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: A"); */
        /* } */
        if (return_projections && projected_points){
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: B"); */
          projected_points->clear();
        }

        std::vector<ProjectedMarker> projected_markers;
        for (auto marker : model){
          auto curr_projected =  camPointFromModelPoint(marker, image_index);

          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: marker: pos: " << marker.position.transpose() << " rot: " << quaternionToRPY(marker.orientation).transpose()  << " : " << (target*signals_per_target_)+marker.signal_id); */
          if (
              (curr_projected.first.x>-0.5) && // edge of the leftmost pixel
              (curr_projected.first.y>-0.5) && // edge of the topmost pixel
              (curr_projected.first.x<(_oc_models_[image_index].width + 0.5)) && // edge of the rightmost pixel
              (curr_projected.first.y<(_oc_models_[image_index].height+ 0.5)) // edge of the bottommost pixel
             ){
            projected_markers.push_back({
                .position = curr_projected.first,
                .signal_id = marker.signal_id,
                .cos_view_angle = curr_projected.second,
                .distance = marker.position.norm()
                });
          }


          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: fid:  " << (target*_frequencies_per_target_)+projected_markers.back().signal_id  << "; target: " << target << "; lfid: " << projected_markers.back().signal_id); */
        }

        std::vector<ProjectedMarker> selected_markers;
        for (auto marker : projected_markers){
          double distance = marker.distance;
          double cos_angle =  marker.cos_view_angle;
          double led_intensity =
            round(std::max(.0, cos_angle) * (led_projection_coefs_[0] + (led_projection_coefs_[1] / ((distance + led_projection_coefs_[2]) * (distance + led_projection_coefs_[2])))));
          /* if (return_projections){ */
          /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: cos_angle: " << cos_angle << " distance: " << distance << " led_intensity: " << led_intensity); */
          /* } */
          if (led_intensity > 0) { // otherwise they will probably not be visible
            selected_markers.push_back(marker);
            /* if (return_projections){ */
            /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: C"); */
            /* } */
          }
        }

        /* ROS_INFO_STREAM("[UVDARPoseCalculator]: projected_markers: " << projected_markers.size() << " vs. selected_markers (i): " << selected_markers.size()); */

        for (int i = 0; i < ((int)(selected_markers.size()) - 1); i++){
          for (int j = i+1; j < (int)(selected_markers.size()); j++){
            double tent_dist = cv::norm(selected_markers[i].position-selected_markers[j].position);
            if (tent_dist < 3){
              if (selected_markers[i].signal_id == selected_markers[j].signal_id){ // if the frequencies are the same, they tend to merge. Otherwise, the result varies
                selected_markers[i].position = (selected_markers[i].position + selected_markers[j].position)/2; //average them
                selected_markers.erase(selected_markers.begin()+j); //remove the other
                j--; //we are not expecting many 2+ clusters, so we will not define special case for this
              }
            }
          }
        }

        /* for (auto pt : observed_points){ */
        /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: observed_marker: " << pt.x << " : " << pt.y << " : " << pt.z); */
        /* } */
        /* for (auto pt : selected_markers){ */
        /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: projected_marker: " << pt.position.x << " : " << pt.position.y << " : " << pt.signal_id); */
        /* } */

        if (return_projections && projected_points){
          for (auto pt : selected_markers){
            projected_points->push_back(cv::Point3d(pt.position.x,pt.position.y,pt.signal_id));
          }
        }

        for (auto& obs_point : observed_points){

          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: observed marker:  " << obs_point); */
          ProjectedMarker closest_projection;
          double closest_distance = std::numeric_limits<double>::max();
          bool any_match_found = false;
          /* for (auto& proj_point : projected_markers){ */
          for (auto& proj_point : selected_markers){
            /* double tent_signal_distance = abs( (1.0/(_signal_ids_[(target*signals_per_target_)+proj_point.signal_id])) - (1.0/(obs_point.z)) ); */
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: signal id:  " << (target*signals_per_target_)+proj_point.signal_id); */
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: signal ids size:  " << _signal_ids_.size()); */
            /* if (tent_signal_distance < (2.0/estimated_framerate_[image_index])){ */
            if (_signal_ids_.at(((target%1000)*signals_per_target_)+proj_point.signal_id) == (int)(obs_point.z)){
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

          total_error += (UNMATCHED_PROJECTED_POINT_PENALTY) * std::max(0,(int)(selected_markers.size() - observed_points.size()));
          total_error += (UNMATCHED_OBSERVED_POINT_PENALTY) * std::max(0,(int)(observed_points.size() - selected_markers.size()));

          return total_error;
        }

        std::pair<std::pair<e::Vector3d, e::Quaterniond>,e::MatrixXd> getCovarianceEstimate(LEDModel model, std::vector<cv::Point3d> observed_points, std::pair<e::Vector3d, e::Quaterniond> pose, int target, int image_index){

          LEDModel model_local = model.rotate(e::Vector3d::Zero(),pose.second).translate(pose.first);

          e::MatrixXd output;

          double trans_scale = 0.1;
          /* rot_scale = 0.05; */
          double rot_scale = 0.1;
          /* auto Y0 = pose; */
          /* e::MatrixXd Y(6,(3*3*3*3*3*3)); */
          e::MatrixXd Y(6,(2*2*2*2*2*2+2*6));
          std::vector<double> Xe;

          /* Y(0,0) = 0; */
          /* Y(1,0) = 0; */
          /* Y(2,0) = 0; */
          /* Y(3,0) = 0; */
          /* Y(4,0) = 0; */
          /* Y(5,0) = 0; */

          double error_init = totalError(model_local, observed_points, target, image_index, {}, false, true);
          /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: terror: " << Xe.back() << " at: [  0, 0, 0, 0, 0, 0  ]"); */

          std::vector<int> j = {-1,1}; // for 6D, we need 21 independent samples. Accounting for point symmetry, this is 42. 6D hypercube has 64 vertices, which is sufficient.
          int k = 0;

          auto Y_rpy = quaternionToRPY(pose.second);
          if (_debug_){
            ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: RPY: [ R=" << Y_rpy(0)<< ", P=" << Y_rpy(1) << ", Y=" << Y_rpy(2) << " ]");
          }

          for (auto x_s : j){
            for (auto y_s : j){
              for (auto z_s : j){
                for (auto roll_s : j){
                  for (auto pitch_s : j){
                    for (auto yaw_s : j){
                      e::Quaterniond rotation(
                          /* e::AngleAxisd(yaw_s*rot_scale,    camera_view_[image_index]*e::Vector3d(0,0,1)) * */
                          /* e::AngleAxisd(pitch_s*rot_scale,  camera_view_[image_index]*e::Vector3d(0,1,0)) * */
                          /* e::AngleAxisd(roll_s*rot_scale,   camera_view_[image_index]*e::Vector3d(1,0,0)) */
                          e::AngleAxisd(yaw_s*rot_scale,    e::Vector3d(0,0,1)) *
                          e::AngleAxisd(pitch_s*rot_scale,  e::Vector3d(0,1,0)) *
                          e::AngleAxisd(roll_s*rot_scale,   e::Vector3d(1,0,0))
                          );
                      auto model_curr = model_local.rotate(pose.first,  rotation);
                      model_curr = model_curr.translate(e::Vector3d(x_s, y_s, z_s)*trans_scale);

                      Xe.push_back(abs(totalError(model_curr, observed_points, target, image_index, {}, false, true)-error_init));
                      if (_debug_){
                        ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: terror: " << Xe.back() << " at: [ " << x_s<< ", " << y_s<< ", " << z_s << ", "<< roll_s<< ", " << pitch_s << ", " << yaw_s << " ]");
                      }
                      Y(0,k) = x_s*trans_scale;
                      Y(1,k) = y_s*trans_scale;
                      Y(2,k) = z_s*trans_scale;
                      Y(3,k) = roll_s*rot_scale;
                      Y(4,k) = pitch_s*rot_scale;
                      Y(5,k) = yaw_s*rot_scale;
                      /* } */
                      k++;
                  }
                }
              }
            }
          }
        }

        for (int i=0; i<6; i++){
          std::vector<double> shifts(6,0.0);// X Y Z ROLL PITCH YAW
          for (auto s : j){
            shifts.at(i) = s;

            e::Quaterniond rotation(
                /* e::AngleAxisd(shifts.at(5)*rot_scale,    camera_view_[image_index]*e::Vector3d(0,0,1)) * */
                /* e::AngleAxisd(shifts.at(4)*rot_scale,  camera_view_[image_index]*e::Vector3d(0,1,0)) * */
                /* e::AngleAxisd(shifts.at(3)*rot_scale,   camera_view_[image_index]*e::Vector3d(1,0,0)) */
                e::AngleAxisd(shifts.at(5)*rot_scale,    e::Vector3d(0,0,1)) *
                e::AngleAxisd(shifts.at(4)*rot_scale,  e::Vector3d(0,1,0)) *
                e::AngleAxisd(shifts.at(3)*rot_scale,   e::Vector3d(1,0,0))
                );
            auto model_curr = model_local.rotate(pose.first,  rotation);
            model_curr = model_curr.translate(e::Vector3d(shifts.at(0), shifts.at(1), shifts.at(2))*trans_scale);

            Xe.push_back(abs(totalError(model_curr, observed_points, target, image_index, {}, false, true)-error_init));
            if (_debug_){
              ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: terror: " << Xe.back() << " at: [ " << shifts.at(0)<< ", " << shifts.at(1)<< ", " << shifts.at(2) << ", "<< shifts.at(3)<< ", " << shifts.at(4) << ", " << shifts.at(5) << " ]");
            }
            Y(0,k) = shifts.at(0)*trans_scale;
            Y(1,k) = shifts.at(1)*trans_scale;
            Y(2,k) = shifts.at(2)*trans_scale;
            Y(3,k) = shifts.at(3)*rot_scale;
            Y(4,k) = shifts.at(4)*rot_scale;
            Y(5,k) = shifts.at(5)*rot_scale;

            k++;

          }
        }


        e::VectorXd W(Xe.size());
        int i = 0;
        double Wsum = 0;
        for (auto xe : Xe){
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: xe("<<i<<") = " << xe); */
          if (xe < 0.00001)
            W(i) = sqr(QPIX);
          else
            W(i) = (sqr(QPIX)/sqrt(xe));
          Wsum += W(i);
          /* if (W(i) > 0.1){ */
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: W("<<i<<") = " << W(i)); */
          /* } */
          i++;
        }
        /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Wsum: [\n" << Wsum << "\n]"); */
        /* ROS_INFO_STREAM("[UVDARPoseCalculator]: W_orig: [\n" << W << "\n]"); */
        W /= (Wsum);
        /* W *= sqr(QPIX)*observed_points.size(); */
        /* W *= sqr(0.8); */
        /* ROS_INFO_STREAM("[UVDARPoseCalculator]: W: [\n" << W << "\n]"); */
        /* auto y = Y*(W/Wsum); */
        auto y = Y*W;
        /* ROS_INFO_STREAM("[UVDARPoseCalculator]: y: [\n" << y << "\n]"); */
        auto Ye = (Y-y.replicate(1,W.size()));
        /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Ye: [\n" << Ye << "\n]"); */

        /* i=0; */
        /* for (auto xe : Xe){ */
        /*   if ((Ye*W(i)) > 0.1){ */
        /*     ROS_ERROR_STREAM("[UVDARPoseCalculator]: W("<<i<<") = " << W(i)); */
        /*   } */
        /*   i++; */
        /* } */

        auto P = Ye*W.asDiagonal()*Ye.transpose();
        /* e::JacobiSVD<e::MatrixXd> svd(P, e::ComputeThinU | e::ComputeThinV); */
        /* if (P.topLeftCorner(3,3).determinant() > 0.001){ */

        /* ROS_INFO_STREAM("[UVDARPoseCalculator]: P: [\n" << P << "\n]"); */
        /* ROS_INFO_STREAM("[UVDARPoseCalculator]: P determinant: [\n" << P.topLeftCorner(3,3).determinant() << "\n]"); */
        /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Singular values: [\n" << svd.singularValues() << "\n]"); */
        /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Matrix V: [\n" << svd.matrixV() << "\n]"); */
        /* } */


        /* output.setIdentity(6,6); */
        return {{pose.first+e::Vector3d(y(0,0), y(1,0), y(2,0)),pose.second},P};
        }

        //Implemented based on "Generalised Covariance Union: A Unified Approach to Hypothesis Merging in Tracking" by STEVEN REECE and STEPHEN ROBERTS
        //and
        //"Generalized Information Representation and Compression Using Covariance Union"  by Ottmar Bochardt et. al. (error: Eq. (12) and (13) have - instead of +)
        std::pair<std::pair<e::Vector3d, e::Quaterniond>, e::MatrixXd> getMeasurementUnion(std::vector<std::pair<e::Vector3d, e::Quaterniond>> means, std::vector<e::MatrixXd> covariances){
          if (means.size() != covariances.size()){
            ROS_ERROR_STREAM("[UVDARPoseCalculator]: The count of means and covariances for union generation does not match. Returning!");
            return std::pair<std::pair<e::Vector3d, e::Quaterniond>, e::MatrixXd>();
          }


          /* std::pair<std::pair<e::Vector3d, e::Quaterniond>, e::MatrixXd> measurement_union; */
          std::vector<std::pair<std::pair<e::Vector3d, e::Quaterniond>, e::MatrixXd>> measurement_unions_prev;
          std::vector<std::pair<std::pair<e::Vector3d, e::Quaterniond>, e::MatrixXd>> measurement_unions_next;

          for (int i = 0; i< (int)(means.size()); i++){
            measurement_unions_prev.push_back({means[i], covariances[i]});
          }

          //pair-wise approach - if performance becomes a problem, we can consider batch solution for later
          //
          /* if (means.size() > 2){ // let's start with the two most mutually distant ones to avoid the drawbacks of serial pair-wise unionization */
          /*   std::tuple<int, int, double> dist_pair =  {-1,-1, 0}; */
          /*   for (int i=0; i<(int)(means.size()); i++){ */
          /*     for (int j=0; j<i; j++){ */
          /*       double mutual_distance = (means[i].first - means[j].first).norm(); */
          /*       if (mutual_distance > std::get<2>(dist_pair) ){ */
          /*         std::get<0>(dist_pair) = i; */
          /*         std::get<1>(dist_pair) = j; */
          /*         std::get<2>(dist_pair) = mutual_distance; */
          /*       } */
          /*     } */
          /*   } */

          /*   if ((std::get<0>(dist_pair) >= 0) && (std::get<1>(dist_pair) >= 0)) */
          /*     measurement_union = twoMeasurementUnion({means[std::get<0>(dist_pair)],covariances[std::get<0>(dist_pair)]}, {means[std::get<1>(dist_pair)], covariances[std::get<1>(dist_pair)]}); */
          /* } */
          /* return measurement_union; */

          /* if (means.size() < 1){ */
          /*   ROS_ERROR_STREAM("[UVDARPoseCalculator]: No measurements!"); */
          /*   measurement_union = {{e::Vector3d(),e::Quaterniond()},{e::MatrixXd::Identity(6,6)}}; */
          /*   return measurement_union; */
          /* } */

          /* measurement_union = {means[0],covariances[0]}; */
          /* for (int i=1; i<(int)(means.size()); i++){ */
          /*   measurement_union = twoMeasurementUnion(measurement_union, {means[i], covariances[i]}); */
          /* } */

          /* return measurement_union; */

          /* if (_debug_) */
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Meas count: "<< means.size());

          while((int)(measurement_unions_prev.size()) > 1){
            for (int i = 0; i< (int)(measurement_unions_prev.size()/2); i++){

              /* ROS_INFO_STREAM("[UVDARPoseCalculator]: a: "<< measurement_unions_prev.at(2*i).first.first.transpose() << ", b: " <<measurement_unions_prev.at((2*i)+1).first.first.transpose()); */
              ROS_INFO_STREAM("[UVDARPoseCalculator]: a: "<< quaternionToRPY(measurement_unions_prev.at(2*i).first.second).transpose() << ", b: " << quaternionToRPY(measurement_unions_prev.at((2*i)+1).first.second).transpose());
              ROS_INFO_STREAM("[UVDARPoseCalculator]: Ca: "<< measurement_unions_prev.at(2*i).second.bottomRightCorner(3,3).eigenvalues().transpose() << ", Cb: " << measurement_unions_prev.at((2*i)+1).second.bottomRightCorner(3,3).eigenvalues().transpose());
              measurement_unions_next.push_back(twoMeasurementUnion(measurement_unions_prev.at(2*i), measurement_unions_prev.at((2*i)+1)));
              /* ROS_INFO_STREAM("[UVDARPoseCalculator]: u: "<< measurement_unions_next.back().first.first.transpose()); */
              ROS_INFO_STREAM("[UVDARPoseCalculator]: u: "<< quaternionToRPY(measurement_unions_next.back().first.second).transpose());
              ROS_INFO_STREAM("[UVDARPoseCalculator]: Cu: "<< measurement_unions_next.back().second.bottomRightCorner(3,3).eigenvalues().transpose());

            }
            if ((measurement_unions_prev.size() % 2) != 0){
              measurement_unions_next.push_back(measurement_unions_prev.back());
            }
            measurement_unions_prev = measurement_unions_next;
            measurement_unions_next.clear();
          }

          return measurement_unions_prev.at(0);

        }

        std::pair<std::pair<e::Vector3d, e::Quaterniond>, e::MatrixXd> getMeasurementUnionSimple(std::vector<std::pair<e::Vector3d, e::Quaterniond>> meas){
          if (meas.size() < 1){
            ROS_ERROR_STREAM("[UVDARPoseCalculator]: No hypotheses provided. Returning!");
            return std::pair<std::pair<e::Vector3d, e::Quaterniond>, e::MatrixXd>();
          }
          e::Vector3d mean_pos(0,0,0);

          for (auto &m : meas){
            mean_pos += m.first;
          }
          mean_pos /= (double)(meas.size());

          e::Quaterniond mean_rot = getAverageOrientation(meas);

          e::MatrixXd Mp(3,(unsigned int)(meas.size()));
          e::MatrixXd Mo(3,(unsigned int)(meas.size()));

          int i = 0;
          for (auto &m : meas){
            Mp.block(0,i, 3,1) = m.first-mean_pos;
            Mo.block(0,i, 3,1) = quaternionToRPY(m.second*mean_rot.inverse());
            i++;
          }

          e::Matrix3d Cp = (Mp*Mp.transpose())/(meas.size()-1);
          e::Matrix3d Co = (Mo*Mo.transpose())/(meas.size()-1);

          e::Matrix6d C = e::Matrix6d::Zero();
          C.topLeftCorner(3,3) = Cp;
          C.bottomRightCorner(3,3) = Co;

          return {{mean_pos,mean_rot},C};

        }

        //contains code from https://gist.github.com/PeteBlackerThe3rd/f73e9d569e29f23e8bd828d7886636a0
        e::Quaterniond getAverageOrientation(std::vector<std::pair<e::Vector3d,e::Quaterniond>> meas){
          if (meas.size() < 1){
            return e::Quaterniond::Identity();
            ROS_ERROR_STREAM("[UVDARPoseCalculator]: No measurements provided. Returning!");
          }

          // first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
          Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

          for (auto &m : meas){
            A += m.second.coeffs()*m.second.coeffs().transpose();
          }
          A /= (double)(meas.size());

          // Compute the SVD of this 4x4 matrix
          Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

          Eigen::VectorXd singularValues = svd.singularValues();
          Eigen::MatrixXd U = svd.matrixU();

          // find the eigen vector corresponding to the largest eigen value
          int largestEigenValueIndex = 0;
          double largestEigenValue = singularValues(0);

          for (int i=1; i<singularValues.rows(); ++i) {
            if (singularValues(i) > largestEigenValue) {
              largestEigenValue = singularValues(i);
              largestEigenValueIndex = i;
            }
          }

          /* if (largestEigenValueIndex == -1){ */
          /*   ROS_ERROR("[UVDARPoseCalculator]: Failed to obtain orientation mean. Returning!"); */
          /*   return std::pair<std::pair<e::Vector3d, e::Quaterniond>, e::MatrixXd>(); */
          /* } */

          Eigen::Quaterniond mean_rot;
          mean_rot.x() = U(0, largestEigenValueIndex);
          mean_rot.y() = U(1, largestEigenValueIndex);
          mean_rot.z() = U(2, largestEigenValueIndex);
          mean_rot.w() = U(3, largestEigenValueIndex);

          return mean_rot;
        }


        std::pair<std::pair<e::Vector3d, e::Quaterniond>, e::MatrixXd> getMeasurementElipsoidHull(std::vector<std::pair<e::Vector3d, e::Quaterniond>> meas){
          if (meas.size() < 1){
            ROS_ERROR_STREAM("[UVDARPoseCalculator]: No hypotheses provided. Returning!");
            return std::pair<std::pair<e::Vector3d, e::Quaterniond>, e::MatrixXd>();
          }

          // position
          std::vector<e::Vector3d> meas_pos;

          e::Vector3d mean_pos(0.0,0.0,0.0);

          for (auto &m : meas){
            meas_pos.push_back(m.first);
            mean_pos += m.first;
          }
          mean_pos /= ((double)(meas.size()));
          std::vector<e::Vector3d> meas_pos_diff;

          // orientation
          e::Quaterniond mean_rot = getAverageOrientation(meas);
          std::vector<e::Vector3d> meas_rpy_diff;

          for (auto &m : meas){
            meas_pos_diff.push_back(m.first-mean_pos);
            meas_rpy_diff.push_back(quaternionToRPY(m.second*mean_rot.inverse()));
          }

          /* auto Hp = get3DEnclosingEllipsoid(meas_pos_diff,0.001); */
          auto Hp = get3DEnclosingEllipsoid(meas_pos_diff);
          auto Ho = get3DEnclosingEllipsoid(meas_rpy_diff);
          
          e::Vector3d mean_pos_shift = Hp.first;
          
          e::Quaterniond mean_rot_shift =
                      e::AngleAxisd(Ho.first(0), e::Vector3d::UnitX()) *
                      e::AngleAxisd(Ho.first(1), e::Vector3d::UnitY()) *
                      e::AngleAxisd(Ho.first(2), e::Vector3d::UnitZ());

          e::Matrix6d C = e::Matrix6d::Zero();
          C.topLeftCorner(3,3) = Hp.second;
          C.bottomRightCorner(3,3) = Ho.second;

          return {{mean_pos+mean_pos_shift,mean_rot*mean_rot_shift},C};
        }

        //based on [Nima Moshtagh (2022). Minimum Volume Enclosing Ellipsoid (https://www.mathworks.com/matlabcentral/fileexchange/9542-minimum-volume-enclosing-ellipsoid), MATLAB Central File Exchange. Retrieved May 11, 2022.]
        //
        //condition and initialization changed to one from [Michael J. Todd; E. Alper Yıldırım (2005). On Khachiyan’s Algorithm for the Computation of Minimum Volume Enclosing Ellipsoids] which seems to work much better
        //
        std::pair<e::Vector3d, e::Matrix3d> get3DEnclosingEllipsoid(std::vector<e::Vector3d> Pv){
          //function [A , c] = MinVolEllipse(P, tolerance)

          // [A , c] = MinVolEllipse(P, tolerance)
          // Finds the minimum volume enclosing ellipsoid (MVEE) of a set of data
          // points stored in matrix P. The following optimization problem is solved: 
          //
          // minimize       log(det(A))
          // subject to     (P_i - c)' * A * (P_i - c) <= 1
          //                
          // in variables A and c, where P_i is the i-th column of the matrix P. 
          // The solver is based on Khachiyan Algorithm, and the final solution 
          // is different from the optimal value by the pre-spesified amount of 'tolerance'.
          //
          // inputs:
          //---------
          // P : (d x N) dimnesional matrix containing N points in R^d.
          // tolerance : error in the solution with respect to the optimal value.
          //
          // outputs:
          //---------
          // A : (d x d) matrix of the ellipse equation in the 'center form': 
          // (x-c)' * A * (x-c) = 1 
          // c : 'd' dimensional vector as the center of the ellipse. 
          // 
          // example:
          // --------
          //      P = rand(5,100);
          //      [A, c] = MinVolEllipse(P, .01)
          //
          //      To reduce the computation time, work with the boundary points only:
          //      
          //      K = convhulln(P');  
          //      K = unique(K(:));  
          //      Q = P(:,K);
          //      [A, c] = MinVolEllipse(Q, .01)
          //
          //
          // Nima Moshtagh (nima@seas.upenn.edu)
          // University of Pennsylvania
          //
          // December 2005
          // UPDATE: Jan 2009
          //%%%%%%%%%%%%%%%%%%%% Solving the Dual problem%%%%%%%%%%%%%%%%%%%%%%%%%%%5
          // ---------------------------------
          // data points 
          // -----------------------------------
          int d = 3;
          double n = (double)(d+1);
          int N = (int)(Pv.size());
          e::MatrixXd P = stdVecOfVectorsToMatrix(Pv);

          e::MatrixXd Q = e::MatrixXd::Constant(4,N,1.0);
          Q.block(0,0, 3,N) = P;
          // initializations
          // -----------------------------------
          int count = 0;
          [[maybe_unused]] double err = 1.0;
          /* e::VectorXd u = (1.0/((double)(N))) * e::VectorXd::Constant(N,1.0);          // 1st iteration */
          e::VectorXd u = e::VectorXd::Zero(N);

          //initial volume approximation 
          {
            std::vector<e::Vector3d> Pv_local = Pv;
            if (N <= (2*d)){
              u = e::VectorXd::Constant(N,(1.0/((double)(N)))); 
            }
            else {
              u = e::VectorXd::Zero(N);
              int span_dim = 0;
              int compliant_count = 0;
              e::MatrixXd psi(3,0);
              while (span_dim < d){
                e::Vector3d direction;
                if (span_dim == 0){
                  direction = e::Vector3d(1.0,0.0,0.0);
                }
                else {
                  e::FullPivLU<e::MatrixXd> lu(psi.transpose());
                  e::MatrixXd l_null_space = lu.kernel();
                  /* ROS_INFO_STREAM("[UVDARPoseCalculator]: span: \n" << psi); */
                  /* ROS_INFO_STREAM("[UVDARPoseCalculator]: nullspace: \n" << l_null_space); */
                  direction = l_null_space.topLeftCorner(3,1).normalized();
                }
                /* ROS_INFO_STREAM("[UVDARPoseCalculator]: direction: " << direction); */

                double alpha = std::numeric_limits<double>::lowest();
                double beta = std::numeric_limits<double>::max();
                e::Vector3d a_alpha, a_beta;
                int j_alpha = -1, j_beta = -1;
                int j = 0;
                for (e::Vector3d &v : Pv_local){
                  if (v.array().isNaN().any()){
                    j++;
                    continue;
                  }
                  double dirtest = direction.transpose()*v;
                  /* ROS_INFO_STREAM("[UVDARPoseCalculator]: dirtest: " << dirtest); */
                  if (dirtest > alpha){
                    alpha = dirtest;
                    a_alpha = v;
                    j_alpha = j;
                  }
                  if (dirtest < beta){
                    beta = dirtest;
                    a_beta = v;
                    j_beta = j;
                  }
                  j++;
                }
                Pv_local[j_alpha] = e::Vector3d(std::nan(""),std::nan(""),std::nan(""));//so that we won't get duplicates
                Pv_local[j_beta] = e::Vector3d(std::nan(""),std::nan(""),std::nan(""));//so that we won't get duplicates
                /* ROS_INFO_STREAM("[UVDARPoseCalculator]: alpha: " << alpha << ", beta: " << beta); */
                /* ROS_INFO_STREAM("[UVDARPoseCalculator]: j_alpha: " << j_alpha << ", j_beta: " << j_beta); */
                if ((j_alpha == -1) || (j_beta == -1)){
                  /* ROS_ERROR("[UVDARPoseCalculator]: index -1 on enclosing ellipsoid initialization!"); */
                }
                if (u(j_alpha)<1){
                  compliant_count++;
                  u(j_alpha) = 1.0;
                }
                if (u(j_beta)<1){
                  compliant_count++;
                  u(j_beta) = 1.0;
                }
                e::MatrixXd psi_new(3,span_dim+1);
                psi_new << psi, (a_beta-a_alpha).normalized();
                psi = psi_new;
                span_dim++;
              }
              u = u*(1.0/(double)(compliant_count));
              /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Basing initial u on " << compliant_count << " points"); */
            }
          }
          // Khachiyan Algorithm
            // -----------------------------------
          /* e::VectorXd cmp = e::VectorXd::Constant(N,((1+eps)*n)); */
          while (count < 1000){
            e::Matrix4d X = Q * u.asDiagonal() * Q.transpose();       // X = \sum_i ( u_i * q_i * q_i')  is a (d+1)x(d+1) matrix
            e::VectorXd m = (Q.transpose() * X.inverse() * Q).diagonal();  // M the diagonal vector of an NxN matrix


            int jp, jm;
            double maximum = m.maxCoeff(&jp);
            double minimum = m.minCoeff(&jm);
            double eps_plus = ((maximum/n) - 1.0);
            double eps_minus = (1.0 - (minimum/n));

            double eps = std::max(eps_plus,eps_minus);
            bool test = ((m.array()>((1.0+eps)*n)).any()) || (((m.array()<((1.0-eps)*n))&&(u.array()>0.00001)).any()) ;
            /* ROS_INFO_STREAM("["<< ros::this_node::getName().c_str()<<"]: " << u.transpose()); */
            /* ROS_INFO_STREAM("["<< ros::this_node::getName().c_str()<<"]: " << m.transpose()); */
            /* ROS_INFO_STREAM("["<< ros::this_node::getName().c_str()<<"]: " << ((1.0+eps)*n)); */
            /* ROS_INFO_STREAM("["<< ros::this_node::getName().c_str()<<"]: " << (m.array()>((1.0+eps)*n))); */
            /*   ROS_INFO_STREAM("["<< ros::this_node::getName().c_str()<<"]: " << ((1.0-eps)*n)); */
            /*   ROS_INFO_STREAM("["<< ros::this_node::getName().c_str()<<"]: " << (m.array()<((1.0-eps)*n))); */
            if (!test){
              break;
            }

            double step_size;
            e::VectorXd new_u;
            if (eps_plus>=eps_minus){
              eps = eps_plus;
              step_size = (maximum - n)/(n*(maximum-1.0));
              new_u = (1.0 - step_size)*u ;
              new_u(jp) = new_u(jp) + step_size;
              /* ROS_INFO_STREAM("["<< ros::this_node::getName().c_str()<<"]: " << "+"); */
            }
            else{
              eps = eps_minus;
              step_size = std::min(((n - minimum)/(n*(minimum-1.0))),((u(jm))/(1.0-u(jm))));
              new_u = (1.0 + step_size)*u ;
              new_u(jm) = new_u(jm) - step_size;
              /* ROS_INFO_STREAM("["<< ros::this_node::getName().c_str()<<"]: " << "-"); */
            }
            count = count + 1;
            err = (new_u - u).norm();
            u = new_u;

          }
          /* ROS_INFO_STREAM("["<< ros::this_node::getName().c_str()<<"]: " << "We did "<< count << " iterations. err: " << err ); */
          //%%%%%%%%%%%%%%%%%% Computing the Ellipse parameters%%%%%%%%%%%%%%%%%%%%%%
          // Finds the ellipse equation in the 'center form': 
          // (x-c)' * A * (x-c) = 1
          // It computes a dxd matrix 'A' and a d dimensional vector 'c' as the center
          // of the ellipse. 
          e::MatrixXd U = u.asDiagonal();
          // the A matrix for the ellipse
          // --------------------------------------------
          /* e::Matrix3d A = (1.0/(double)(d)) * (P * U * P.transpose() - (P*u)*(P*u).transpose() ).inverse(); */
          e::Matrix3d A = (1.0/(double)(d)) * (P * U * P.transpose() - (P*u)*(P*u).transpose() ).inverse();
          /* e::Matrix3d A = (1.0/(double)(d)) * (P * U * P.transpose() - (P*u)*(P*u).transpose() ).inverse(); */


          //Now to convert it into covariance matrix
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: A:\n" << A); */
          e::JacobiSVD<e::MatrixXd> svd(A, e::ComputeThinU | e::ComputeThinV);
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: sgval of A:\n" << svd.singularValues().transpose()); */
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: sgvec of A:\n" << svd.matrixV()); */

          e::Vector3d sgvs = svd.singularValues();
          e::Matrix3d D = ((sgvs.array()<0.00000001).select(std::numeric_limits<double>::max(),sgvs)).cwiseInverse().asDiagonal();
          e::Matrix3d V = svd.matrixV();

            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Singular values: [\n" << svd.singularValues() << "\n]"); */
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Matrix V: [\n" << svd.matrixV() << "\n]"); */

          //
          /* e::EigenSolver<e::Matrix3d> es(A); */
          /* e::Matrix3d D = es.eigenvalues().real().cwiseInverse().asDiagonal(); */
          /* e::Matrix3d V = es.eigenvectors().real(); */

          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: D:\n" << D); */

          e::Matrix3d C = V*D*V.inverse();

          // center of the ellipse 
          // --------------------------------------------
          e::Vector3d c = P * u;

          return {c,C};
        }

        e::MatrixXd stdVecOfVectorsToMatrix(std::vector<e::Vector3d> V){
          e::MatrixXd M(3,V.size());
          int i = 0;
          for (e::Vector3d &v : V){
            M.block(0,i, 3,1) = v;
            i++;
          }
          return M;
        }


        std::pair<std::pair<e::Vector3d, e::Quaterniond>, e::MatrixXd> twoMeasurementUnion(std::pair<std::pair<e::Vector3d, e::Quaterniond>, e::MatrixXd> a, std::pair<std::pair<e::Vector3d, e::Quaterniond>, e::MatrixXd> b){
          e::MatrixXd U = e::MatrixXd::Identity(6,6);
          e::Vector3d up;
          e::Quaterniond uo;

          //position

          e::Matrix3d A = a.second.topLeftCorner(3,3);
          e::Matrix3d B = b.second.topLeftCorner(3,3);

          e::Vector3d c = b.first.first - a.first.first;
          /* e::Matrix3d c2 = c*c.transpose(); */


          double om = 0;
          double  value_prev = std::numeric_limits<double>::max();

          int pos_steps = 5;
          for (int p = 0; p<=pos_steps; p++){
            double om_tent = (double)(p)/(double)(pos_steps);

            auto Ut = getCandidateUnion(A,B,c,om_tent);


            double value_curr =Ut.determinant();
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: om: "<< om_tent << ", val: " << value_curr); */
            if (value_curr < value_prev){
              /* ROS_INFO_STREAM("[UVDARPoseCalculator]: acc"); */
              value_prev = value_curr;
              om = om_tent;
            }
          }

          U.topLeftCorner(3,3) = getCandidateUnion(A,B,c,om);
          up = a.first.first + om*c;

          //orientation

          A = a.second.bottomRightCorner(3,3);
          B = b.second.bottomRightCorner(3,3);

          e::Quaterniond c_q = b.first.second*a.first.second.inverse();


          /* double c_ang = e::AngleAxisd(c_q).angle(); */
          e::Matrix3d c_M = c_q.toRotationMatrix();
          e::Vector3d c_v = e::Vector3d( rotmatToRoll(c_M), rotmatToPitch(c_M), rotmatToYaw(c_M));
          /* e::Matrix3d c2_o = c_v*c_v.transpose(); */


          double om_o = 0;
          value_prev = std::numeric_limits<double>::max();

          int rot_steps = 5;
          for (int p = 0; p<=rot_steps; p++){
            double om_tent = (double)(p)/(double)(rot_steps);

            auto Ut = getCandidateUnion(A,B,c_v,om_tent,false);
            if (quaternionToRPY(c_q).norm() < 0.01){
              ROS_INFO_STREAM("[UVDARPoseCalculator]: c_v: " << c_v.transpose() << "\n c2: \n" << c_v*c_v.transpose());
              ROS_INFO_STREAM("[UVDARPoseCalculator]: U_t: " << Ut.eigenvalues().transpose());
            }


            double value_curr =Ut.determinant();
            if (value_curr < value_prev){
              value_prev = value_curr;
              om_o = om_tent;
            }
          }

          U.bottomRightCorner(3,3) = getCandidateUnion(A,B,c_v,om_o,false);
          e::AngleAxisd c_aa = e::AngleAxisd(c_q);
          uo = (e::AngleAxisd(c_aa.angle()*om_o,c_aa.axis()))*(a.first.second);

          return {{up, uo}, U};
        }

        e::Matrix3d getCandidateUnion(const e::Matrix3d &A, const e::Matrix3d &B,const e::Vector3d &c, double om, bool force_consistency=true){

          e::Matrix3d c2 = c*c.transpose();

          double beta1 = 1.0, beta2 = 1.0;
          if (force_consistency){
            //The beta term comes from S. Reece and S. Roberts: Generalized Covariance Union: A Unified Approach to Hypothesis Merging in Tracking
            e::LLT<e::Matrix3d,e::Upper> Sla(A);
            e::Matrix3d Sa = Sla.matrixU();
            e::Vector3d m1 = om*c;
            double d1 = (Sa.transpose().inverse()*m1).norm();
            double dc1 = ((1+d1)*(1+d1))/(1+(d1*d1));
            beta1 = (d1>1?2:dc1);

            e::LLT<e::Matrix3d,e::Upper> Slb(B);
            e::Matrix3d Sb = Slb.matrixU();
            e::Vector3d m2 = (1-om)*c;
            double d2 = (Sb.transpose().inverse()*m2).norm();
            double dc2 = ((1+d2)*(1+d2))/(1+(d2*d2));
            beta2 = (d2>1?2:dc2);
          }

          //The rest of the algorithm comes from O. Bochardt, R. Calhoun, J.K.Uhlmann and S.J.Julier: Generalized information representation and compression using covariance union
          e::Matrix3d U1     = beta1 * (A + (sqr(om)*c2));
          e::Matrix3d U2     = beta2 * (B + (sqr(1.0-om)*c2));

          e::LLT<e::Matrix3d,e::Upper> Sl(U2);
          e::Matrix3d S = Sl.matrixU();
          e::Matrix3d US = S.inverse().transpose()*U1*S.inverse();


          e::JacobiSVD<e::MatrixXd> svd(US, e::ComputeThinU | e::ComputeThinV);

          e::EigenSolver<e::Matrix3d> es(US);
          e::Matrix3d D = es.eigenvalues().real().asDiagonal();
          e::Matrix3d V = es.eigenvectors().real();
          e::Matrix3d U = S.transpose()*V*(D.cwiseMax(e::Matrix3d::Identity()))*V.transpose()*S;

          if (c.norm() < 0.01){
            ROS_INFO_STREAM("[UVDARPoseCalculator]: Singular values: [\n" << svd.singularValues() << "\n]");
            ROS_INFO_STREAM("[UVDARPoseCalculator]: Matrix V: [\n" << svd.matrixV() << "\n]");

            ROS_INFO_STREAM("[UVDARPoseCalculator]: U1:\n " << U1);
            ROS_INFO_STREAM("[UVDARPoseCalculator]: U2:\n " << U2);
            ROS_INFO_STREAM("[UVDARPoseCalculator]: S: " << S);
            ROS_INFO_STREAM("[UVDARPoseCalculator]: US: " << US);
            ROS_INFO_STREAM("[UVDARPoseCalculator]: V: " << V);
            ROS_INFO_STREAM("[UVDARPoseCalculator]: D: " << D);
            ROS_INFO_STREAM("[UVDARPoseCalculator]: U: " << U);
          }

          return U;
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
          /* e::Vector3d led_vector = marker_cam_pos.second*e::Vector3d(1,0,0); */
          e::Vector3d led_vector = fastMatrixVectorProduct(marker_cam_pos.second,e::Vector3d(1,0,0)); // can accelerate by just selecting the first row...
          double cos_view_angle = view_vector.dot(led_vector);
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: view_vector: " << view_vector.transpose() << " led_vector: " << led_vector.transpose() << " cos_view_angle: " << cos_view_angle); */
          return {output_position, cos_view_angle};
        }

        std::pair<e::Vector3d, e::Matrix3d> opticalFromMarker(LEDMarker marker){
          /* e::Quaterniond q_rotator(0.5,0.5,-0.5,0.5); */
          e::Matrix3d m_rotator;
          m_rotator << \
            0, -1, 0, \
            0, 0, -1, \
            1, 0, 0;
          //two expression of the same for speed optimization
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: marker_pos: " << marker.position << "; rotated: " << rotator*marker.position); */
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: marker_rot: " << marker.orientation.x() << ":"<<marker.orientation.y() << ":"<<marker.orientation.z() << ":"<<marker.orientation.w() << "; rotated: " << (rotator*marker.orientation).x() << ":"<<(rotator*marker.orientation).y() << ":"<<(rotator*marker.orientation).z() << ":"<<(rotator*marker.orientation).w()); */
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: marker_rot: " << quaternionToRPY(marker.orientation).transpose() << "; rotated: " << quaternionToRPY(rotator*marker.orientation).transpose()); */
          /* adfasdfadf */
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Matrix *: " << m_rotator*marker.orientation.toRotationMatrix() << "; Matrix f: "<<fastMatrixMatrixProduct(m_rotator,marker.orientation.toRotationMatrix())); */
          return {fastMatrixVectorProduct(m_rotator,marker.position), fastMatrixMatrixProduct(m_rotator,marker.orientation.toRotationMatrix())};
        }

        std::pair<std::pair<e::Vector3d,e::Quaterniond>,e::MatrixXd> opticalFromBase(std::pair<e::Vector3d,e::Quaterniond> pose, e::MatrixXd covariance){
          auto output_pose = pose;
          auto output_covariance = covariance;
          /* e::Quaterniond rotator(0.5,0.5,-0.5,0.5); */
          /* rotator = _center_fix_[image_index]*rotator; */
          output_pose.first = rot_base_to_optical*pose.first;
          output_pose.second = rot_base_to_optical*pose.second;
          output_covariance.topLeftCorner(3,3) = rot_base_to_optical.toRotationMatrix()*output_covariance.topLeftCorner(3,3)*rot_base_to_optical.toRotationMatrix().transpose();
          output_covariance.bottomRightCorner(3,3) = rot_base_to_optical.toRotationMatrix()*output_covariance.bottomRightCorner(3,3)*rot_base_to_optical.toRotationMatrix().transpose();
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
         * @brief Returns the index of target UAV with a marker based on the signal-based ID of that marker
         *
         * @param s_i Index of the signal of the given marker
         *
         * @return Index of the target carrying the marker
         */
        /* classifyMatch //{ */
        int classifyMatch(int s_i) {
          return s_i/signals_per_target_;
        }
        //}


        /**
         * @brief Returns true if the centroid of the image points is within <margin> pixels from the image edge
         *
         * @param points Vector of image points
         * @param margin The distance from the image edge to consider
         * @param image_index The index of the current camera used to generate the input points
         *
         * @return True if centroid is in the image margin, False otherwise
         */
        /* avgIsNearEdge //{ */
        bool avgIsNearEdge(std::vector< cv::Point3d > points, int margin, int image_index){


          cv::Point2d mean(0,0);
          for (auto &point : points){
            mean += cv::Point2d(point.x, point.y);
          }

          mean /= (int)(points.size());
          if (
              (mean.x < margin) ||
              (mean.y < margin) ||
              (mean.x > (camera_image_sizes_[image_index].width-margin)) ||
              (mean.y > (camera_image_sizes_[image_index].height-margin))
             ){
            return true;
          }


          return false;
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

        e::Vector3d quaternionToRPY(e::Quaterniond q){
          auto rotmat = q.toRotationMatrix();
          return e::Vector3d(rotmatToRoll(rotmat), rotmatToPitch(rotmat), rotmatToYaw(rotmat));
        }

        //}

        e::Vector3d fastMatrixVectorProduct(e::Matrix3d M, e::Vector3d v){//matrix - vector multiplication in this order
          e::Vector3d output;
          output.setZero();
          for (int i=0; i<3; i++){
            for (int j=0; j<3; j++){
              output.coeffRef(i) += M.coeff(i,j)*v.coeff(j); //coeff should disable range checking
            }
          }
          return output;
        }

        e::Matrix3d fastMatrixMatrixProduct(e::Matrix3d M1, e::Matrix3d M2){//matrix multiplication in this order
          e::Matrix3d output;
          output.setZero();
          for (int i=0; i<3; i++){//row 
            for (int j=0; j<3; j++){// column - for the first matrix
              for (int k=0; k<3; k++){//for iterating among element multiplication
                output.coeffRef(i,j) += M1.coeff(i,k)*M2.coeff(k,j); //coeff should disable range checking
              }
            }
          }
          return output;
        }

        template <typename T> int sgn(T val) {
          return (T(0) < val) - (val < T(0));
        }

        /* attributes //{ */
        bool _debug_;
        bool _profiling_;

        std::string _uav_name_;

        Profiler profiler;


        using blinkers_seen_callback_t = boost::function<void (const mrs_msgs::ImagePointsWithFloatStampedConstPtr& msg)>;
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
        bool _publish_constituents_;
        ros::Timer timer_visualization_;
        std::vector<cv::Size> camera_image_sizes_;
        cv::Mat image_visualization_;
        std::unique_ptr<mrs_lib::ImagePublisher> pub_visualization_;


        std::vector<struct ocam_model> _oc_models_;
        std::vector<e::Quaterniond> _center_fix_;


        /* Eigen::MatrixXd Px2,Px3,Px2q; */
        /* Eigen::MatrixXd Px2qb,Px3qb,Px4qb; */
        /* Eigen::VectorXd X2,X3,X2q; */
        /* Eigen::VectorXd X2qb,X3qb,X4qb; */

        std::vector<ros::Publisher> pub_measured_poses_;
        std::vector<ros::Publisher> pub_constituent_poses_;
        std::vector<ros::Publisher> pub_constituent_hypo_poses_;

        std::vector<int> _signal_ids_;
        int signals_per_target_;
        int _target_count_;
        /* std::unique_ptr<UVDARFrequencyClassifier> ufc_; */


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

        bool _separate_by_distance_;

        LEDModel model_;

        bool initialized_ = false;

        std::vector<std::vector<e::Vector3d>> axis_vectors_default;

        std::vector<std::vector<e::Vector3d>> axis_vectors_;

        std::vector<std::shared_ptr<std::mutex>>  mutex_separated_points_;
        std::vector<std::vector<std::pair<int,std::vector<cv::Point3d>>>> separated_points_;

        double led_projection_coefs_[3] = {1.3398, 31.4704, 0.0154}; //empirically measured coefficients of the decay of blob radius wrt. distance of a LED in our UVDAR cameras.


        std::mutex transformer_mutex;
        mrs_lib::Transformer transformer_;
        std::vector<std::optional<geometry_msgs::TransformStamped>> tf_fcu_to_cam;
        std::vector<e::Quaterniond> camera_view_;

        e::Quaterniond rot_base_to_optical = e::Quaterniond(0.5,0.5,-0.5,0.5);
        e::Quaterniond rot_optical_to_base = e::Quaterniond(0.5,-0.5,0.5,-0.5);
        //}
  };

} //uvdar

int main(int argc, char** argv) {
  ros::init(argc, argv, "uvdar_pose_calculator");

  ros::NodeHandle nh("~");
  uvdar::UVDARPoseCalculator upc(nh);
  ROS_INFO("[UVDARPoseCalculator]: UVDAR Pose calculator node initiated");
  ros::spin();
  return 0;
}

