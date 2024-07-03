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
#include <geometry_msgs/Pose.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/image_publisher.h>
#include <mrs_lib/timer.h>
#include <std_msgs/Float32.h>
#include <uvdar_core/ImagePointsWithFloatStamped.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "OCamCalib/ocam_functions.h"
#include <unscented/unscented.h>
/* #include <p3p/P3p.h> */
#include <color_selector/color_selector.h>
/* #include <frequency_classifier/frequency_classifier.h> */

#define MAX_DIST_INIT 100.0
#define BRACKET_STEP 10
#define QPIX 2 //pixel std. dev


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

/* #define ERROR_THRESHOLD_INITIAL (752.0/M_PI) */
/* #define ERROR_THRESHOLD_INITIAL sqr(10) */
#define ERROR_THRESHOLD_INITIAL(img) sqr(_oc_models_.at(img).width/50.0)
#define ERROR_THRESHOLD_MUTATION_1(img) sqr(_oc_models_.at(img).width/75.0)
#define ERROR_THRESHOLD_MUTATION_2(img) sqr(_oc_models_.at(img).width/100.0)
#define ERROR_THRESHOLD_MUTATION_3(img) sqr(_oc_models_.at(img).width/150.0)

#define PF_REPROJECT_THRESHOLD_VERIFIED(img) sqr(_oc_models_.at(img).width/150.0)
/* #define PF_REPROJECT_THRESHOLD_VERIFIED(img) sqr(2) */
#define PF_REPROJECT_THRESHOLD_UNFIT(img) sqr(_oc_models_.at(img).width/50.0)
/* #define PF_MUTATION_THRESHOLD(img) sqr(_oc_models_.at(img).width/150.0) */
#define SCATTER_TIME_STEP 0.1//s
#define MUTATION_COUNT 1
#define MUTATION_POSITION_MAX_STEP 5.0//m per second
#define MUTATION_ORIENTATION_MAX_STEP 3.14//rad per second
#define MUTATION_VELOCITY_MAX_STEP 1.0//m per second^2

#define MAX_INITIAL_VELOCITY 1.0//m per second

#define MAX_HYPOTHESIS_COUNT 1000

#define INITIAL_ROUGH_HYPOTHESIS_COUNT 200
#define INITIAL_HYPOTHESIS_COUNT 10

#define MAX_HYPOTHESIS_AGE 1.5

#define MAX_INIT_ITERATIONS 10000
#define MAX_MUTATION_REFINE_ITERATIONS 1000

#define SIMILAR_ERRORS_THRESHOLD sqr(1)

#define UVDAR_RANGE(img) (_oc_models_.at(img).width/50.0)
#define MAX_HYPOTHESIS_SPREAD 8.0

#define REJECT_UPSIDE_DOWN true

#define EDGE_DETECTION_MARGIN 10

#define SINGLE_HYPOTHESIS_COVARIANCE sqr(0.1)

#define PUBLISH_HYPO_CONSTITUENTS false

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
        std::scoped_lock lock(profiler_mutex);
        latest_times.push_back(getTime());
      }

      void addValue(std::string preamble){
        if (active){
          std::scoped_lock lock(profiler_mutex);
          auto now_time = getTime();
          elapsed_time.push_back({currDepthIndent() + preamble,std::chrono::duration_cast<std::chrono::microseconds>(now_time - latest_times.back()).count()});
          latest_times.back() = now_time;
        }
      }

      void addValueBetween(std::string preamble, std::chrono::time_point<std::chrono::high_resolution_clock> start, std::chrono::time_point<std::chrono::high_resolution_clock> stop){
        if (active){
          std::scoped_lock lock(profiler_mutex);
          elapsed_time.push_back({currDepthIndent() + preamble,std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()});
        }
      }

      void addValueSince(std::string preamble, std::chrono::time_point<std::chrono::high_resolution_clock> start){

        if (active){
          std::scoped_lock lock(profiler_mutex);
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
          std::scoped_lock lock(profiler_mutex);
          latest_times.push_back(getTime());
          curr_depth_indent++;
        }
      }

      void unindent(){
        if (active){
          std::scoped_lock lock(profiler_mutex);
          latest_times.pop_back();
          curr_depth_indent--;
        }
      }

      void printAll(std::string preamble){
        std::scoped_lock lock(profiler_mutex);
        for (auto data : elapsed_time){
          std::cout << preamble << " "  << data.first << " took : " << double(data.second)/1000.0 << " ms" << std::endl;
        }
      }

      void clear(){
        std::scoped_lock lock(profiler_mutex);
        elapsed_time.clear();
        latest_times.clear();
      }

      void stop(){
        std::scoped_lock lock(profiler_mutex);
        active = false;
      }

      void start(){
        std::scoped_lock lock(profiler_mutex);
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

      std::mutex profiler_mutex;
  };

  /**
   * @brief A processing class for converting retrieved blinking markers from a UV camera image into relative poses of observed UAV that carry these markers, as well as the error covariances of these estimates
   */
  class UVDARPoseCalculator {
    struct Pose {
      e::Vector3d position;
      e::Quaterniond orientation;
    };
    struct Twist {
      e::Vector3d linear;
      e::Quaterniond angular; //TODO or remove
    };

    struct LEDMarker {
      Pose pose;
      int type = -1; // 0 - directional, 1 - omni ring, 2 - full omni
      int signal_id = -1;
    };

    struct InputData {
      std::vector<uvdar_core::Point2DWithFloat> points;
      ros::Time time;
        /* geometry_msgs::TransformStamped tf; */
    };

    struct ImagePointIdentified{
      int ID;
      cv::Point2i position;
    };

    struct ImageCluster{
      int ID;
      std::vector<ImagePointIdentified> points;
    };

    enum HypothesisFlag { neutral, unfit, verified };
    class Hypothesis {
      public:
      int index;
      Pose pose;
      Twist twist;
      HypothesisFlag flag;
      ros::Time observed;
      ros::Time propagated;
      int unique_id;

      Hypothesis() {
        unique_id = rand(); //not perfectly unique, this is only for debugging
        /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Creating hypothesis with ID "<< unique_id << " at " << ros::Time::now()); */
      }

      void setPose(geometry_msgs::Pose inp){
        pose.position = e::Vector3d(
            inp.position.x,
            inp.position.y,
            inp.position.z
            );
        pose.orientation = e::Quaterniond(
            inp.orientation.w,
            inp.orientation.x,
            inp.orientation.y,
            inp.orientation.z
            );
        }

      geometry_msgs::Pose getPose(){
          geometry_msgs::Pose output;
          output.position.x = pose.position.x();
          output.position.y = pose.position.y();
          output.position.z = pose.position.z();
          
          output.orientation.w = pose.orientation.w();
          output.orientation.x = pose.orientation.x();
          output.orientation.y = pose.orientation.y();
          output.orientation.z = pose.orientation.z();

          return output;
        }

      std::string flagString(){
        switch (flag){
          case verified: return "verified";
          case neutral: return "neutral";
          default: return "unfit";
        }
      }

    };

    class AssociatedHypotheses {
      public:
        std::list<Hypothesis> hypotheses;
        int target;
        int verified_count = 0;

        bool debug;

        AssociatedHypotheses(const std::vector<Hypothesis> &hs, int target_i, bool debug_i){
          debug = debug_i;
          /* hypotheses = hs; */
          std::copy(hs.begin(), hs.end(), std::back_inserter(hypotheses));
          target = target_i;
          for (auto &h :hypotheses){
            if (h.flag == verified){
              verified_count++;
            }
          }
        }

        std::list<Hypothesis>::iterator at(int index){
          std::list<Hypothesis>::iterator it = hypotheses.begin();
          std::advance(it, index);
          return it;
        }

        std::vector<Hypothesis> getVerified(){
          std::vector<Hypothesis> output;
          for (auto &h : hypotheses){
            if (h.flag == verified){
              output.push_back(h);
            }
          }
          return output;

        }

        std::list<Hypothesis>::iterator removeHypothesis(std::list<Hypothesis>::iterator it){
          int hypothesis_count = (int)(hypotheses.size());
          Hypothesis hypo_value = (*it);
          if (debug)
                ROS_INFO_STREAM("[UVDARPoseCalculator]: Removing hypothesis: " << hypo_value.unique_id << ": " << hypo_value.pose.position.transpose() <<", status: " << hypo_value.flagString());
          if (hypothesis_count == 0){
            ROS_ERROR_STREAM("[UVDARPoseCalculator]: Cannot remove, no hypotheses available");
            return it;
          }

          if (hypo_value.flag == verified){
            verified_count --;
          }


          return hypotheses.erase(it);

        }

        void removeUnfit(){
          hypotheses.remove_if([](Hypothesis h){return h.flag == unfit;});
        }

        void addHypothesis(const Hypothesis &h){
          hypotheses.push_back(h);
          if (h.flag == verified){
            verified_count++;
          }
        }

        void addHypotheses(const std::vector<Hypothesis> &hs){
          for (auto &h :hs){
            if (h.flag == verified){
              verified_count++;
            }
          }

          hypotheses.insert(hypotheses.end(), hs.begin(), hs.end());
        }

        void setVerified(std::list<Hypothesis>::iterator it){
          if ((*it).flag != verified)
            verified_count++;
          (*it).flag = verified;
        }

        void setUnfit(std::list<Hypothesis>::iterator it){
          if ((*it).flag == verified)
            verified_count--;
          (*it).flag = unfit;
        }

        void setNeutral(std::list<Hypothesis>::iterator it){
          if ((*it).flag == verified)
            verified_count--;
          (*it).flag = neutral;
        }
    };

    class LEDModel {
      private:
      std::vector<LEDMarker> markers;
      std::vector<std::vector<int>> groups;

      public:
      LEDModel() = default;

      ~LEDModel(){
        markers.clear();
        groups.clear();
      }

      LEDModel(std::string model_file){
        if (!parseModelFile(model_file)){
          ros::shutdown();
          return;
        }

        prepareGroups();
      }

      LEDModel(const LEDModel &input){
        markers = input.markers;
        groups = input.groups;
      }

      LEDModel translate(e::Vector3d position) const {
        LEDModel output = *this;
        for (auto &marker : output.markers){
          marker.pose.position += position;
          /* output.push_back(marker); */
        }
        return output;
      }

      LEDModel translate(geometry_msgs::Point pi) const {
        LEDModel output = *this;
        e::Vector3d position(pi.x,pi.y,pi.z);
        for (auto &marker : output.markers){
          marker.pose.position += position;
        }
        return output;
      }

      std::vector<LEDMarker> getMarkers(){
        std::vector<LEDMarker> output = markers;
        return output;
      }

      LEDModel rotate(e::Vector3d center, e::AngleAxisd aa) const {
        LEDModel output = *this;
        for (auto &marker : output.markers){
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: pos a: " << marker.position); */
          marker.pose.position -= center;
          marker.pose.position = aa*marker.pose.position;
          marker.pose.position += center;
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: pos b: " << marker.position); */
          marker.pose.orientation = aa * marker.pose.orientation;
          /* output.push_back(marker); */
        }
        return output;
      }

      LEDModel rotate(e::AngleAxisd aa) const {
        LEDModel output = *this;
        for (auto &marker : output.markers){
          marker.pose.position = aa*marker.pose.position;
          marker.pose.orientation = aa * marker.pose.orientation;
        }
        return output;
      }

      LEDModel rotate(e::Quaterniond q) const {
        LEDModel output = *this;
        for (auto &marker : output.markers){
          marker.pose.position = q*marker.pose.position;
          marker.pose.orientation = q * marker.pose.orientation;
        }
        return output;
      }
      
      LEDModel rotate(geometry_msgs::Quaternion qi) const {
        e::Quaterniond q(qi.w,qi.x,qi.y,qi.z);
        return rotate(q);
      }


      LEDModel rotate(e::Vector3d center, e::Vector3d axis, double angle) const {
        e::Quaterniond rotation(e::AngleAxisd(angle, axis));
        return rotate(center, rotation);
      }

      LEDModel rotate(e::Vector3d center, e::Quaterniond orientation) const {
        LEDModel output = *this;
        for (auto &marker : output.markers){
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: pos a: " << marker.position); */
          marker.pose.position -= center;
          marker.pose.position = orientation*marker.pose.position;
          marker.pose.position += center;
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: pos b: " << marker.pose.position); */
          marker.pose.orientation = orientation * marker.pose.orientation;
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
              double tent_dist = (markers[groups[i][0]].pose.position - markers[groups[j][0]].pose.position).norm();
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
          double angle_between = a.pose.orientation.angularDistance(b.pose.orientation);
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

      LEDModel& operator=(const LEDModel &other){
          markers = other.markers;
          groups = other.groups;
          return *this;
      }

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
            if ((marker.pose.position - markers.at(group.at(0)).pose.position).norm() < LED_GROUP_DISTANCE){
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
            curr_lm.pose.position = e::Vector3d(X,Y,Z);
            int type;
            iss >> type;
            curr_lm.type = type;
            double pitch, yaw;
            iss >> pitch;
            iss >> yaw;
            tf::Quaternion qtemp;
            qtemp.setRPY(0, pitch, yaw);
            curr_lm.pose.orientation.x() = qtemp.x();
            curr_lm.pose.orientation.y() = qtemp.y();
            curr_lm.pose.orientation.z() = qtemp.z();
            curr_lm.pose.orientation.w() = qtemp.w();
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

    struct ReprojectionContext {
      int target;
      std::vector<ImagePointIdentified> observed_points;
      int image_index;
      geometry_msgs::TransformStamped tocam_tf;
      LEDModel model;
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
          profiler_main_.start();
          profiler_thread_.start();
        }
        else {
          profiler_main_.stop();
          profiler_thread_.stop();
        }

        param_loader.loadParam("gui", _gui_, bool(false));
        param_loader.loadParam("publish_visualization", _publish_visualization_, bool(false));

        param_loader.loadParam("publish_constituents", _publish_constituents_, bool(false));

        param_loader.loadParam("quadrotor",_quadrotor_,bool(false));

        /* param_loader.loadParam("beacon",_beacon_,bool(false)); */

        param_loader.loadParam("custom_model",_custom_model_,bool(false));
        param_loader.loadParam("model_file",_model_file_, std::string(""));
        
        param_loader.loadParam("separate_by_distance",_separate_by_distance_,bool(true));
        param_loader.loadParam("max_cluster_distance",_max_cluster_distance_,double(100));

        param_loader.loadParam("output_frame", _output_frame_, std::string("local_origin"));

        prepareModel();


        /* load the signals //{ */
        param_loader.loadParam("signal_ids", _signal_ids_);
        if (_signal_ids_.empty()){
          ROS_WARN("[UVDARPoseCalculator]: No signal IDs were supplied, using the default sequence set.");
          _signal_ids_ = {0, 1, 2, 3, 4, 5, 6, 7, 8};
        }

        //}



        /* Subscribe to blinking point topics and advertise poses//{ */
        std::vector<std::string> _blinkers_seen_topics;
        param_loader.loadParam("blinkers_seen_topics", _blinkers_seen_topics, _blinkers_seen_topics);
        if (_blinkers_seen_topics.empty()) {
          ROS_WARN("[UVDARPoseCalculator]: No topics of blinkers were supplied");
        }
        _camera_count_ = (unsigned int)(_blinkers_seen_topics.size());


        for (unsigned int i = 0; i < _camera_count_; i++){
          /* tocam_tf_.push_back(geometry_msgs::TransformStamped()); */
          /* fromcam_tf_.push_back(geometry_msgs::TransformStamped()); */
          input_data_initialized.push_back(false);
          InputData id;
          latest_input_data_.push_back(id);

          blinkers_seen_callback_t callback = [image_index=i,this] (const uvdar_core::ImagePointsWithFloatStampedConstPtr& pointsMessage) { 
            ProcessPoints(pointsMessage, image_index);
          };
          ROS_INFO_STREAM("[UVDARPoseCalculator]: Subscribing to " << _blinkers_seen_topics[i]);
          sub_blinkers_seen_.push_back(
              nh.subscribe(_blinkers_seen_topics[i], 1, callback));

          ROS_INFO_STREAM("[UVDARPoseCalculator]: Advertising measured poses " << i+1);

          if (_publish_constituents_){
            /* pub_constituent_poses_.push_back(nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("constituentPoses"+std::to_string(i+1), 1)); */ 
            /* if (PUBLISH_HYPO_CONSTITUENTS){ */
            /*   pub_constituent_hypo_poses_.push_back(nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("constituentHypoPoses"+std::to_string(i+1), 1)); */ 
            /* } */
              pub_hypotheses_ = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("constituentHypotheses", 1); 
              pub_hypotheses_tentative_ = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("constituentHypothesesTentative", 1); 
          }

          camera_image_sizes_.push_back(cv::Size(-1,-1));

          /* mutex_separated_points_.push_back(std::make_shared<std::mutex>()); */
        }
        //}
        
        pub_measured_poses_ = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("measuredPoses", 1); 

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

        /* tf_fcu_to_cam.resize(_blinkers_seen_topics.size(), std::nullopt); */
        /* camera_view_.resize(_blinkers_seen_topics.size(), e::Quaterniond()); */

        initModelsAndAxisVectors();


        /* timer_initializer_ = nh.createTimer(ros::Rate(1), &UVDARPoseCalculator::InitializationThreadStarter, this, false); //Thread that generates fresh, naive hypotheses from images at low rate */
        /* initializer_thread_ = std::make_unique<std::thread>(&UVDARPoseCalculator::InitializationThread,this); */
        ros::Rate ir(1);
        initializer_thread_ = std::make_unique<mrs_lib::ThreadTimer>(nh, ir, &UVDARPoseCalculator::InitializationThread, this, false, true);
        timer_particle_filter_ = nh.createTimer(ros::Rate(1.0/SCATTER_TIME_STEP), &UVDARPoseCalculator::ParticleScatteringThread, this, false); //Thread that filters and propagates hypotheses from prior estimates

        ros::Duration idt(0);
        particle_filtering_thread_ = std::make_unique<mrs_lib::ThreadTimer>(nh, idt, &UVDARPoseCalculator::InitializationThread, this, true, false); //the thread callback will change upon use.

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
            file_name = ros::package::getPath("uvdar_core")+"/config/ocamcalib/calib_results_bf_uv_fe.txt";
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
          auto center_dir = directionFromCamPoint(cv::Point2d(_oc_models_.at(i).width/2,_oc_models_.at(i).height/2),i);
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
            file_name = ros::package::getPath("uvdar_core")+"/config/models/quadrotor.txt";
          }
          else {
            file_name = ros::package::getPath("uvdar_core")+"/config/models/hexarotor.txt";
          }
        }
        else {
            file_name = _model_file_;
        }
        model_ = LEDModel(file_name);
        std::tie(maxdiameter_, mindiameter_) = model_.getMaxMinVisibleDiameter();

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
      void ProcessPoints(const uvdar_core::ImagePointsWithFloatStampedConstPtr& msg, size_t image_index) {
        if (!initialized_){
          ROS_ERROR_STREAM("[UVDARPoseCalculator]: Uninitialized! Ignoring image points.");
          return;
        }
        /* ROS_INFO("[UVDARPoseCalculator]:PP: A"); */
        {
          std::scoped_lock lock(input_mutex);
          latest_input_data_[image_index].time = msg->stamp;
          latest_input_data_[image_index].points = msg->points;
        }
        camera_image_sizes_[image_index].width = msg->image_width;
        camera_image_sizes_[image_index].height = msg->image_height;

        /* ROS_INFO("[UVDARPoseCalculator]:PP: B"); */
        /* { */
          /* std::scoped_lock lock(transformer_mutex); */
          /* auto tocam_tmp = transformer_.getTransform(_uav_name_+"/"+_output_frame_,_camera_frames_[image_index], msg->stamp - ros::Duration(0.1)); */
          /* if (!tocam_tmp){ */
          /*   ROS_ERROR_STREAM_THROTTLE(1.0,"[UVDARPoseCalculator]: Could not obtain transform from " << _uav_name_+"/"+_output_frame_ << " to " << _camera_frames_[image_index] << "!"); */
          /*   return; */
          /* } */
          /* tocam_tf_[image_index] = tocam_tmp.value(); */

          /* auto fromcam_tmp = transformer_.getTransform(_camera_frames_[image_index],_uav_name_+"/"+_output_frame_, msg->stamp - ros::Duration(0.1)); */
          /* if (!fromcam_tmp){ */
          /*   ROS_ERROR_STREAM_THROTTLE(1.0,"[UVDARPoseCalculator]: Could not obtain transform from " << _camera_frames_[image_index] << " to " << _uav_name_+"/"+_output_frame_ << "!"); */
          /*   return; */
          /* } */
          /* fromcam_tf_[image_index] = fromcam_tmp.value(); */
        /* } */

        /* e::Vector3d tv(0,0,1); */
        /* auto tt1 = transformer_.transform(tv, fromcam_tf_[image_index]); */
        /* auto tt2 = transform(tv, fromcam_tf_[image_index]); */
        /* ROS_INFO_STREAM("[UVDARPoseCalculator]: C: " << image_index << "; Transforation: " << std::endl <<  fromcam_tf_[image_index]); */
        /* if (tt1){ */
        /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: C: " << image_index << "; OG Transformed " << tv.transpose() << " to " << tt1.value().transpose()); */
        /* } */
        /* if (tt2){ */
        /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: C: " << image_index << "; NW Transformed " << tv.transpose() << " to " << tt2.value().transpose()); */
        /* } */

        input_data_initialized[image_index] = true;


        /* ROS_INFO_STREAM("[UVDARPoseCalculator]:PP: C, " << image_index); */

        {
          std::scoped_lock lock(threadconfig_mutex);
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]:PP: D, " << image_index); */
          particle_filtering_thread_->setCallback(boost::bind(&UVDARPoseCalculator::ParticleFilteringThread, this, _1, image_index));
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]:PP: E, " << image_index); */
          particle_filtering_thread_->start();
        }


      }
      //}

      /* void InitializationThreadStarter([[maybe_unused]] const ros::TimerEvent& te) { */
      /* } */

        /**
         * @brief Thread function for generating a large set of initial hypotheses from a single image observation
         *
         * @param te TimerEvent for the timer spinning this thread
         */
        /* InitializationThread() //{ */
        void InitializationThread([[maybe_unused]] const ros::TimerEvent& evt) {
          /* ROS_INFO("[UVDARPoseCalculator]:IN: A"); */
          if (!ros::ok())
            return;
          /* ROS_INFO("[UVDARPoseCalculator]:IN: B"); */
          /* ros::Rate r(1); */
          /* while (ros::ok()){ */
          /* ROS_INFO("[%s]: Here A", ros::this_node::getName().c_str()); */
          if (!initialized_){
            return;
          }
          /* ROS_INFO("[%s]: Here B", ros::this_node::getName().c_str()); */


        if (!batch_processsed_) {
          return;
        }
          /* ROS_INFO("[%s]: Here C", ros::this_node::getName().c_str()); */

          /* ROS_INFO("[UVDARPoseCalculator]:IN: C"); */
        batch_processsed_ = false;
        for (unsigned int image_index = 0; image_index < _camera_count_; image_index++){
          /* if (!input_data_initialized[image_index]){ */
          /*   ROS_INFO("[%s]: TF for camera %d not initialized, skipping hypothesis intializaiton.", ros::this_node::getName().c_str(), image_index); */
          /*   continue; */
          /* } */
          /* ROS_INFO("[%s]: Initializing from C:%d", ros::this_node::getName().c_str(), image_index); */

          InputData latest_local;
          geometry_msgs::TransformStamped fromcam_tf;
          geometry_msgs::TransformStamped tocam_tf;

          {
            std::scoped_lock lock(input_mutex);
            if (!input_data_initialized[image_index]){
              ROS_ERROR_STREAM_THROTTLE(1.0,"[UVDARPoseCalculator]: Input data for camera " << image_index << " not yet intializad!");
              continue;
            }
            latest_local = latest_input_data_[image_index];
          }
          {
            std::scoped_lock lock(transformer_mutex);
            auto fromcam_tmp = transformer_.getTransform(_camera_frames_[image_index],_uav_name_+"/"+_output_frame_, latest_local.time - ros::Duration(0.1));
            if (!fromcam_tmp){
              ROS_ERROR_STREAM_THROTTLE(1.0,"[UVDARPoseCalculator]: Could not obtain transform from " << _camera_frames_[image_index] << " to " <<  _uav_name_+"/"+_output_frame_ << "!");
              continue;
            }
            fromcam_tf = fromcam_tmp.value();

            auto tocam_tmp = transformer_.getTransform(_uav_name_+"/"+_output_frame_,_camera_frames_[image_index], latest_local.time - ros::Duration(0.1));
            if (!tocam_tmp){
              ROS_ERROR_STREAM_THROTTLE(1.0,"[UVDARPoseCalculator]: Could not obtain transform from " << _uav_name_+"/"+_output_frame_ << " to " << _camera_frames_[image_index] << "!");
              continue;
            }
            tocam_tf = tocam_tmp.value();
          }

          /* ROS_INFO("[UVDARPoseCalculator]:IN: D"); */
          /* batch_processsed_[image_index] = false; */

          /* int                        countSeen; */
          std::vector<uvdar_core::Point2DWithFloat> points;
          last_blink_time_ = latest_local.time;
          if (_debug_)
            ROS_INFO_STREAM("[UVDARPoseCalculator]: Received points: " << latest_local.points.size());

          if (latest_local.points.size() < 1) {
            continue;
          }

          if (estimated_framerate_.size() <= image_index || estimated_framerate_[image_index] < 0) {
            ROS_INFO_THROTTLE(1.0,"[UVDARPoseCalculator]: Framerate is not yet estimated. Waiting...");
            batch_processsed_ = true;
            continue;
          }

          /* if (!tf_fcu_to_cam[image_index]){ */
          /*   ROS_INFO_THROTTLE(1.0,"[UVDARPoseCalculator]: Camera TF not yet obtained. Attempting to retrieve it..."); */
          /*   { */
          /*     std::scoped_lock lock(transformer_mutex); */
          /*     tf_fcu_to_cam[image_index] = transformer_.getTransform(_uav_name_+"/fcu", _camera_frames_[image_index], latest_local.time); */
          /*   } */
          /*   if (!tf_fcu_to_cam[image_index]) { */
          /*     ROS_ERROR_STREAM_THROTTLE(1.0,"[UVDARPoseCalculator]: Could not obtain transform from " << _uav_name_+"/fcu" << " to " << _camera_frames_[image_index] << "!"); */
          /*     batch_processsed_ = true; */
          /*     return; */
          /*   } */
          /*   /1* else { *1/ */
          /*     /1* const bool prepared_rotations = prepareModelsAndOrientations(image_index); *1/ */
          /*     /1* if (!prepared_rotations) *1/ */
          /*     /1*   tf_fcu_to_cam[image_index] = std::nullopt; *1/ */
          /*   /1* } *1/ /1* } *1/ */
        /* } */

          /* ROS_INFO("[UVDARPoseCalculator]:IN: E"); */
          for (const auto& point : latest_local.points) {
            if (_use_masks_){
              if (_masks_[image_index].at<unsigned char>(cv::Point2i(point.x, point.y)) < 100){
                if (_debug_){
                  ROS_INFO_STREAM("[UVDARPoseCalculator]: Discarding point " << cv::Point2i(point.x, point.y) << " with f="  << point.value);
                }
                continue;
              }
            }

            if (point.value <= 200) {
              points.push_back(point);
            }
          }

          /* ROS_INFO("[UVDARPoseCalculator]:IN: F"); */
          /* std::scoped_lock lock(*(mutex_separated_points_[image_index])); */

          /* mrs_msgs::PoseWithCovarianceArrayStamped msg_measurement_array; */
          /* msg_measurement_array.header.frame_id = _camera_frames_[image_index]; */
          /* msg_measurement_array.header.stamp = last_blink_time_; */

          /* mrs_msgs::PoseWithCovarianceArrayStamped msg_constuents_array; */
          /* mrs_msgs::PoseWithCovarianceArrayStamped msg_constuents_hypo_array; */
          /* msg_constuents_array.header.frame_id = _camera_frames_[image_index]; */
          /* msg_constuents_array.header.stamp = last_blink_time_; */
          /* msg_constuents_hypo_array.header.frame_id = _camera_frames_[image_index]; */
          /* msg_constuents_hypo_array.header.stamp = last_blink_time_; */

          std::vector<ImageCluster> separated_points;
          if ((int)(points.size()) > 0) {
            separated_points = separateBySignals(points);

          /* ROS_INFO("[UVDARPoseCalculator]:IN: G"); */
            auto start_target_cycle = profiler_thread_.getTime();
            /* profiler_thread_.indent(); */
            for (int i = 0; i < ((int)(separated_points.size())); i++) {

              /* auto start_target_iteration = profiler_thread_.getTime(); */
              if (_debug_){
                ROS_INFO_STREAM("[UVDARPoseCalculator]: target [" << separated_points[i].ID << "]: ");
                ROS_INFO_STREAM("[UVDARPoseCalculator]: p: ");
                for (auto pt : separated_points[i].points)
                  ROS_INFO_STREAM("[UVDARPoseCalculator]:    " << pt.position << ", ID: " << pt.ID);
              }
              mrs_msgs::PoseWithCovarianceIdentified pose;
              /* std::vector<mrs_msgs::PoseWithCovarianceIdentified> constituents; */
              std::vector<Hypothesis> new_hypotheses;
              /* bool res = extractSingleRelative(separated_points[i].second, separated_points[i].first, image_index, pose, constituents, new_hypotheses); */
              /* ROS_INFO("[%s]: C:%d - Image clusters: %d", ros::this_node::getName().c_str(), image_index, (int)(separated_points.size())); */
              new_hypotheses = extractHypotheses(separated_points[i].points, separated_points[i].ID, image_index, fromcam_tf, tocam_tf, profiler_thread_, latest_local.time);

              //TODO: consider doing multiple for each hypothesis
              double velocity_max_step = MAX_INITIAL_VELOCITY;
              for (auto &h : new_hypotheses){
                double vd = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);//scaler form 0 to 1
                e::Vector3d initial_velocity = (e::Vector3d::Random().normalized())*vd*velocity_max_step;
                h.twist.linear = initial_velocity;
              }
              /* ROS_INFO("[%s]: C:%d - New hypothesis count: %d", ros::this_node::getName().c_str(), image_index, (int)(new_hypotheses.size())); */

              /* if (res){ */
              /*   msg_measurement_array.poses.push_back(pose); */

                /* if (_publish_constituents_){ */
                /*   for (auto &constituent : constituents){ */
                /*     msg_constuents_array.poses.push_back(constituent); */
                /*   } */
                /*   if (PUBLISH_HYPO_CONSTITUENTS){ */
                /*     for (auto &constituent : new_hypotheses){ */
                /*       msg_constuents_hypo_array.poses.push_back(constituent); */
                /*     } */
                /*   } */
                /* } */


                /* profiler.addValueSince("Target "+std::to_string(separated_points[i].first),start_target_iteration); */
                /* if (_profiling_){ */
                /* } */
              /* } */

              profiler_thread_.addValueSince("All targets",start_target_cycle);

              if (_profiling_){
                profiler_thread_.printAll("[UVDARPoseCalculator]: [cam:"+std::to_string(image_index)+"]:");
              }
              profiler_thread_.clear();

              /* HypothesesToGlobal(new_hypotheses, image_index,latest_local.time); */
              {
                std::scoped_lock lock(hypothesis_mutex);
                int index = 0;
                bool found = false;
                for (auto &hb : hypothesis_buffer_){
                  if (hb.target == (separated_points[i].ID%1000)){
                    found = true;
                    break;
                  }
                  index++;
                }

                if (!found){
                  /* hypothesis_buffer_.push_back({.hypotheses = new_hypotheses,.target = (separated_points[i].ID%1000),.verified_count = 0}); */
                  hypothesis_buffer_.push_back(AssociatedHypotheses(new_hypotheses, (separated_points[i].ID%1000), _debug_));
                }
                else{

                  /* removeExtraHypotheses(index, latest_local.time); */
                  /* if (_debug_) */
                  /*   ROS_INFO("[%s]: Culling. Curr hypothesis count: %d", ros::this_node::getName().c_str(), (int)(hypothesis_buffer_.at(index).hypotheses.size())); */
                  /* hypothesis_buffer_.at(index).hypotheses.insert(hypothesis_buffer_.at(index).hypotheses.end(), new_hypotheses.begin(), new_hypotheses.end()); */
                  hypothesis_buffer_.at(index).addHypotheses(new_hypotheses);
                  /* hypothesis_buffer_.at(index).verified_count+=(int)(new_hypotheses.size()); */
                  if (_debug_)
                    ROS_INFO("[%s]: Inserting new. Curr hypothesis count: %d", ros::this_node::getName().c_str(), (int)(hypothesis_buffer_.at(index).hypotheses.size()));
                }

              }
            }
          /* ROS_INFO("[UVDARPoseCalculator]:IN: H"); */
            /* profiler_thread_.unindent(); */

            /* if (_publish_constituents_){ */
            /*   pub_constituent_poses_[image_index].publish(msg_constuents_array); */
            /*   if (PUBLISH_HYPO_CONSTITUENTS){ */
            /*     pub_constituent_hypo_poses_[image_index].publish(msg_constuents_hypo_array); */
            /*   } */
            /* } */

          }

        }
        if (_debug_){
          ROS_INFO("[%s]: Hypothesis count: ", ros::this_node::getName().c_str());
          for (auto &hb : hypothesis_buffer_){
            ROS_INFO("[%s]:     target: %d: %d", ros::this_node::getName().c_str(), hb.target,(int)(hb.hypotheses.size()));
          }
        }
        batch_processsed_ = true;

        /* r.sleep(); */
        /* } */
          /* ROS_INFO("[UVDARPoseCalculator]:IN: I"); */
        }
        //}
        //

        /**
         * @brief Thread function for refining hypotheses based on new observations
         *
         * @param te TimerEvent for the timer spinning this thread
         */
        /* ParticleScatteringThread() //{ */
        void ParticleScatteringThread([[maybe_unused]] const ros::TimerEvent& te) {
          if ((!initialized_)){
            return;
          }
          /* ROS_INFO("[UVDARPoseCalculator]:PF: A"); */

          auto now_time = ros::Time::now();
          if (true){
            /* if (false){ */
            std::scoped_lock lock(hypothesis_mutex);
            for (int index = 0; index < (int)(hypothesis_buffer_.size()); index++){
              int verified_count = hypothesis_buffer_.at(index).verified_count;
              if (_debug_)
                ROS_INFO("[%s]: Prev. hypothesis count: %d, of which verified: %d", ros::this_node::getName().c_str(), (int)(hypothesis_buffer_.at(index).hypotheses.size()), verified_count);
              profiler_main_.indent();
              /* auto mutations = mutateHypotheses(hypothesis_buffer_.at(index).hypotheses, std::min(std::max(0,MAX_HYPOTHESIS_COUNT - (int)(hypothesis_buffer_.at(index).hypotheses.size())),verified_count), now_time); */
              auto mutations = mutateHypotheses(hypothesis_buffer_.at(index), hypothesis_buffer_.at(index).hypotheses.size()/2, now_time);
              /* auto mutations = mutateHypotheses(hypothesis_buffer_.at(index).hypotheses, MUTATION_COUNT); */
              if (_debug_)
                ROS_INFO("[%s]: Made: %d mutations", ros::this_node::getName().c_str(), (int)(mutations.size()));
              profiler_main_.unindent();
              hypothesis_buffer_.at(index).addHypotheses(mutations);
              if (_debug_)
                ROS_INFO("[%s]: Adding mutations to set. Curr hypothesis count: %d", ros::this_node::getName().c_str(), (int)(hypothesis_buffer_.at(index).hypotheses.size()));
            }

            /* ROS_INFO("[UVDARPoseCalculator]:PF: B"); */
            for (unsigned int image_index = 0; image_index < _camera_count_; image_index++){

              InputData latest_local;
              geometry_msgs::TransformStamped tocam_tf;
              {
                std::scoped_lock lock(input_mutex);
                if (!input_data_initialized[image_index]){
                  /* ROS_ERROR_STREAM("[UVDARPoseCalculator]:PF: Input data for camera " << image_index << " not yet intializad!"); */
                  continue;
                }
                latest_local = latest_input_data_[image_index];
              }
              auto separated_points = separateBySignals(latest_local.points);

              /* ROS_INFO("[UVDARPoseCalculator]:PF: C"); */
              {
                std::scoped_lock lock(transformer_mutex);
                auto tocam_tmp = transformer_.getTransform(_uav_name_+"/"+_output_frame_,_camera_frames_[image_index], latest_local.time - ros::Duration(0.1));
                if (!tocam_tmp){
                  /* ROS_ERROR_STREAM("[UVDARPoseCalculator]:PF: Could not obtain transform from " << _uav_name_+"/"+_output_frame_ << " to " << _camera_frames_[image_index] << "!"); */
                  continue;
                }
                tocam_tf = tocam_tmp.value();
              }
              /* ROS_INFO("[UVDARPoseCalculator]:PF: D"); */

              /* ROS_INFO("[UVDARPoseCalculator]:PF: E"); */


              /* ROS_INFO("[UVDARPoseCalculator]:PF: F"); */
            }
              auto start = profiler_main_.getTime();
              for (int index = 0; index < (int)(hypothesis_buffer_.size()); index++){
                /* removeOldHypotheses(index,now_time); */
                removeExtraHypotheses(index, now_time, profiler_main_);
                if (_debug_)
                  ROS_INFO("[%s]: Culling. Curr hypothesis count: %d", ros::this_node::getName().c_str(), (int)(hypothesis_buffer_.at(index).hypotheses.size()));
                /* propagateHypotheses(index, now_time); */
              }
              profiler_main_.addValueSince("Extra hypothesis removal", start);
          }


          {
              /* for (int index = 0; index < (int)(hypothesis_buffer_.size()); index++){ */
              /*   /1* removeOldHypotheses(index,now_time); *1/ */
              /*   removeExtraHypotheses(index, now_time); */
              /*   if (_debug_) */
              /*     ROS_INFO("[%s]: Culling. Curr hypothesis count: %d", ros::this_node::getName().c_str(), (int)(hypothesis_buffer_.at(index).hypotheses.size())); */
              /* } */
            std::scoped_lock lock(hypothesis_mutex);
            if (_publish_constituents_){
              auto start_pub_const = profiler_main_.getTime();

              mrs_msgs::PoseWithCovarianceArrayStamped msg_constuents_array;
              msg_constuents_array.header.frame_id = _uav_name_+"/"+_output_frame_;
              msg_constuents_array.header.stamp = latest_input_data_[0].time;

              mrs_msgs::PoseWithCovarianceArrayStamped msg_constuents_tentative_array;
              msg_constuents_tentative_array.header.frame_id = _uav_name_+"/"+_output_frame_;
              msg_constuents_tentative_array.header.stamp = latest_input_data_[0].time;

              for (auto &hb : hypothesis_buffer_){
                if (_debug_)
                  ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Current hypothesis count for target " << hb.target << " is " << hb.hypotheses.size());
                for (auto &h : hb.hypotheses){
                  /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Age: " << (now_time - h.updated).toSec()); */

                  mrs_msgs::PoseWithCovarianceIdentified constituent;
                  constituent.id = h.index;
                  constituent.pose.position.x = h.pose.position.x();
                  constituent.pose.position.y = h.pose.position.y();
                  constituent.pose.position.z = h.pose.position.z();
                  constituent.pose.orientation.x = h.pose.orientation.x();
                  constituent.pose.orientation.y = h.pose.orientation.y();
                  constituent.pose.orientation.z = h.pose.orientation.z();
                  constituent.pose.orientation.w = h.pose.orientation.w();

                  e::MatrixXd hypo_covar = e::MatrixXd::Identity(6,6)*0.01;
                  for (int i=0; i<6; i++){
                    for (int j=0; j<6; j++){
                      constituent.covariance[6*j+i] =  hypo_covar(j,i);
                    }
                  }
                  if (h.flag == verified)
                    msg_constuents_array.poses.push_back(constituent);
                  else if (h.flag == neutral)
                    msg_constuents_tentative_array.poses.push_back(constituent);
                }
              }

              pub_hypotheses_.publish(msg_constuents_array);
              pub_hypotheses_tentative_.publish(msg_constuents_tentative_array);
              profiler_main_.addValueSince("Publishing constituents", start_pub_const);
            }

            mrs_msgs::PoseWithCovarianceArrayStamped msg_output;
            msg_output.header.frame_id = _uav_name_+"/"+_output_frame_;
            msg_output.header.stamp = latest_input_data_[0].time;
            for (auto &hb : hypothesis_buffer_){
              mrs_msgs::PoseWithCovarianceIdentified msg_target;
              auto start_conv_hull = profiler_main_.getTime();
              auto res = getMeasurementElipsoidHull(hb);
              profiler_main_.addValueSince("Constructing convex hull", start_conv_hull);
              if (res){
                auto [m, C] = res.value();
                msg_target.id = hb.target;
                msg_target.pose.position.x = m.position.x();
                msg_target.pose.position.y = m.position.y();
                msg_target.pose.position.z = m.position.z();
                msg_target.pose.orientation.x = m.orientation.x();
                msg_target.pose.orientation.y = m.orientation.y();
                msg_target.pose.orientation.z = m.orientation.z();
                msg_target.pose.orientation.w = m.orientation.w();

                for (int i=0; i<6; i++){
                  for (int j=0; j<6; j++){
                    msg_target.covariance[6*j+i] =  C(j,i);
                  }
                }
                msg_output.poses.push_back(msg_target);
              }
            }
            pub_measured_poses_.publish(msg_output);
          }

          {
            std::scoped_lock lock(hypothesis_mutex);
            auto start_hypo_propagation = profiler_main_.getTime();
            for (int index = 0; index < (int)(hypothesis_buffer_.size()); index++){
              propagateHypotheses(index, now_time);
            }
              profiler_main_.addValueSince("Propagating hypotheses", start_hypo_propagation);
          }



          if (_profiling_){
            profiler_main_.printAll("[UVDARPoseCalculator]: [PF]:");
          }
          profiler_main_.clear();
        }
        //}
        

        void ParticleFilteringThread([[maybe_unused]] const ros::TimerEvent& te, const int& image_index) {
          if ((!initialized_)){
            return;
          }
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]:PF: A, " <<  image_index); */

          /* auto now_time = ros::Time::now(); */

          InputData latest_local;
          geometry_msgs::TransformStamped tocam_tf;
          {
            std::scoped_lock lock(input_mutex);
            if (!input_data_initialized[image_index]){
              /* ROS_ERROR_STREAM("[UVDARPoseCalculator]:PF: Input data for camera " << image_index << " not yet intializad!"); */
              return;
            }
            latest_local = latest_input_data_[image_index];
          }
          auto separated_points = separateBySignals(latest_local.points);

          /* ROS_INFO_STREAM("[UVDARPoseCalculator]:PF: C, " <<  image_index); */
          {
            std::scoped_lock lock(transformer_mutex);
            auto tocam_tmp = transformer_.getTransform(_uav_name_+"/"+_output_frame_,_camera_frames_[image_index], latest_local.time - ros::Duration(0.1));
            if (!tocam_tmp){
              /* ROS_ERROR_STREAM("[UVDARPoseCalculator]:PF: Could not obtain transform from " << _uav_name_+"/"+_output_frame_ << " to " << _camera_frames_[image_index] << "!"); */
              return;
            }
            tocam_tf = tocam_tmp.value();
          }
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]:PF: D, " <<  image_index); */

          for (int index = 0; index < (int)(hypothesis_buffer_.size()); index++){
            {
              std::scoped_lock lock(hypothesis_mutex);

              if (_debug_)
                ROS_INFO("[%s]: C:%d, I:%d Refining. Prev. hypothesis count: %d", ros::this_node::getName().c_str(), image_index, index, (int)(hypothesis_buffer_.at(index).hypotheses.size()));

              /* auto start = profiler.getTime(); */
              /* auto local_hypotheses = HypothesesToLocal(hypothesis_buffer_,image_index); */
              /* ROS_INFO("[%s]: C:%d Checking...", ros::this_node::getName().c_str(), image_index); */
              /* ROS_INFO("[%s]: C:%d Removing unfit and mutating. Prev. hypthesis count: %d", ros::this_node::getName().c_str(), image_index, (int)(hypothesis_buffer_.size())); */
              /* auto mutations = removeUnfitAndMutateHypotheses(hypothesis_buffer_.at(index).first); */

              /* ROS_INFO("[%s]: C:%d Finding unfit mutations. Prev. mutation count: %d", ros::this_node::getName().c_str(), image_index, (int)(mutations.size())); */
              /* double threshold_mutation = PF_MUTATION_THRESHOLD(image_index); */
              /* checkHypothesisFitness(mutations,image_index,threshold_mutation, latest_local, tocam_tf); */
              /* ROS_INFO("[%s]: C:%d Removing unfit mutations...", ros::this_node::getName().c_str(), image_index); */
              /* removeUnfitHypotheses(mutations); */


              /* profiler_main_.indent(); */
              /* refineHypotheses(separated_points, hypothesis_buffer_.at(index).first, hypothesis_buffer_.at(index).second, image_index, tocam_tf, profiler_main_); */
              /* profiler_main_.unindent(); */

              double threshold_reproject_unfit = PF_REPROJECT_THRESHOLD_UNFIT(image_index);
              double threshold_reproject_verified = PF_REPROJECT_THRESHOLD_VERIFIED(image_index);

              profiler_main_.indent();
              checkHypothesisFitness(hypothesis_buffer_.at(index), image_index, threshold_reproject_unfit, threshold_reproject_verified, separated_points, tocam_tf, latest_local.time);
              profiler_main_.addValue("Hypotheses fitness checking");
              removeUnfitHypotheses(hypothesis_buffer_.at(index));
              profiler_main_.addValue("Removal of unfit hypotheses");
              profiler_main_.unindent();
              if (_debug_)
                ROS_INFO("[%s]: C:%d, I:%d Curr. hypothesis count: %d", ros::this_node::getName().c_str(), image_index, index, (int)(hypothesis_buffer_.at(index).hypotheses.size()));
              /* ROS_INFO("[%s]: C:%d Removing extras, left: %d", ros::this_node::getName().c_str(), image_index, (int)(hypothesis_buffer_.at(index).hypotheses.size())); */
            }

            /* ROS_INFO("[UVDARPoseCalculator]:PF: E"); */


            /* ROS_INFO("[UVDARPoseCalculator]:PF: F"); */
          }

          /* ROS_INFO("[UVDARPoseCalculator]:PF: Z"); */
        }
        //}
       
        
        
        void HypothesesToGlobal(std::vector<Hypothesis> &input, int image_index, ros::Time time){
          std::optional<geometry_msgs::TransformStamped> tf;
          {
            std::scoped_lock lock(transformer_mutex);
            tf = transformer_.getTransform(_camera_frames_[image_index],_uav_name_+"/"+_output_frame_, time);
          }
          for (auto &h : input){
            std::optional<geometry_msgs::Pose> pose_transformed;

            h.pose = transform(h.pose, tf.value()).value();
          }
        }

        Hypothesis HypothesisToCamera(Hypothesis input, int image_index, ros::Time time){
          std::optional<geometry_msgs::TransformStamped> tf;
          {
            std::scoped_lock lock(transformer_mutex);
            tf = transformer_.getTransform(_uav_name_+"/"+_output_frame_,_camera_frames_[image_index], time);
          }
          std::optional<geometry_msgs::Pose> pose_transformed;
          Hypothesis output;
          output.pose = transform(input.pose, tf.value()).value();
          output.index = input.index;
          output.flag = input.flag;
          /* output.updated = time; */

          return output;
        }
        
        void checkHypothesisFitness(AssociatedHypotheses &hypotheses, int image_index, double threshold_unfit, double threshold_verified, std::vector<ImageCluster> points, geometry_msgs::TransformStamped tocam_tf, ros::Time time){
          /* InputData latest_local; */
          /* { */
          /*   std::scoped_lock lock(input_mutex); */
          /*   latest_local = latest_input_data_[image_index]; */
          /* } */
          /* std::vector<ImageCluster> separated_points; */
          /* separated_points = separateBySignals(input_data.points); */
          ReprojectionContext rpc;
          rpc.image_index=image_index;
          rpc.tocam_tf=tocam_tf;
          rpc.model=model_;

          std::vector<ImagePointIdentified> associated_points;
          /* int i=0; */
          for (auto &pts : points){
            if ((pts.ID%1000)==(hypotheses.target%1000)){
              associated_points = pts.points;
              for (auto hit = hypotheses.hypotheses.begin(); hit!=hypotheses.hypotheses.end(); hit++){
                if (_debug_)
                  ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Checking hypotheis " << (*hit).unique_id);
                if (isInView((*hit),image_index, tocam_tf)){
                  if (_debug_)
                    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Hypotheis " << (*hit).unique_id << " is in view of camera " << image_index);

                  /* auto model_curr = model_.rotate(h.pose.orientation).translate(h.pose.position); */

                  double threshold_scaled_unfit = (int)(associated_points.size())*threshold_unfit;
                  double threshold_scaled_verified = (int)(associated_points.size())*threshold_verified;

                  rpc.observed_points=associated_points;
                  rpc.target=(*hit).index;

                  double error_total = hypothesisError((*hit), rpc);
                  /* ROS_INFO("[%s]: error: %f vs threshold_scaled: %f", ros::this_node::getName().c_str(), error_total, threshold_scaled); */

                  if (error_total > threshold_scaled_unfit){
                    if (_debug_)
                      ROS_INFO_STREAM("[UVDARPoseCalculator]: setting hypo " << (*hit).unique_id << " as unfit, err:" << error_total << " vs " << threshold_scaled_unfit);
                    /* if (h.flag == verified) */
                    /*   hypotheses.verified_count--; */
                    /* h.flag = unfit; */
                    hypotheses.setUnfit(hit);
                    /* h.updated = time; */
                  }
                  else if (error_total < threshold_scaled_verified) {
                    if (_debug_)
                      ROS_INFO_STREAM("[UVDARPoseCalculator]: setting hypo " << (*hit).unique_id << " as verified, err:" << error_total << " vs " << threshold_scaled_verified);
                    /* if (h.flag != verified) */
                    /*   hypotheses.verified_count++; */
                    /* h.flag = verified; */
                    hypotheses.setVerified(hit);
                    (*hit).observed = time;
                  }
                  else {
                    if (_debug_)
                      ROS_INFO_STREAM("[UVDARPoseCalculator]: setting hypo " << (*hit).unique_id << " as neutral, err:" << error_total << " vs " << threshold_scaled_verified << " and " << threshold_scaled_unfit);
                    hypotheses.setNeutral(hit);
                  }
                  /* h.updated = time; */
                }
              }
            }
            else
              continue;
          }
        }

        std::vector<Hypothesis> removeUnfitAndMutateHypotheses(AssociatedHypotheses &hypotheses){
          removeUnfitHypotheses(hypotheses);
          return mutateHypotheses(hypotheses, MUTATION_COUNT, ros::Time::now());
        }

        void removeUnfitHypotheses(AssociatedHypotheses &hypotheses){
          hypotheses.removeUnfit();
        }

        void propagateHypotheses(int index, ros::Time time){
          for (auto &h : hypothesis_buffer_.at(index).hypotheses){
            h.pose.position += h.twist.linear*(time-h.propagated).toSec();
            h.propagated = time;
          }
        }

        void removeExtraHypotheses(int index, ros::Time time, Profiler &profiler){
          profiler.indent();
          auto start = profiler.getTime();
          removeOldHypotheses(index, time);
          profiler.addValueSince("Old hypothesis removal", start);

          std::vector<std::list<Hypothesis>::iterator> nonverified_hypotheses;
          for (auto hit = hypothesis_buffer_.at(index).hypotheses.begin(); hit!=hypothesis_buffer_.at(index).hypotheses.end(); hit++){
            if ((*hit).flag != verified){
              nonverified_hypotheses.push_back(hit);
            }
          }
          profiler.addValue("Search for unverified hypotheses");
          while (((int)(hypothesis_buffer_.at(index).hypotheses.size()) > MAX_HYPOTHESIS_COUNT) && (nonverified_hypotheses.size() > 0) ){ //first, let's try to remove the unverified only...
            int cull_index_selection = rand() % (int)(nonverified_hypotheses.size());
            auto hit = nonverified_hypotheses.at(cull_index_selection);
            if ((*hit).flag !=verified){
              if (_debug_)
                ROS_INFO_STREAM("[UVDARPoseCalculator]: Culling extra unverified hypothesis " << (*hit).unique_id << ".");
              hypothesis_buffer_.at(index).removeHypothesis(hit);
              nonverified_hypotheses.erase(nonverified_hypotheses.begin()+cull_index_selection);
            }
              /* hypothesis_buffer_.at(index).hypotheses.erase(hypothesis_buffer_.at(index).hypotheses.begin()+cull_index); //remove */
          }
          profiler.addValue("Unverified hypothesis removal");
          while ((int)(hypothesis_buffer_.at(index).hypotheses.size()) > MAX_HYPOTHESIS_COUNT){
            int cull_index = rand() % (int)(hypothesis_buffer_.at(index).hypotheses.size());
            /* if (hypothesis_buffer_.at(index).hypotheses[cull_index].flag == verified) */
              /* hypothesis_buffer_.at(index).verified_count--; */
            /* hypothesis_buffer_.at(index).hypotheses.erase(hypothesis_buffer_.at(index).hypotheses.begin()+cull_index); //remove */
            auto hit = hypothesis_buffer_.at(index).at(cull_index);
              if (_debug_)
                ROS_INFO_STREAM("[UVDARPoseCalculator]: Culling extra hypothesis " << (*hit).unique_id << ".");
            hypothesis_buffer_.at(index).removeHypothesis(hit);
          }
          profiler.addValue("Remaining hypothesis removal");
          profiler.unindent();
          return;
        }

        void removeOldHypotheses(int index, ros::Time time){
          for (auto hit = hypothesis_buffer_.at(index).hypotheses.begin(); hit!=hypothesis_buffer_.at(index).hypotheses.end(); hit++){
            if (ros::Duration(time - (*hit).observed).toSec() > MAX_HYPOTHESIS_AGE){
              /* if (hypothesis_buffer_.at(index).hypotheses[i].flag == verified) */
              /*   hypothesis_buffer_.at(index).verified_count--; */
              /* hypothesis_buffer_.at(index).hypotheses.erase(hypothesis_buffer_.at(index).hypotheses.begin()+i); //remove old */
              if (_debug_)
                ROS_INFO_STREAM("[UVDARPoseCalculator]: Hypothesis " << (*hit).unique_id << " is " << ros::Duration(time - (*hit).observed).toSec() << " old, compared to " << time << ".");
              hit = hypothesis_buffer_.at(index).removeHypothesis(hit);
            }
          }

        }


        std::vector<Hypothesis> mutateHypotheses(AssociatedHypotheses &hypotheses, int count, ros::Time time){
          if ((int)(hypotheses.hypotheses.size()) == 0){
            return {};
          }

          std::vector<Hypothesis> mutations;

          for (int i = 0; i < count; i++){
            int parent_index = rand() % (int)(hypotheses.hypotheses.size());

            auto selected = hypotheses.at(parent_index);

            if ((*selected).flag == verified){
              auto new_mutations = generateVelocityMutations(*selected, 10, time, neutral, 1.0);//verification from current image only checks if they fit in the current moment - the target may have been moving differently than the hypothesis
              mutations.insert(mutations.end(),new_mutations.begin(),new_mutations.end());

              new_mutations = generateMutations(*selected, 10, time, 1.0, 1.0);
              mutations.insert(mutations.end(),new_mutations.begin(),new_mutations.end());
            }
            else if ((*selected).flag == neutral){
              auto new_mutations = generateMutations(*selected, 1, time, 1.0, 1.0);//consider them as old as the parent if the parent wasn't verified
              mutations.insert(mutations.end(),new_mutations.begin(),new_mutations.end());
            }
            else{
              i--;
            }
          }

          return mutations;
        }

        bool isInView(Hypothesis h, int image_index, geometry_msgs::TransformStamped tocam_tf){

          auto curr_projected =  camPointFromGlobal(h.pose.position, image_index, tocam_tf);

          if (
              (curr_projected.x >= 0) &&
              (curr_projected.y >= 0) &&
              (curr_projected.x < camera_image_sizes_[image_index].width) &&
              (curr_projected.y < camera_image_sizes_[image_index].height)
             ){
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: C: " << image_index << " - In view: "<< curr_projected); */
            return true;
          }
          else
            return false;
        }

        std::vector<Hypothesis> generateMutations(Hypothesis hin, int count, [[maybe_unused]] ros::Time time, double position_max_step = -1.0, double angle_max_step = -1.0){
          std::vector<Hypothesis> output;
          if (position_max_step < 0)
            position_max_step = MUTATION_POSITION_MAX_STEP*SCATTER_TIME_STEP;//m
          /* if (velocity_max_step < 0) */
          /*   velocity_max_step = MUTATION_VELOCITY_MAX_STEP*SCATTER_TIME_STEP;//m */
          if (angle_max_step < 0)
            angle_max_step = MUTATION_ORIENTATION_MAX_STEP*SCATTER_TIME_STEP;//rad

          Hypothesis current_mutation;
          current_mutation.index=hin.index;
          current_mutation.flag=neutral;
          current_mutation.observed=hin.observed;
          current_mutation.propagated=hin.propagated;

          for (int i = 0; i < count; i++) {
            double d = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);//scaler form 0 to 1
            e::Vector3d offset_position = (e::Vector3d::Random().normalized())*d*position_max_step;
            current_mutation.pose.position = hin.pose.position+offset_position;

            /* double vd = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);//scaler form 0 to 1 */
            /* e::Vector3d offset_velocity = (e::Vector3d::Random().normalized())*vd*velocity_max_step; */
            /* current_mutation.twist.linear = hin.twist.linear+offset_velocity; */
            /* if ((hin.flag == verified) && (time > hin.propagated)) */
            /*   current_mutation.twist.linear += */
            /*     0.5 * //dampening */
            /*     ((current_mutation.pose.position-hin.pose.position)/(time-hin.propagated).toSec()); // if our original hypothesis was verified, this mutation should have been the result of an incorrect velocity estimate */

            current_mutation.twist.linear = hin.twist.linear;

            double a = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);//scaler form 0 to 1
            auto offset_rotation = e::AngleAxisd(a*angle_max_step, e::Vector3d::Random().normalized());
            current_mutation.pose.orientation = offset_rotation*hin.pose.orientation;

            output.push_back(current_mutation);
          }
          /* for (double x_sgn = -1; x_sgn < 1.5; x_sgn=x_sgn+1.0){ */
          /*   for (double y_sgn = -1; y_sgn < 1.5; y_sgn=y_sgn+1.0){ */
          /*     for (double z_sgn = -1; z_sgn < 1.5; z_sgn=z_sgn+1.0){ */
          /*       e::Vector3d shift_vector(x_sgn,y_sgn,z_sgn); */
          /*       shift_vector.normalize(); */

          /*       for (auto v : axis_vectors_global_){ */
          /*         for (double a_sgn = -1; a_sgn < 1.5; a_sgn=a_sgn+1.0){ */

          /*           if ((abs(x_sgn)<0.001) && (abs(y_sgn)<0.001) && (abs(z_sgn)<0.001) && (abs(a_sgn)<0.001) ) */
          /*             continue; */


          /*           current_mutation.pose.position = hin.pose.position+(position_max_step*shift_vector); */
          /*           current_mutation.pose.orientation = e::AngleAxisd(a_sgn*angle_step,v)*hin.pose.orientation; */

          /*           output.push_back(current_mutation); */
          /*         } */
          /*       } */

          /* }}} */

          return output;
        }

        std::vector<Hypothesis> generateVelocityMutations(Hypothesis hin, int count,[[maybe_unused]] ros::Time time, HypothesisFlag flag = neutral, double velocity_max_step = -1.0){
          std::vector<Hypothesis> output;
          if (velocity_max_step < 0)
            velocity_max_step = MUTATION_VELOCITY_MAX_STEP*SCATTER_TIME_STEP;//m

          Hypothesis current_mutation;
          current_mutation.index=hin.index;
          current_mutation.flag=flag;
          current_mutation.observed=hin.observed;
          current_mutation.propagated=hin.propagated;

          current_mutation.pose = hin.pose;

          for (int i = 0; i < count; i++) {


            double vd = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);//scaler form 0 to 1
            e::Vector3d offset_velocity = (e::Vector3d::Random().normalized())*vd*velocity_max_step;
            current_mutation.twist.linear = hin.twist.linear+offset_velocity;
            /* if ((hin.flag == verified) && (time > hin.propagated)) */
            /*   current_mutation.twist.linear += */
            /*     0.5 * //dampening */
            /*     ((current_mutation.pose.position-hin.pose.position)/(time-hin.propagated).toSec()); // if our original hypothesis was verified, this mutation should have been the result of an incorrect velocity estimate */

            output.push_back(current_mutation);
          }
          return output;
        }

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
       * @brief Separate set of input points into groups presumed to belong each to an individual UAV. Here, the separation is done primarily based on blinking sequences. Each UAV has its own set of sequences. If multiple distant clusters of sequences associated with the same UAV are found, they are returned with IDs increased by 1000 for each subsequent cluster
       *
       * @param points A set of points, where the X and Y coordinates correspond to their image positions and Z corresponds to their blinking frequencies
       *
       * @return A set of separated points sets, each accompanied by a unique integer identifier. The idientifier is equal to TID + 1000*CL where TID is the id of the UAV associated with frequencies of the markers and CL is the order number of the current cluster starting with 0. Ordinarilly, there should be only one cluster per target.
       */
      /* separateBySignals //{ */
      std::vector<ImageCluster> separateBySignals(std::vector<uvdar_core::Point2DWithFloat> points){
        std::vector<ImageCluster> separated_points;
        /* separated_points.resize(_target_count_); */


        for (int i = 0; i < (int)(points.size()); i++) {
          /* if (points[i].z > 1) { */
          if (points[i].value >= 0) {
            /* int mid = ufc_->findMatch(points[i].z); */
            int mid = points[i].value;
            int tid = classifyMatch(mid);
            if (_debug_)
              ROS_INFO("[%s]: SIG: %d, MID: %d, TID: %d", ros::this_node::getName().c_str(),(int)points[i].value, mid, tid);
            if (tid>=0){
              int index = -1;
              int j=0;
              for (auto& sep_pts : separated_points){
                if (sep_pts.ID == tid){
                  index = j;
                  break;
                }
                j++;
              }
              if (index < 0){
                separated_points.push_back({
                    .ID = tid,
                    .points = {{
                      .ID = (int)(points[i].value),
                      .position = cv::Point2i((int)(points[i].x), (int)(points[i].y))
                    }}
                    });
              }
              else {
                separated_points[index].points.push_back({
                      .ID = (int)(points[i].value),
                      .position = cv::Point2i((int)(points[i].x), (int)(points[i].y))
                    });
              }

            }
          }
        }


        auto fullAverage = [] (std::vector<ImagePointIdentified> points) { 
          cv::Point2d sum(0,0);
          for (auto p : points){
            sum += cv::Point2d(p.position);
          }
          sum /= (double)(points.size());
          return sum;
        };

        if (_separate_by_distance_){
          for (int s = 0; s < (int)(separated_points.size()); s++){
            std::vector<ImageCluster> clusters; // subsets of the frequency-separated split apart by distance
            std::vector<cv::Point2d> cluster_centroids;
            for (int i=0; i<(int)(separated_points[s].points.size());i++){
              bool cluster_found = false;
              for (int j=0; j<(int)(clusters.size());j++){
                if ( cv::norm((cv::Point2d(separated_points[s].points[i].position.x,separated_points[s].points[i].position.y) - cluster_centroids[j])) < _max_cluster_distance_){
                  clusters[j].points.push_back(separated_points[s].points[i]);
                  cluster_centroids.push_back(fullAverage(clusters[j].points));
                  cluster_found = true;
                }
              }
              if (!cluster_found){
                ImageCluster new_cluster;
                new_cluster.points.push_back(separated_points[s].points[i]);
                clusters.push_back(new_cluster);
                cv::Point2d init_cluster_pt_2d(separated_points[s].points[i].position.x, separated_points[s].points[i].position.y);
                cluster_centroids.push_back(init_cluster_pt_2d);
              }
            }
            if (clusters.size() > 1){
              int tid_orig = separated_points[s].ID;
              separated_points.erase(separated_points.begin()+s);

              auto it = separated_points.begin()+s;
              int cl_n = 0;
              for (auto& cluster : clusters){
                separated_points.insert(it+cl_n,1,{.ID=cl_n*1000+tid_orig,.points = cluster.points});
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
       * @brief Calculates a pose with error covariance of a UAV observed as a set of its blinking markers. This heavily exploits the approach for accounting for input errors and ambiguity ranges using the unscented transform shown in [V Walter, M Vrba and M Saska. "On training datasets for machine learning-based visual relative localization of micro-scale UAVs" (ICRA 2020). 2020].
       *
       * @param points The set of observed blinking markers in the image space of the current camera
       * @param target The index of the current target UAV
       * @param image_index The index of the current camera observing the UAV
       * @param output_pose The output estimated pose with covariance, encapsulated in a ros message. Also includes the target index
       */
      /* extractSingleRelative //{ */
      /* bool extractSingleRelative(std::vector<ImagePointIdentified> points, int target, size_t image_index, mrs_msgs::PoseWithCovarianceIdentified& output_pose, std::vector<mrs_msgs::PoseWithCovarianceIdentified> &constituents, std::vector<mrs_msgs::PoseWithCovarianceIdentified> &constituents_hypo) { */

      /*   Pose final_mean = */
      /*   { */
      /*     e::Vector3d( std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()), */
      /*     e::Quaterniond(1,0,0,0) */
      /*   }; */
      /*   e::MatrixXd final_covariance; */

      /*   if (_debug_){ */
      /*     ROS_INFO_STREAM("[UVDARPoseCalculator]: framerateEstim: " << estimated_framerate_[image_index]); */
      /*   } */

      /*   auto start = profiler.getTime(); */

      /*   double alpha_max; */
      /*   std::vector<e::Vector3d> v_w; */
      /*   if (points.size() != 1){ */
      /*     for (auto& point: points){ */
      /*       v_w.push_back(directionFromCamPoint(point.position, image_index)); */
      /*     } */
      /*     alpha_max = getLargestAngle(v_w); */
      /*   } */
          
      /*   if ((points.size() == 1) || (alpha_max < 0.01) || (avgIsNearEdge(points,EDGE_DETECTION_MARGIN, image_index))){ */
      /*     auto v_w_s = baseFromOptical(directionFromCamPoint(points.at(0).position, image_index)); */
      /*     v_w_s *= 15.0; //max range */
      /*     final_mean.position << v_w_s.x(),v_w_s.y(),v_w_s.z(); */
      /*     final_mean.orientation = e::Quaterniond(1,0,0,0); */
      /*     final_covariance.setIdentity(6,6); */
      /*     final_covariance *= 1000;//large covariance for angles in radians */
      /*     final_covariance.topLeftCorner(3, 3) = getLongCovariance(v_w_s,(maxdiameter_*1.0),1000.0); */
      /*   } */
      /*   else { */
      /*     auto furthest_position = getRoughInit(model_, v_w, image_index); */
      /*     if (_debug_){ */
      /*       ROS_INFO_STREAM("[UVDARPoseCalculator]: Furthest possible distance: " << furthest_position.norm()); */
      /*     } */
      /*     /1* auto rough_init = std::chrono::high_resolution_clock::now(); *1/ */
      /*     /1* elapsedTime.push_back({currDepthIndent() + ,std::chrono::duration_cast<std::chrono::microseconds>(rough_init - start).count()}); *1/ */
      /*     profiler.addValueSince("Rough initialization", start); */

      /*     auto [hypotheses, errors] = getViableInitialHyptheses(model_, points, furthest_position, target, image_index); */

      /*     int initial_hypothesis_count = (int)(hypotheses.size()); */

      /*     /1* auto fitted_position = iterFitPosition(model_, points, rough_initialization, target,  image_index); *1/ */
      /*     /1* if (_debug_){ *1/ */
      /*     /1*   ROS_INFO_STREAM("[UVDARPoseCalculator]: Fitted position: " << fitted_position.transpose()); *1/ */
      /*     /1* } *1/ */

      /*     if (_debug_) */
      /*       ROS_INFO_STREAM("[UVDARPoseCalculator]: Rough hypotheses for target " << target << " in image " << image_index << ": "); */
      /*     int i = 0; */
      /*     if (_debug_) */
      /*       for (auto h: hypotheses){ */
      /*         ROS_INFO_STREAM("x: [" << h.pose.position.transpose() << "] rot: [" << rad2deg(camera_view_[image_index].inverse()*quaternionToRPY(h.pose.orientation)).transpose() << "] with error of " << errors.at(i++)); */
      /*       } */


      /*     /1* auto viable_hypotheses = std::chrono::high_resolution_clock::now(); *1/ */
      /*     /1* elapsedTime.push_back({currDepthIndent() + "Viable initial hypotheses",std::chrono::duration_cast<std::chrono::microseconds>(viable_hypotheses - rough_init).count()}); *1/ */
      /*     profiler.addValue("Viable initial hypotheses"); */

      /*     std::vector<Hypothesis> selected_poses; */
      /*     std::vector<double> projection_errors; */
      /*     /1* projection_errors.push_back(std::numeric_limits<double>::max()); *1/ */
      /*     double smallest_error_in_set = std::numeric_limits<double>::max(); */
      /*     for (auto h: hypotheses){ */
      /*       /1* profiler.stop(); *1/ */
      /*       profiler.indent(); */

      /*       if (_publish_constituents_){ */
      /*         if (PUBLISH_HYPO_CONSTITUENTS){ */
      /*           auto hypothesis_optical = opticalFromBase(h.pose); */
      /*           mrs_msgs::PoseWithCovarianceIdentified constituent; */

      /*           constituent.id = target; */
      /*           constituent.pose.position.x = hypothesis_optical.position.x(); */
      /*           constituent.pose.position.y = hypothesis_optical.position.y(); */
      /*           constituent.pose.position.z = hypothesis_optical.position.z(); */
      /*           constituent.pose.orientation.x = hypothesis_optical.orientation.x(); */
      /*           constituent.pose.orientation.y = hypothesis_optical.orientation.y(); */
      /*           constituent.pose.orientation.z = hypothesis_optical.orientation.z(); */
      /*           constituent.pose.orientation.w = hypothesis_optical.orientation.w(); */

      /*           e::MatrixXd hypo_covar = e::MatrixXd::Identity(6,6)*0.01; */
      /*           for (int i=0; i<6; i++){ */
      /*             for (int j=0; j<6; j++){ */
      /*               constituent.covariance[6*j+i] =  hypo_covar(j,i); */
      /*             } */
      /*           } */
      /*           constituents_hypo.push_back(constituent); */
      /*         } */
      /*       } */

      /*       auto [fitted_pose, error] = iterFitFull(model_, points, h, target,  image_index); */

      /*       /1* double error = 1.0; *1/ */
      /*       /1* auto fitted_pose = h; *1/ */


      /*       profiler.unindent(); */

      /*       bool found_equivalent = false; */
      /*       for (auto &prev_fitted : selected_poses){ */ 
      /*         if ((fitted_pose.pose.position - prev_fitted.pose.position).norm() < 0.1){ */

      /*           auto q_diff = fitted_pose.pose.orientation*prev_fitted.pose.orientation.inverse(); */
      /*           double ang_diff = e::AngleAxisd(q_diff).angle(); */
      /*           if (ang_diff < 0.2){ */
      /*             if (_debug_){ */
      /*               ROS_INFO_STREAM("[UVDARPoseCalculator]: Discarding duplicate hypothesis"); */
      /*             } */
      /*             found_equivalent = true;// we don't need duplicate initial hypoteheses */
      /*             break; */

      /*           } */
      /*         } */
      /*       } */
      /*       if (found_equivalent){ */
      /*         continue; */
      /*       } */



      /*       if (error < 0){ //discarded fitting; */
      /*         continue; */
      /*       } */
      /*       if (_debug_){ */
      /*         ROS_INFO_STREAM("[UVDARPoseCalculator]: Fitted pose: [" << fitted_pose.pose.position.transpose() << "] rot: [" << rad2deg(quaternionToRPY(camera_view_[image_index].inverse()*fitted_pose.pose.orientation)).transpose() << "] with error of " << error); */
      /*       } */


      /*       if (fitted_pose.pose.position.norm() < 0.5) { //We don't expect to be able to measure relative poses of UAVs this close - the markers would be too bright and too far apart */
      /*         ROS_INFO_STREAM("[UVDARPoseCalculator]: Hypothesis too close: " << fitted_pose.pose.position.transpose() << " is " << fitted_pose.pose.position.norm() << "m from camera."); */
      /*         continue; */
      /*       } */

      /*       e::Vector3d unit_vec; */
      /*       unit_vec << 1.0,0,0; */
      /*       if (acos(fitted_pose.pose.position.normalized().dot(unit_vec)) > rad2deg(95)) //our lenses only allow us to see UAVs ~92.5 degrees away from the optical axis of the camera */
      /*         continue; */


      /*       /1* if (projection_errors.back() > (error+(SIMILAR_ERRORS_THRESHOLD*(int)(points.size())))){ *1/ */
      /*       projection_errors.push_back(error); */
      /*       selected_poses.push_back(fitted_pose); */

      /*       if ((error) < (smallest_error_in_set)){ */
      /*         smallest_error_in_set = error; */
      /*       } */
      /*       /1* else if (projection_errors.back() > (error-(SIMILAR_ERRORS_THRESHOLD*(int)(points.size())))){ *1/ */
      /*     } */
      /*     int fitted_hypothesis_count = (int)(selected_poses.size()); */


      /*     auto projection_errors_backup = projection_errors; */

      /*     double threshold = ERROR_THRESHOLD_FITTED(image_index)*(int)(points.size()); */

      /*     for (int i = 0; i<(int)(selected_poses.size()); i++){ */


      /*       if (projection_errors[i] > threshold){ // if the frequencies are the same, they tend to merge. Otherwise, the result varies */
      /*         if (_debug_){ */
      /*           ROS_ERROR_STREAM("[UVDARPoseCalculator]: error: " << projection_errors[i]); */
      /*         } */
      /*         selected_poses.erase(selected_poses.begin()+i); //remove the other */
      /*         projection_errors.erase(projection_errors.begin()+i); */
      /*         i--; */ 
      /*       } */
      /*     } */

      /*     /1* auto precise_fitting = std::chrono::high_resolution_clock::now(); *1/ */
      /*     /1* elapsedTime.push_back({currDepthIndent() + "Precise fitting",std::chrono::duration_cast<std::chrono::microseconds>(precise_fitting - viable_hypotheses).count()}); *1/ */
      /*     profiler.addValue("Precise fitting"); */

      /*     if ((int)(selected_poses.size()) == 0){ */
      /*       ROS_ERROR_STREAM("[UVDARPoseCalculator]: No suitable hypothesis found!"); */
      /*       ROS_ERROR_STREAM("[UVDARPoseCalculator]: Initial hypothesis count: "<< initial_hypothesis_count << ", fitted hypothesis count: " << fitted_hypothesis_count); */
      /*       /1* ROS_ERROR_STREAM("[UVDARPoseCalculator]: Points:"); *1/ */
      /*       /1* for (auto pt : points){ *1/ */
      /*       /1* ROS_ERROR_STREAM("[UVDARPoseCalculator]: " << pt); *1/ */
      /*       /1* } *1/ */
      /*       /1* ROS_ERROR_STREAM("[UVDARPoseCalculator]: Projection errors were: " ); *1/ */
      /*       /1* for (auto pre : projection_errors_backup){ *1/ */
      /*       /1* ROS_ERROR_STREAM("[UVDARPoseCalculator]: " << pre); *1/ */
      /*       /1* } *1/ */
      /*       return {}; */
      /*     } */


      /*     if (_publish_constituents_){ */
      /*       std::vector<e::MatrixXd> covariances; */
      /*       for ([[ maybe_unused ]] auto &p : selected_poses){ */
      /*         profiler.indent(); */
      /*         /1* auto [new_pose, covariance] = getCovarianceEstimate(model_, points, p, target,  image_index); *1/ */

      /*         e::MatrixXd covariance(6,6); */
      /*         covariance.setIdentity(); */
      /*         double pos_cov = 0.002; */
      /*         double rot_cov = 0.05; */
      /*         covariance(0,0) = pos_cov; */
      /*         covariance(1,1) = pos_cov; */
      /*         covariance(2,2) = pos_cov; */

      /*         covariance(3,3) = rot_cov; */
      /*         covariance(4,4) = rot_cov; */
      /*         covariance(5,5) = rot_cov; */

      /*         covariances.push_back(covariance); */
      /*         /1* p=new_pose; *1/ */
      /*         profiler.unindent(); */
      /*         /1* if (_debug_){ *1/ */
      /*         /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: Covariance: [\n" << covariances.back() << "\n]"); *1/ */
      /*         /1* } *1/ */
      /*       } */

      /*       /1* auto covariance_estimation = std::chrono::high_resolution_clock::now(); *1/ */
      /*       /1* elapsedTime.push_back({currDepthIndent() + "Covariance estimation",std::chrono::duration_cast<std::chrono::microseconds>(covariance_estimation - precise_fitting).count()}); *1/ */
      /*       /1* profiler.addValue("Covariance estimation"); *1/ */

      /*       /1* for (int i = 0; i<(int)(selected_poses.size()); i++){ *1/ */
      /*       /1*   /2* if (_debug_){ *2/ *1/ */
      /*       /1*   /2*   ROS_INFO_STREAM("[UVDARPoseCalculator]: Covariance base: [\n" << covariances[i] << "\n]"); *2/ *1/ */
      /*       /1*   /2* } *2/ *1/ */
      /*       /1*   auto constituent_pose_optical = opticalFromBase(selected_poses[i]); *1/ */
      /*       /1*   /2* if (_debug_){ *2/ *1/ */
      /*       /1*   /2*   ROS_INFO_STREAM("[UVDARPoseCalculator]: Covariance opt:: [\n" << constituent_pose_optical.second << "\n]"); *2/ *1/ */
      /*       /1*   /2* } *2/ *1/ */
      /*       /1*   mrs_msgs::PoseWithCovarianceIdentified constituent; *1/ */

      /*       /1*   constituent.id = target; *1/ */
      /*       /1*   constituent.pose.position.x = constituent_pose_optical.first.first.x(); *1/ */
      /*       /1*   constituent.pose.position.y = constituent_pose_optical.first.first.y(); *1/ */
      /*       /1*   constituent.pose.position.z = constituent_pose_optical.first.first.z(); *1/ */
      /*       /1*   constituent.pose.orientation.x = constituent_pose_optical.first.second.x(); *1/ */
      /*       /1*   constituent.pose.orientation.y = constituent_pose_optical.first.second.y(); *1/ */
      /*       /1*   constituent.pose.orientation.z = constituent_pose_optical.first.second.z(); *1/ */
      /*       /1*   constituent.pose.orientation.w = constituent_pose_optical.first.second.w(); *1/ */
      /*       /1*   for (int i=0; i<constituent_pose_optical.second.cols(); i++){ *1/ */
      /*       /1*     for (int j=0; j<constituent_pose_optical.second.rows(); j++){ *1/ */
      /*       /1*       constituent.covariance[constituent_pose_optical.second.cols()*j+i] =  constituent_pose_optical.second(j,i); *1/ */
      /*       /1*     } *1/ */
      /*       /1*   } *1/ */
      /*       /1*   constituents.push_back(constituent); *1/ */
      /*       /1* } *1/ */
      /*     } */



      /*     /1* if ((int)(selected_poses.size()) == 1){ *1/ */
      /*     /1*   final_mean = selected_poses.at(0).pose; *1/ */
      /*     /1*   final_covariance = e::Matrix6d::Identity()*SINGLE_HYPOTHESIS_COVARIANCE; *1/ */
      /*     /1* } *1/ */
      /*     /1* else if ((int)(selected_poses.size()) > 1){ *1/ */
      /*       /1* if (((selected_poses.front().first - selected_poses.back().first).norm() < MAX_HYPOTHESIS_SPREAD) || (selected_poses.size() < 100)) *1/ */
      /*       /1* { if (_debug_) *1/ */
      /*       /1*   ROS_INFO_STREAM("[UVDARPoseCalculator]: " << selected_poses.size() << " equivalent hypotheses found! I will attempt to smear them together into a unified measurement."); *1/ */
      /*         /1* std::tie(final_mean, final_covariance) = getMeasurementElipsoidHull(selected_poses); *1/ */
      /*         /1* final_covariance += e::Matrix6d::Identity()*SINGLE_HYPOTHESIS_COVARIANCE; *1/ */
      /*       /1* } *1/ */
      /*       /1* else{ *1/ */
      /*       /1*   auto v_w_s = baseFromOptical(directionFromCamPoint(points.at(0), image_index)); *1/ */
      /*       /1*   v_w_s *= 15.0; //max range *1/ */
      /*       /1*   final_mean.first << v_w_s.x(),v_w_s.y(),v_w_s.z(); *1/ */
      /*       /1*   final_mean.second = e::Quaterniond(1,0,0,0); *1/ */
      /*       /1*   final_covariance.setIdentity(6,6); *1/ */
      /*       /1*   final_covariance *= M_PI;//large covariance for angles in radians *1/ */
      /*       /1*   final_covariance.topLeftCorner(3, 3) = getLongCovariance(v_w_s,(model_.getMaxMinVisibleDiameter().first*1.0),1000.0); *1/ */

      /*       /1* } *1/ */
      /*     /1* } *1/ */


      /*     /1* auto measurement_union = std::chrono::high_resolution_clock::now(); *1/ */
      /*     /1* elapsedTime.push_back({currDepthIndent() + "Measurement union",std::chrono::duration_cast<std::chrono::microseconds>(measurement_union - covariance_estimation).count()}); *1/ */
      /*     /1* profiler.addValue("Measurement union"); *1/ */


      /*     /1* covariance += e::MatrixXd::Identity(6,6)*0.0001; *1/ */


      /*     /1* tf::Quaternion qtemp; *1/ */
      /*     /1* qtemp.setRPY((ms.x(3)), (ms.x(4)), (ms.x(5))); *1/ */
      /*     /1* qtemp=tf::Quaternion(-0.5,0.5,-0.5,-0.5)*qtemp; //bring relative orientations to the optical frame of the camera (Roll, Pitch and Yaw were estimated in the more intuitive sensor frame (X forward, Y to the left, Z up) *1/ */
      /*     /1* qtemp.normalize();//just in case *1/ */
      /*     } */

      /*     /1* auto fitted_pose_optical = opticalFromBase(final_mean,final_covariance); *1/ */

      /*     /1* output_pose.id = target; *1/ */
      /*     /1* output_pose.pose.position.x = fitted_pose_optical.first.first.x(); *1/ */
      /*     /1* output_pose.pose.position.y = fitted_pose_optical.first.first.y(); *1/ */
      /*     /1* output_pose.pose.position.z = fitted_pose_optical.first.first.z(); *1/ */
      /*     /1* output_pose.pose.orientation.x = fitted_pose_optical.first.second.x(); *1/ */
      /*     /1* output_pose.pose.orientation.y = fitted_pose_optical.first.second.y(); *1/ */
      /*     /1* output_pose.pose.orientation.z = fitted_pose_optical.first.second.z(); *1/ */
      /*     /1* output_pose.pose.orientation.w = fitted_pose_optical.first.second.w(); *1/ */
      /*     /1* for (int i=0; i<fitted_pose_optical.second.cols(); i++){ *1/ */
      /*     /1*   for (int j=0; j<fitted_pose_optical.second.rows(); j++){ *1/ */
      /*     /1*     output_pose.covariance[fitted_pose_optical.second.cols()*j+i] =  fitted_pose_optical.second(j,i); *1/ */
      /*     /1*   } *1/ */
      /*     /1* } *1/ */

      /*     /1* if (_debug_){ *1/ */
      /*     /1*   ROS_INFO_STREAM("[UVDARPoseCalculator]: Y: \n" << fitted_pose_optical.first.first.transpose() << " : [" << fitted_pose_optical.first.second.x() << "," << fitted_pose_optical.first.second.y() << "," <<fitted_pose_optical.first.second.z() << "," <<fitted_pose_optical.first.second.w() << "]" ); *1/ */
      /*     /1*   ROS_INFO_STREAM("[UVDARPoseCalculator]: Py: \n" << fitted_pose_optical.second ); *1/ */
      /*     /1* } *1/ */

      /*     return true; */
      /* } */
      //}
      //
      std::vector<Hypothesis> extractHypotheses(std::vector<ImagePointIdentified> points, int target, size_t image_index, geometry_msgs::TransformStamped fromcam_tf, geometry_msgs::TransformStamped tocam_tf, Profiler &profiler, ros::Time time) {

        auto start = profiler.getTime();

        double alpha_max;
        std::vector<e::Vector3d> v_w;
        e::Vector3d furthest_position;

        if (points.size() != 1){
          for (auto& point: points){
            v_w.push_back(directionFromCamPoint(point.position, image_index));
          }
          alpha_max = getLargestAngle(v_w);
        }
        if ((points.size() == 1) || (alpha_max < 0.01) || (avgIsNearEdge(points,EDGE_DETECTION_MARGIN, image_index))){
          /* auto v_w_s = baseFromOptical(directionFromCamPoint(points.front().position, image_index)); */
          auto v_w_s = directionFromCamPoint(points.front().position,image_index);
          v_w_s *= UVDAR_RANGE(image_index); //max range
          furthest_position = v_w_s;
        }
        else {
          furthest_position = getRoughInit(model_, v_w, image_index);
        }

          if (_debug_){
            ROS_INFO_STREAM("[UVDARPoseCalculator]: C: " << image_index << ", Furthest point : " << furthest_position.transpose());
            ROS_INFO_STREAM("[UVDARPoseCalculator]: Furthest possible distance: " << furthest_position.norm());
          }
          /* auto rough_init = std::chrono::high_resolution_clock::now(); */
          /* elapsedTime.push_back({currDepthIndent() + ,std::chrono::duration_cast<std::chrono::microseconds>(rough_init - start).count()}); */
          profiler.addValueSince("Rough initialization", start);

          auto [hypotheses_init, errors_init] = getViableInitialHyptheses(points, furthest_position, target, image_index, fromcam_tf, tocam_tf, INITIAL_ROUGH_HYPOTHESIS_COUNT, time);
          profiler.addValue("InitialHypotheses");
          double ratio_found = (double)(hypotheses_init.size()) / (double)(INITIAL_ROUGH_HYPOTHESIS_COUNT);
          //ROS_INFO(" Ratio Init: %f", 100*ratio_found );

          int desired_count = (int)(ratio_found*INITIAL_HYPOTHESIS_COUNT);
          auto hypotheses_refined = refineByMutation(model_, points, hypotheses_init,     tocam_tf, image_index, target, ERROR_THRESHOLD_MUTATION_1(image_index), 1.0, 1.0, desired_count);
          /* profiler.addValue("Initial Mutation 1"); */
          ratio_found = (double)(hypotheses_refined.size()) / desired_count;
          //ROS_INFO(" Ratio Refin 1: %f", 100*ratio_found );
          desired_count = (int)(ratio_found*desired_count);
          hypotheses_refined      = refineByMutation(model_, points, hypotheses_refined,  tocam_tf, image_index, target, ERROR_THRESHOLD_MUTATION_2(image_index), 1.0, 1.0, desired_count);
          /* profiler.addValue("Initial Mutation 2"); */
          ratio_found = (double)(hypotheses_refined.size()) / desired_count;
          //ROS_INFO(" Ratio Refin 2: %f", 100*ratio_found );
          desired_count = (int)(ratio_found*desired_count);
          hypotheses_refined      = refineByMutation(model_, points, hypotheses_refined,  tocam_tf, image_index, target, ERROR_THRESHOLD_MUTATION_3(image_index), 1.0, 1.0, desired_count);
          /* profiler.addValue("Initial Mutation 3"); */

          int static_hypothesis_count = (int)(hypotheses_refined.size());
          for (int i=0; i<static_hypothesis_count; i++){
            auto new_mutations = generateVelocityMutations(hypotheses_refined.at(i), 5, time, neutral, 1.0);
            hypotheses_refined.insert(hypotheses_refined.end(),new_mutations.begin(),new_mutations.end());
          }
          profiler.addValue("Initial Velocity Mutation");

          /* for (auto &h : hypotheses_refined){ */
          /*   h.flag = verified; */
          /* } */


          /* auto hypotheses_refined = hypotheses_init; */

          /* int initial_hypothesis_count = (int)(hypotheses.size()); */

          /* std::vector<Hypothesis> hypotheses_fitted; */
          /* int i = 0; */
          /* for (auto &h : hypotheses_init){ */
          /*   double error_t; */
          /*   Hypothesis h_t; */
          /*   /1* std::tie(h_t, error_t) = iterFitFull(model_, points, h, target,  image_index, tocam_tf, profiler); *1/ */

          /*   /1* if (_debug_) *1/ */
          /*   /1*   ROS_INFO_STREAM("[UVDARPoseCalculator]: Fitted position: Pre. error: " << errors_init[i] << ", New error: " << error_t); *1/ */
          /*   /1* const double threshold = (double)(points.size())*ERROR_THRESHOLD_FITTED(image_index); *1/ */
          /*   /1* if (error_t <= threshold){ *1/ */
          /*     hypotheses_fitted.push_back(h); */
          /*     if (_debug_) */
          /*       ROS_INFO_STREAM("[UVDARPoseCalculator]: Adding"); */
          /*   /1* } *1/ */
          /*   /1* else { *1/ */
          /*   /1*   if (_debug_) *1/ */
          /*   /1*     ROS_INFO_STREAM("[UVDARPoseCalculator]: Rejecting!"); *1/ */
          /*   /1* } *1/ */

          /*   i++; */
          /* } */
          /* if (_debug_){ */
          /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: Fitted position: " << fitted_position.transpose()); */
          /* } */

          /* if (_debug_){ */
          /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: Rough hypotheses for target " << target << " in image " << image_index << ": "); */
          /*   int i = 0; */
          /*   for (auto h: hypotheses){ */
          /*     ROS_INFO_STREAM("x: [" << h.pose.position.transpose() << "] rot: [" << rad2deg(quaternionToRPY(h.pose.orientation)).transpose() << "] with error of " << errors.at(i++)); */
          /*   } */
          /* } */


          /* auto viable_hypotheses = std::chrono::high_resolution_clock::now(); */
          /* elapsedTime.push_back({currDepthIndent() + "Viable initial hypotheses",std::chrono::duration_cast<std::chrono::microseconds>(viable_hypotheses - rough_init).count()}); */
          profiler.addValue("Viable initial hypotheses");

          return hypotheses_refined;
        
      }

          /* std::vector<Hypothesis> selected_poses; */
          /* std::vector<double> projection_errors; */
          /* /1* projection_errors.push_back(std::numeric_limits<double>::max()); *1/ */
          /* double smallest_error_in_set = std::numeric_limits<double>::max(); */
          /* for (auto h: hypotheses){ */
          /*   /1* profiler.stop(); *1/ */
          /*   profiler.indent(); */


          /*   auto [fitted_pose, error] = iterFitFull(model_, points, h, target,  image_index); */

          /*   /1* double error = 1.0; *1/ */
          /*   /1* auto fitted_pose = h; *1/ */


          /*   profiler.unindent(); */

          /*   bool found_equivalent = false; */
          /*   for (auto &prev_fitted : selected_poses){ */ 
          /*     if ((fitted_pose.pose.position - prev_fitted.pose.position).norm() < 0.1){ */

          /*       auto q_diff = fitted_pose.pose.orientation*prev_fitted.pose.orientation.inverse(); */
          /*       double ang_diff = e::AngleAxisd(q_diff).angle(); */
          /*       if (ang_diff < 0.2){ */
          /*         if (_debug_){ */
          /*           ROS_INFO_STREAM("[UVDARPoseCalculator]: Discarding duplicate hypothesis"); */
          /*         } */
          /*         found_equivalent = true;// we don't need duplicate initial hypoteheses */
          /*         break; */

          /*       } */
          /*     } */
          /*   } */
          /*   if (found_equivalent){ */
          /*     continue; */
          /*   } */



          /*   if (error < 0){ //discarded fitting; */
          /*     continue; */
          /*   } */
          /*   if (_debug_){ */
          /*     ROS_INFO_STREAM("[UVDARPoseCalculator]: Fitted pose: [" << fitted_pose.pose.position.transpose() << "] rot: [" << rad2deg(quaternionToRPY(fitted_pose.pose.orientation)).transpose() << "] with error of " << error); */
          /*   } */


          /*   if (fitted_pose.pose.position.norm() < 0.5) { //We don't expect to be able to measure relative poses of UAVs this close - the markers would be too bright and too far apart */
          /*     ROS_INFO_STREAM("[UVDARPoseCalculator]: Hypothesis too close: " << fitted_pose.pose.position.transpose() << " is " << fitted_pose.pose.position.norm() << "m from camera."); */
          /*     continue; */
          /*   } */

          /*   e::Vector3d unit_vec; */
          /*   unit_vec << 1.0,0,0; */
          /*   if (acos(fitted_pose.pose.position.normalized().dot(unit_vec)) > rad2deg(95)) //our lenses only allow us to see UAVs ~92.5 degrees away from the optical axis of the camera */
          /*     continue; */


          /*   /1* if (projection_errors.back() > (error+(SIMILAR_ERRORS_THRESHOLD*(int)(points.size())))){ *1/ */
          /*   projection_errors.push_back(error); */
          /*   selected_poses.push_back(fitted_pose); */

          /*   if ((error) < (smallest_error_in_set)){ */
          /*     smallest_error_in_set = error; */
          /*   } */
          /*   /1* else if (projection_errors.back() > (error-(SIMILAR_ERRORS_THRESHOLD*(int)(points.size())))){ *1/ */
          /* } */
          /* int fitted_hypothesis_count = (int)(selected_poses.size()); */


          /* auto projection_errors_backup = projection_errors; */

          /* double threshold = ERROR_THRESHOLD_FITTED(image_index)*(int)(points.size()); */

          /* for (int i = 0; i<(int)(selected_poses.size()); i++){ */


          /*   if (projection_errors[i] > threshold){ // if the frequencies are the same, they tend to merge. Otherwise, the result varies */
          /*     ROS_INFO("[%s]: error: %f vs threshold_scaled: %f", ros::this_node::getName().c_str(), projection_errors[i], threshold); */
          /*     if (_debug_){ */
          /*       ROS_ERROR_STREAM("[UVDARPoseCalculator]: error: " << projection_errors[i]); */
          /*     } */
          /*     selected_poses.erase(selected_poses.begin()+i); //remove the other */
          /*     projection_errors.erase(projection_errors.begin()+i); */
          /*     i--; */ 
          /*   } */
          /* } */

          /* /1* auto precise_fitting = std::chrono::high_resolution_clock::now(); *1/ */
          /* /1* elapsedTime.push_back({currDepthIndent() + "Precise fitting",std::chrono::duration_cast<std::chrono::microseconds>(precise_fitting - viable_hypotheses).count()}); *1/ */
          /* profiler.addValue("Precise fitting"); */

          /* if ((int)(selected_poses.size()) == 0){ */
          /*   ROS_ERROR_STREAM("[UVDARPoseCalculator]: No suitable hypothesis found!"); */
          /*   ROS_ERROR_STREAM("[UVDARPoseCalculator]: Initial hypothesis count: "<< initial_hypothesis_count << ", fitted hypothesis count: " << fitted_hypothesis_count); */
          /*   /1* ROS_ERROR_STREAM("[UVDARPoseCalculator]: Points:"); *1/ */
          /*   /1* for (auto pt : points){ *1/ */
          /*   /1* ROS_ERROR_STREAM("[UVDARPoseCalculator]: " << pt); *1/ */
          /*   /1* } *1/ */
          /*   /1* ROS_ERROR_STREAM("[UVDARPoseCalculator]: Projection errors were: " ); *1/ */
          /*   /1* for (auto pre : projection_errors_backup){ *1/ */
          /*   /1* ROS_ERROR_STREAM("[UVDARPoseCalculator]: " << pre); *1/ */
          /*   /1* } *1/ */
          /*   return {}; */
          /* } */





          /* if ((int)(selected_poses.size()) == 1){ */
          /*   final_mean = selected_poses.at(0); */
          /*   final_covariance = e::Matrix6d::Identity()*SINGLE_HYPOTHESIS_COVARIANCE; */
          /* } */
          /* else if ((int)(selected_poses.size()) > 1){ */
          /*   /1* if (((selected_poses.front().first - selected_poses.back().first).norm() < MAX_HYPOTHESIS_SPREAD) || (selected_poses.size() < 100)) *1/ */
          /*   { if (_debug_) */
          /*     ROS_INFO_STREAM("[UVDARPoseCalculator]: " << selected_poses.size() << " equivalent hypotheses found! I will attempt to smear them together into a unified measurement."); */
          /*     /1* std::tie(final_mean, final_covariance) = getMeasurementUnion(selected_poses, covariances); *1/ */
          /*     /1* std::tie(final_mean, final_covariance) = getMeasurementUnionSimple(selected_poses); *1/ */
          /*     std::tie(final_mean, final_covariance) = getMeasurementElipsoidHull(selected_poses); */
          /*     final_covariance += e::Matrix6d::Identity()*SINGLE_HYPOTHESIS_COVARIANCE; */
          /*   } */
          /* } */


          /* profiler.addValue("Measurement union"); */


            /* return selected_poses; */
          /* } */

          /* auto fitted_pose_optical = opticalFromBase(final_mean,final_covariance); */

          /* output_pose.id = target; */
          /* output_pose.pose.position.x = fitted_pose_optical.first.first.x(); */
          /* output_pose.pose.position.y = fitted_pose_optical.first.first.y(); */
          /* output_pose.pose.position.z = fitted_pose_optical.first.first.z(); */
          /* output_pose.pose.orientation.x = fitted_pose_optical.first.second.x(); */
          /* output_pose.pose.orientation.y = fitted_pose_optical.first.second.y(); */
          /* output_pose.pose.orientation.z = fitted_pose_optical.first.second.z(); */
          /* output_pose.pose.orientation.w = fitted_pose_optical.first.second.w(); */
          /* for (int i=0; i<fitted_pose_optical.second.cols(); i++){ */
          /*   for (int j=0; j<fitted_pose_optical.second.rows(); j++){ */
          /*     output_pose.covariance[fitted_pose_optical.second.cols()*j+i] =  fitted_pose_optical.second(j,i); */
          /*   } */
          /* } */

          /* if (_debug_){ */
          /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: Y: \n" << fitted_pose_optical.first.first.transpose() << " : [" << fitted_pose_optical.first.second.x() << "," << fitted_pose_optical.first.second.y() << "," <<fitted_pose_optical.first.second.z() << "," <<fitted_pose_optical.first.second.w() << "]" ); */
          /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: Py: \n" << fitted_pose_optical.second ); */
          /* } */

          /* return{}; */
      /* } */
      //}

          e::Vector3d getRoughInit([[ maybe_unused ]] LEDModel model, std::vector<e::Vector3d> v_w, [[ maybe_unused ]] int image_index){

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
            double l_max = UVDAR_RANGE(image_index);
            if ((int)(v_w.size()) > 1){
              /* auto [d_max, d_min] = model.getMaxMinVisibleDiameter(); */
              double alpha_max = getLargestAngle(v_w);

              l_max = (maxdiameter_/2.0)/tan(alpha_max/2.0);
              /* if (_debug_) */
              /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: d_max: " << d_max << "; alpha_max: " << alpha_max << "; l_rough: " << l_max); */
            }

            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: d_min: " << d_min << "; alpha_min: " << alpha_min << "; l_rough: " << l_max); */
          

            /* return baseFromOptical(v_avg*l_max)*1.25; */
            return v_avg*l_max*1.25;
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

          void initModelsAndAxisVectors(int orientation_step_count=8){ // spread out the axis of initial hypotheses - presume that Z is roughly upwards
            for (unsigned int i=0; i<_camera_count_; i++){
              axis_vectors_.push_back(std::vector<e::Vector3d>());
              initial_orientations_.push_back(std::vector<std::pair<e::AngleAxisd,LEDModel>>());
            }

            axis_vectors_global_.push_back(e::Vector3d::UnitZ());

            axis_vectors_global_.push_back(e::Vector3d::UnitX());
            axis_vectors_global_.push_back(e::Vector3d::UnitY());

            axis_vectors_global_.push_back(e::Vector3d(sqrt(0.5), sqrt(0.5), 0.0));
            axis_vectors_global_.push_back(e::Vector3d(-sqrt(0.5), sqrt(0.5), 0.0));

            axis_vectors_global_.push_back(e::Vector3d(0.0, sqrt(0.5), sqrt(0.5)));
            axis_vectors_global_.push_back(e::Vector3d(0.0, -sqrt(0.5), sqrt(0.5)));
            axis_vectors_global_.push_back(e::Vector3d(sqrt(0.5), 0.0, sqrt(0.5)));
            axis_vectors_global_.push_back(e::Vector3d(-sqrt(0.5), 0.0, sqrt(0.5)));

            double angle_step = 2.0*M_PI/(double)(orientation_step_count);

            for (auto v : axis_vectors_global_){
              for (int j=0; j<orientation_step_count; j++){
                auto orientation_tent = e::AngleAxisd(j*angle_step,v);
                  /* if (REJECT_UPSIDE_DOWN){ */
                  /*   if ((((camera_view_global_.inverse())*(orientation_tent * (camera_view_global_ * e::Vector3d::UnitZ())) ).z()) < 0.1){ */
                  /*     continue; */
                  /*   } */
                  /* } */
                  initial_orientations_global_.push_back(
                      std::pair(
                        orientation_tent,
                        model_.rotate(orientation_tent)
                        )
                      );
              }
            }

          }

          /* bool prepareModelsAndOrientations(unsigned int image_index, int orientation_step_count=8){ // spread out the axis of initial hypotheses - presume that Z is roughly upwards */
          /*   if (!tf_fcu_to_cam[image_index]){ */
          /*     ROS_INFO_STREAM("[UVDARPoseCalculator]: Transformation for camera " << image_index << " is missing, returning."); */
          /*     return false; */
          /*   } */
          /*   const auto tf = tf_fcu_to_cam[image_index].value(); */

          /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: camera " << image_index << " optical rotation: " << std::endl <<  tf2::transformToEigen(tf.transform).rotation()); */
          /*   camera_view_[image_index] = rot_optical_to_base*tf2::transformToEigen(tf.transform).rotation(); */

          /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: Composed rotation matrix"); */
          /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: \n" << camera_view_[image_index].toRotationMatrix()); */

          /*   ROS_INFO_STREAM("[UVDARPoseCalculator]: Initializing sample rotation axes for camera " << image_index); */
          /*   for (auto av : axis_vectors_global_){ */
          /*     axis_vectors_[image_index].push_back(camera_view_[image_index]*av); */
          /*     ROS_INFO_STREAM("[UVDARPoseCalculator]:   - " << av.transpose()); */
          /*   } */

          /*   LEDModel model_local = model_.rotate(camera_view_[image_index]); */

          /*   double angle_step = 2.0*M_PI/(double)(orientation_step_count); */

          /*   for (auto v : axis_vectors_[image_index]){ */
          /*     for (int j=0; j<orientation_step_count; j++){ */
          /*       auto orientation_tent = e::AngleAxisd(j*angle_step,v); */
          /*         if (REJECT_UPSIDE_DOWN){ */
          /*           if ((((camera_view_[image_index].inverse())*(orientation_tent * (camera_view_[image_index] * e::Vector3d::UnitZ())) ).z()) < 0.1){ */
          /*             continue; */
          /*           } */
          /*         } */
          /*         initial_orientations_[image_index].push_back( */
          /*             std::pair( */
          /*               orientation_tent, */
          /*               model_local.rotate(orientation_tent) */
          /*               ) */
          /*             ); */
          /*     } */
          /*   } */

          /*   return true; */
          /* } */



          std::pair<std::vector<Hypothesis>,std::vector<double>> getViableInitialHyptheses( std::vector<ImagePointIdentified> observed_points, e::Vector3d furthest_position, int target, int image_index, geometry_msgs::TransformStamped fromcam_tf, geometry_msgs::TransformStamped tocam_tf, int initial_hypothesis_count, ros::Time time){
            /* const auto start = profiler.getTime(); */

            e::FullPivLU<e::MatrixXd> lu(furthest_position.normalized().transpose());
            e::MatrixXd l_null_space = lu.kernel();
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: span: \n" << psi); */
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: nullspace: \n" << l_null_space); */
            e::Vector3d side_shift_init = l_null_space.topLeftCorner(3,1).normalized()*maxdiameter_;

            double threshold = (int)(observed_points.size())*ERROR_THRESHOLD_INITIAL(image_index);

            ReprojectionContext rpc;
            rpc.image_index=image_index;
            rpc.tocam_tf=tocam_tf;
            rpc.target=target;
            rpc.observed_points=observed_points;
            rpc.model=model_;


            std::vector<Hypothesis> initial_hypotheses;
            std::vector<double> errors;
            int iter = 0;
            for (; (int)(initial_hypotheses.size()) < initial_hypothesis_count;){
                double d = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);//random double form 0 to 1
                double sidestep_direction = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);//random double form 0 to 1
                double sidestep_distance = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);//random double form 0 to 1

                auto side_shift_local = (e::AngleAxisd(2.0*M_PI*sidestep_direction,furthest_position.normalized())*side_shift_init)*sidestep_distance;
                /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Side shift: " << side_shift_local.transpose()); */

                e::Vector3d current_position = (furthest_position*d) + (side_shift_local);

                double a = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);//scaler form 0 to 1
                e::Quaterniond current_orientation(e::AngleAxisd(a*2.0*M_PI, e::Vector3d::Random().normalized()));

                auto global_position = transform(current_position, fromcam_tf);

                /* rpc.model=model_.rotate(current_orientation).translate(global_position.value()); */
                Hypothesis hypo_new;
                hypo_new.index = target;
                hypo_new.pose = {.position=global_position.value(), .orientation=current_orientation};
                hypo_new.flag = neutral;
                hypo_new.observed = time;
                hypo_new.propagated = time;
                double error_total = hypothesisError(hypo_new, rpc);

                /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Err: "<< error_total << " from P: " <<  global_position.value().transpose() << "; R: " << quaternionToRPY(current_orientation).transpose()); */

                
                if (error_total < threshold){

                  initial_hypotheses.push_back(hypo_new);
                  errors.push_back(error_total);
                }
                iter++;
                if (iter > MAX_INIT_ITERATIONS){
                  break;
                }

            }
            if (_debug_)
              ROS_INFO_STREAM("[UVDARPoseCalculator]: Iterations for initialization : "<< iter );
            return {initial_hypotheses, errors};
          }

          std::vector<Hypothesis> refineByMutation(const LEDModel& model, const std::vector<ImagePointIdentified>& observed_points, std::vector<Hypothesis> &hypotheses,  geometry_msgs::TransformStamped tocam_tf, int image_index, int target, double threshold_local, double position_max_step, double angle_max_step, unsigned int desired_count){
            std::vector<Hypothesis> output;
            double threshold = (int)(observed_points.size())*threshold_local;

            ReprojectionContext rpc;

            rpc.model=model;
            rpc.target=target;
            rpc.tocam_tf=tocam_tf;
            rpc.image_index=image_index;
            rpc.observed_points = observed_points;

            for (auto &h : hypotheses){
              if ( hypothesisError(h, rpc) < threshold){
                output.push_back(h);
                output.back().flag = neutral;
              }
            }

            int iter = 0;
            for (; (unsigned int)(output.size()) < desired_count;){
            for (auto &h : hypotheses){

                auto new_mutations = generateMutations(h, 1, h.observed, position_max_step, angle_max_step);
                for (auto &hm : new_mutations){
                  double error_curr = hypothesisError(hm, rpc);
                  if (error_curr < threshold){
                    output.push_back(hm);
                    output.back().flag = neutral;
                    if (_debug_)
                      ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Adding hypo with error of " << error_curr);
                  }
                  else {
                    /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Discarding hypo with error of " << error_curr << " vs. " << threshold); */
                  }
                }
              }

              iter++;
              if (iter > MAX_MUTATION_REFINE_ITERATIONS){
                break;
              }
            }

            if (_debug_)
              ROS_INFO_STREAM("[UVDARPoseCalculator]: Iterations for mutation : "<< iter );

            return output;

          }

          /* std::pair<Hypothesis, double> iterFitFull(const LEDModel& model, const std::vector<ImagePointIdentified>& observed_points, const Hypothesis& hypothesis, int target, int image_index, geometry_msgs::TransformStamped tocam_tf, Profiler &profiler) */
          /* { */
          /*   const auto start = profiler.getTime(); */

          /*   ReprojectionContext rpc; */
          /*   rpc.image_index=image_index; */
          /*   rpc.tocam_tf=tocam_tf; */
          /*   rpc.observed_points=observed_points; */
          /*   rpc.target=target; */

          /*   /1* if (_debug_) *1/ */
          /*   /1*   ROS_INFO_STREAM("[UVDARPoseCalculator]: Refined hypotheses for target " << target << " in image " << image_index << ": "); *1/ */
          /*   e::Vector3d position_curr = hypothesis.pose.position; */
          /*   const e::Quaterniond orientation_start  = hypothesis.pose.orientation; */
          /*   e::Quaterniond orientation_curr  = orientation_start; */
          /*   auto model_curr = model.rotate(orientation_curr).translate(position_curr); */



          /*   const double pos_step_init = 1.0; */
          /*   const double angle_step_init = 0.1; */

          /*   std::shared_ptr<std::vector<ImagePointIdentified>> projected_points = std::make_shared<std::vector<ImagePointIdentified>>(); */
            
          /*   rpc.model=model_curr; */
          /*   double error_total = totalError(rpc, projected_points, _debug_); */

          /*   if (_debug_){ */
          /*     ROS_INFO_STREAM("[UVDARPoseCalculator]: Proj. points:"); */
          /*     for (const auto& p : *projected_points){ */
          /*       ROS_INFO_STREAM("[UVDARPoseCalculator]: " << p.position.x << ":\t" << p.position.y << "; ID:" << p.ID); */
          /*     } */
          /*   } */
          /*   using vec6_t = e::Matrix<double, 6, 1>; */
          /*   vec6_t gradient = std::numeric_limits<double>::max()*vec6_t::Ones(); */
          /*   const double threshold = (double)(observed_points.size())*ERROR_THRESHOLD_FITTED(image_index); */
          /*   if (_debug_) */
          /*     ROS_INFO_STREAM("[UVDARPoseCalculator]: total error init: " << error_total); */
          /*   profiler.addValueSince("Variable initialization, initial e="+std::to_string(error_total),start); */


          /*   profiler.indent(); */
          /*   int iters = 0; */
          /*   double prev_error_total = error_total*1.5; */
          /*   while ((error_total > (threshold)) && ((prev_error_total - error_total) > (prev_error_total*0.1)) && (iters < 2)){ */
          /*     const auto loop_start = profiler.getTime(); */
          /*     prev_error_total = error_total; */
          /*     int grad_iter = 0; */
          /*     for (int dim = 0; dim < 3; dim++){ */
          /*       double pos_step = 0.1; */
          /*       bool extreme = false; */
          /*       int it=0; */
          /*       double a_error = 0.0, b_error = 0.0; */
          /*       while (it<1000){//get local position gradient */
          /*         it++; */
          /*         grad_iter++; */
          /*         e::Vector3d grad_vec = e::Vector3d::Zero(); */
          /*         grad_vec(dim) = pos_step; */
          /*         const LEDModel shape_a = model_curr.translate(grad_vec); */
          /*         const LEDModel shape_b = model_curr.translate(-grad_vec); */

          /*         rpc.model=shape_a; */
          /*         a_error = totalError(rpc); */
          /*         rpc.model=shape_b; */
          /*         b_error = totalError(rpc); */
          /*         /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  1"); *1/ */
          /*         if ( (pos_step > (0.01*pos_step_init)) && (sgn(b_error - error_total) == sgn(a_error - error_total))) { */
          /*         /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  2"); *1/ */
          /*           pos_step /= 2; */
          /*           if (pos_step <= (0.01*pos_step_init)){ */
          /*         /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  3"); *1/ */
          /*             extreme = true; */
          /*             break; */
          /*           } */
          /*           continue; */
          /*         } */
          /*         if (pos_step <= (0.01*pos_step_init)){ */
          /*         /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  4"); *1/ */
          /*           extreme = true; */
          /*         } */
          /*         /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  5"); *1/ */
          /*         break; */
          /*       } */

          /*       if (!extreme) */
          /*         gradient(dim) = ((a_error-b_error)/(2*pos_step))/((double)(observed_points.size())); */
          /*       else */ 
          /*         gradient(dim) = 0; */
          /*     } */

          /*     profiler.addValueSince("Position gradient (it:"+std::to_string(grad_iter)+",g="+std::to_string(gradient.head<3>().norm())+")",loop_start); */

          /*     if ((gradient.head<3>().norm()) > (0.01)){//to check for extremes */
          /*       double grad_norm = gradient.head<3>().norm(); */
          /*       const double m_lin_gradient = grad_norm; */
          /*       /1* const double t_parameter = -0.5*m_lin_gradient; *1/ */
          /*       const double t_parameter = 0.5*m_lin_gradient; */
          /*       double dist = 0.0; */
          /*       const double lin_step_init = 0.01; */
          /*       double alpha_lin_step = lin_step_init; */
          /*       const e::Vector3d p_step_dir = -(gradient.head<3>().normalized()); */
          /*       /1* const e::Vector3d p_step_dir = -(gradient.head<3>()); *1/ */
          /*       const double error_shift_prev = error_total; */
          /*       double error_shift_curr = error_total; */
          /*       auto model_shifted_curr = model_curr; */
          /*       const auto loop_start_gradient_descent = profiler.getTime(); */

          /*       int j=0; */
          /*       /1* double condition = std::numeric_limits<double>::max(); *1/ */

          /*       bool shrinking; */
          /*       model_shifted_curr = model_curr.translate(p_step_dir*alpha_lin_step); */

          /*       rpc.model=model_shifted_curr; */
          /*       error_shift_curr = totalError(rpc); */
          /*       if ((error_shift_prev-error_shift_curr) >= (alpha_lin_step*t_parameter)){ */
          /*         shrinking = false; */
          /*       } */
          /*       else{ */
          /*         shrinking = true; */
          /*       } */

          /*       bool backtracking_failed = false; */
          /*       /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  Starting Armijo"); *1/ */
          /*       /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  Initial error: " << error_total); *1/ */
          /*       /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  Initial direction: " << p_step_dir.transpose()); *1/ */
          /*       /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  Gradient: " << gradient.head<3>().transpose()); *1/ */
          /*       while (j < 100){ */
          /*         j++; */
          /*         model_shifted_curr = model_curr.translate(p_step_dir*alpha_lin_step); */
          /*       rpc.model=model_shifted_curr; */
          /*         error_shift_curr = totalError(rpc); */

          /*         if (!shrinking){ */
          /*           /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  grow?"); *1/ */
          /*           /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  current erorr: " << error_shift_curr); *1/ */
          /*           /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  compare: " << (error_shift_prev-error_shift_curr) << " vs " << (alpha_lin_step*t_parameter)); *1/ */
          /*           /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  condition: " << ((error_shift_prev-error_shift_curr) - (alpha_lin_step*t_parameter))); *1/ */
          /*           if ((error_shift_prev-error_shift_curr) >= (alpha_lin_step*t_parameter)){ */
          /*             /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  growing"); *1/ */
          /*             alpha_lin_step *= 2.0; */
          /*           } */
          /*           else { */
          /*             /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  terminating"); *1/ */
          /*             dist = alpha_lin_step*0.5; */
          /*             break; */
          /*           } */
          /*         } */
          /*         else { */
          /*           /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  shrink?"); *1/ */
          /*           /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  current erorr: " << error_shift_curr); *1/ */
          /*           /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  compare: " << (error_shift_prev-error_shift_curr) << " vs " << (alpha_lin_step*t_parameter)); *1/ */
          /*           /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  condition: " << ((error_shift_prev-error_shift_curr) - (alpha_lin_step*t_parameter))); *1/ */
          /*           if ((error_shift_prev-error_shift_curr) < (alpha_lin_step*t_parameter)){ */
          /*             /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  shrinking"); *1/ */
          /*             alpha_lin_step *= 0.5; */
          /*           } */
          /*           else { */
          /*             /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  terminating"); *1/ */
          /*             dist = alpha_lin_step; */
          /*             break; */
          /*           } */
          /*         } */

          /*         if ((alpha_lin_step < 0.01) || (alpha_lin_step > 20)){ */
          /*           backtracking_failed = true; */
          /*           /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  Breaking Armijo, step is extreme."); *1/ */
          /*           break; */
          /*         } */
          /*       } */
          /*       if (!backtracking_failed){ */
          /*         position_curr += p_step_dir*dist; */
          /*         model_curr = model_curr.translate(p_step_dir*dist); */
          /*         rpc.model=model_curr; */
          /*         error_total = totalError(rpc);; */
          /*         /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  Moved the hypothesis by "<< dist << "m"); *1/ */
          /*       } */
          /*       /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  Final error: " << error_total); *1/ */

          /*       profiler.addValueSince("Position fitting - iter. "+std::to_string(j)+" (e="+std::to_string(error_total)+")",loop_start_gradient_descent); */
          /*   } */

          /*   /1* iters++; *1/ */
          /*   /1* profiler.addValueSince("Iteration loop position "+std::to_string(iters),loop_start); *1/ */
          /*   /1* } *1/ */

          /*   /1* iters = 0; *1/ */

          /*   /1* prev_error_total = error_total+(threshold); *1/ */
          /*   /1* while ((error_total > (threshold*0.1)) && ((prev_error_total - error_total) > (prev_error_total*0.10)) && (iters < 50)){ *1/ */
          /*     /1* const auto loop_start = profiler.getTime(); *1/ */
          /*     const auto rot_steps = profiler.getTime(); */
          /*     /1* prev_error_total = error_total; *1/ */
          /*     grad_iter = 0; */
          /*     for (int dim = 0; dim < 3; dim++){ */
          /*       double angle_step = 0.1; */
          /*       bool extreme = false; */
          /*       int it=0; */
          /*       double a_error = 0.0, b_error = 0.0; */
          /*       while (it<1000){ */
          /*         it++; */
          /*         grad_iter++; */

          /*         e::Vector3d grad_axis_vec = e::Vector3d::Zero(); */
          /*         grad_axis_vec(dim) = 1; */
          /*         const LEDModel shape_a = model_curr.rotate(position_curr,  orientation_curr*grad_axis_vec, angle_step); */
          /*         const LEDModel shape_b = model_curr.rotate(position_curr,  orientation_curr*grad_axis_vec, -angle_step); */

          /*         rpc.model=shape_a; */
          /*         a_error = totalError(rpc); */
          /*         /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: ccw: " << a_error); *1/ */
          /*         rpc.model=shape_b; */
          /*         b_error = totalError(rpc); */
          /*         /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: cw: " << b_error); *1/ */

          /*         if ( (angle_step > (0.1*angle_step_init)) && (sgn(b_error - error_total) == sgn(a_error - error_total))) { */
          /*           angle_step /= 2; */
          /*           if (angle_step <= (0.1*angle_step_init)){ */
          /*             extreme = true; */
          /*             break; */
          /*           } */
          /*           continue; */
          /*         } */
          /*         break; */
          /*       } */

          /*       if (!extreme) */
          /*         gradient(3+dim) = ((a_error-b_error)/(2*angle_step))/((double)(observed_points.size())); */
          /*       else */
          /*         gradient(3+dim) = 0; */
          /*     } */

          /*     profiler.addValueSince("Orientation gradient (it:"+std::to_string(grad_iter)+",g="+std::to_string(gradient.head<3>().norm())+")", rot_steps); */

          /*     if ((gradient.tail<3>().norm()) > (0.01)){//to check for extremes */
          /*       const double rot_step_init = angle_step_init/2.0; */
          /*       double alpha_rot_step = rot_step_init; */
          /*       const e::Vector3d norm_gradient = gradient.tail<3>().normalized()*0.01;//small, to avoid cross-axis influence */
          /*       const e::Vector3d p_step_axis = */
          /*         e::AngleAxisd( */
          /*             e::AngleAxisd(-norm_gradient(0), e::Vector3d::UnitX()) * */
          /*             e::AngleAxisd(-norm_gradient(1), e::Vector3d::UnitY()) * */
          /*             e::AngleAxisd(-norm_gradient(2), e::Vector3d::UnitZ()) */
          /*             ).axis(); */

          /*       /1* -(gradient.topRightCorner(3,1).normalized()); *1/ */
          /*       const double error_rot_prev = error_total; */
          /*       double error_rot_curr = error_total; */
          /*       auto model_rotated_curr = model_curr; */

          /*       const double m_rot_gradient = gradient.tail<3>().norm(); */
          /*       const double t_parameter = 0.5*m_rot_gradient; */
          /*       double angle = 0.0; */
          /*       int j=0; */
          /*       /1* double condition = std::numeric_limits<double>::max(); *1/ */

          /*       bool shrinking; */
          /*       model_rotated_curr = model_curr.rotate(position_curr, p_step_axis,alpha_rot_step); */
          /*       rpc.model=model_rotated_curr; */
          /*       error_rot_curr = totalError(rpc); */
          /*       /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  prev: "<< error_rot_prev << ", curr: " << error_rot_curr); *1/ */
          /*       if ((error_rot_prev-error_rot_curr) >= (alpha_rot_step*t_parameter)){ */
          /*         shrinking = false; */
          /*       } */
          /*       else{ */
          /*         shrinking = true; */
          /*       } */

          /*       bool backtracking_failed = false; */
          /*       while (j < 10){ */
          /*         j++; */
          /*         model_rotated_curr = model_curr.rotate(position_curr, p_step_axis,alpha_rot_step); */
          /*         rpc.model=model_rotated_curr; */
          /*         error_rot_curr = totalError(rpc); */
                  
          /*         /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  prev: "<< error_rot_prev << ", curr: " << error_rot_curr); *1/ */
          /*         /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  alpha_rot_step: "<< alpha_rot_step << ", t_parameter: " << t_parameter << ", product: " << alpha_rot_step*t_parameter); *1/ */
          /*         if (!shrinking){ */
          /*           if ((error_rot_prev-error_rot_curr) >= (alpha_rot_step*t_parameter)){ */
          /*             /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  growing"); *1/ */
          /*             alpha_rot_step *= 2.0; */
          /*           } */
          /*           else { */
          /*             /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  terminating"); *1/ */
          /*             angle = alpha_rot_step*0.5; */
          /*             break; */
          /*           } */
          /*         } */
          /*         else { */
          /*           if ((error_rot_prev-error_rot_curr) < (alpha_rot_step*t_parameter)){ */
          /*             /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  shrinking"); *1/ */
          /*             alpha_rot_step *= 0.5; */
          /*           } */
          /*           else { */
          /*             /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  terminating"); *1/ */
          /*             angle = alpha_rot_step; */
          /*             break; */
          /*           } */
          /*         } */

          /*         if (alpha_rot_step > 1.5){ */
          /*           backtracking_failed = true; */
          /*           /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  Breaking Armijo, step is extreme: " << alpha_rot_step); *1/ */
          /*           break; */
          /*         } */
          /*         if (alpha_rot_step < 0.0001){ */
          /*           /1* backtracking_failed = true; *1/ */
          /*           /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  Breaking Armijo, step is too small: " << alpha_rot_step); *1/ */
          /*           angle = alpha_rot_step*2.0; */
          /*           break; */
          /*         } */
          /*       } */
          /*       if (!backtracking_failed){ */
          /*         orientation_curr = e::AngleAxisd(angle,p_step_axis)*orientation_curr; */
          /*         model_curr = model_curr.rotate(position_curr, p_step_axis,angle); */
          /*         rpc.model=model_curr; */
          /*         error_total = totalError(rpc, projected_points,_debug_); */
          /*         /1* ROS_INFO_STREAM("[UVDARPoseCalculator]:  Moved the hypothesis by "<< rad2deg(angle) << "deg"); *1/ */
          /*       } */
          /*       /1* else { *1/ */
          /*       /1* profiler.addValue("backtracking_failed"); *1/ */
          /*       /1* } *1/ */

          /*       profiler.addValue("Orientation fitting - iter. "+std::to_string(j)+" (e="+std::to_string(error_total)+")"); */
          /*     } */

          /*     if ((orientation_curr.angularDistance(orientation_start)) > (deg2rad(50))){ */
          /*       error_total = -1; */
          /*       /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: Skipping hypothesis - fitting went too far."); *1/ */
          /*       break; */
          /*     } */

          /*     iters++; */
          /*     profiler.addValueSince("Iteration loop "+std::to_string(iters),loop_start); */
          /*     } */
          /*     profiler.unindent(); */

          /*     profiler.addValue("Main fitting loop (e="+std::to_string(error_total)+")"); */

          /*     profiler.addValue("Final operations"); */
              
          /*   if (_debug_){ */
          /*     ROS_INFO_STREAM("[UVDARPoseCalculator]: Proj. points fitted:"); */
          /*     for (const auto& p : *projected_points){ */
          /*       ROS_INFO_STREAM("[UVDARPoseCalculator]: " << p.position.x << ":\t" << p.position.y << "; ID:" << p.ID); */
          /*     } */
          /*   } */

          /*   Hypothesis hypo_new; */
          /*   hypo_new.index = target; */
          /*   hypo_new.pose = {.position = position_curr, .orientation = orientation_curr}; */
          /*   hypo_new.flag = verified; */
          /*   hypo_new.updated = hypothesis.updated; */

          /*     return {hypo_new, error_total}; */
      /* } */

      double hypothesisError(Hypothesis &hypothesis, const ReprojectionContext rpc, std::shared_ptr<std::vector<ImagePointIdentified>> projected_points={}, bool return_projections=false, bool discrete_pixels=false){
        if  ( rpc.target != hypothesis.index){
          ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Hypothesis " << hypothesis.unique_id << " has different ID of " << hypothesis.index << " compared to the target with id " << rpc.target);
          return -1;
        }

        e::Vector3d position_curr = hypothesis.pose.position;
        const e::Quaterniond orientation_curr  = hypothesis.pose.orientation;
        auto model_curr = rpc.model.rotate(orientation_curr).translate(position_curr);
        auto rpc_tr = rpc;
        rpc_tr.model = model_curr;

        double error_total = modelError(rpc_tr, projected_points, return_projections, discrete_pixels);
        return error_total;
      }

      double modelError(const ReprojectionContext rpc, std::shared_ptr<std::vector<ImagePointIdentified>> projected_points={}, bool return_projections=false, bool discrete_pixels=false){

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
        for (auto marker : rpc.model){
          /* auto curr_projected =  camPointFromModelPoint(marker, image_index); */
          auto curr_projected =  camPointFromGlobal(marker, rpc.image_index, rpc.tocam_tf);

          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: marker: pos: " << marker.position.transpose() << " rot: " << quaternionToRPY(marker.orientation).transpose()  << " : " << (target*signals_per_target_)+marker.signal_id); */
          if (
              (std::get<0>(curr_projected).position.x>-0.5) && // edge of the leftmost pixel
              (std::get<0>(curr_projected).position.y>-0.5) && // edge of the topmost pixel
              (std::get<0>(curr_projected).position.x<(_oc_models_[rpc.image_index].width + 0.5)) && // edge of the rightmost pixel
              (std::get<0>(curr_projected).position.y<(_oc_models_[rpc.image_index].height+ 0.5)) // edge of the bottommost pixel
             ){
            projected_markers.push_back({
                .position = std::get<0>(curr_projected).position,
                .signal_id = marker.signal_id,
                .cos_view_angle = std::get<2>(curr_projected),
                .distance = std::get<1>(curr_projected),
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
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: C:" << image_index << ", cos_angle: " << cos_angle << " distance: " << distance << " led_intensity: " << led_intensity); */
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: C:" << image_index << ", ID: " << marker.signal_id <<  ", marker pos: x:" << marker.position.x << " y:" << marker.position.y ); */
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
            projected_points->push_back({.ID=pt.signal_id, .position = cv::Point2i(pt.position.x,pt.position.y) });
          }
        }

        for (auto& obs_point : rpc.observed_points){

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
            if (_signal_ids_.at(((rpc.target%1000)*signals_per_target_)+proj_point.signal_id) == (int)(obs_point.ID)){
              double tent_image_distance;
              if (!discrete_pixels){
                tent_image_distance = cv::norm(proj_point.position - cv::Point2d(obs_point.position));
              }
              else {
                tent_image_distance = cv::norm(cv::Point2i(proj_point.position.x - obs_point.position.x, proj_point.position.y - obs_point.position.y));
              }
                /* ROS_INFO_STREAM("[UVDARPoseCalculator]: obs_point: " << cv::Point2d(obs_point.position.x, obs_point.position.y) << " : proj_point: " << proj_point.position); */
              /* if (tent_image_distance > 100){ */
                /* exit(3); */
              /* } */
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

          total_error += (UNMATCHED_PROJECTED_POINT_PENALTY) * std::max(0,(int)(selected_markers.size() - rpc.observed_points.size()));
          /* total_error += (UNMATCHED_OBSERVED_POINT_PENALTY) * std::max(0,(int)(observed_points.size() - selected_markers.size())); */

          return total_error;
        }

        /* std::pair<std::pair<e::Vector3d, e::Quaterniond>,e::MatrixXd> getCovarianceEstimate(LEDModel model, std::vector<ImagePointIdentified> observed_points, std::pair<e::Vector3d, e::Quaterniond> pose, int target, int image_index, geometry_msgs::TransformStamped tocam_tf){ */

        /*   ReprojectionContext rpc; */
        /*   rpc.target=target; */
        /*   rpc.image_index=image_index; */
        /*   rpc.tocam_tf=tocam_tf; */
        /*   rpc.observed_points=observed_points; */
        /*   rpc.model=model; */

        /*   /1* LEDModel model_local = model.rotate(pose.second).translate(pose.first); *1/ */

        /*   e::MatrixXd output; */

        /*   double trans_scale = 0.1; */
        /*   /1* rot_scale = 0.05; *1/ */
        /*   double rot_scale = 0.1; */
        /*   /1* auto Y0 = pose; *1/ */
        /*   /1* e::MatrixXd Y(6,(3*3*3*3*3*3)); *1/ */
        /*   e::MatrixXd Y(6,(2*2*2*2*2*2+2*6)); */
        /*   std::vector<double> Xe; */

        /*   /1* Y(0,0) = 0; *1/ */
        /*   /1* Y(1,0) = 0; *1/ */
        /*   /1* Y(2,0) = 0; *1/ */
        /*   /1* Y(3,0) = 0; *1/ */
        /*   /1* Y(4,0) = 0; *1/ */
        /*   /1* Y(5,0) = 0; *1/ */

        /*   /1* rpc.model=model_local; *1/ */
        /*   /1* double error_init = totalError(rpc, {}, false, true); *1/ */
        /*   Hypothesis hypo_new; */
        /*   hypo_new.index = target; */
        /*   hypo_new.pose = pose; */
        /*   double error_init = hypothesisError(hypo_new, rpc, {}, false, true); */
        /*   /1* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: terror: " << Xe.back() << " at: [  0, 0, 0, 0, 0, 0  ]"); *1/ */

        /*   std::vector<int> j = {-1,1}; // for 6D, we need 21 independent samples. Accounting for point symmetry, this is 42. 6D hypercube has 64 vertices, which is sufficient. */
        /*   int k = 0; */

        /*   auto Y_rpy = quaternionToRPY(pose.second); */
        /*   if (_debug_){ */
        /*     ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: RPY: [ R=" << Y_rpy(0)<< ", P=" << Y_rpy(1) << ", Y=" << Y_rpy(2) << " ]"); */
        /*   } */

        /*   for (auto x_s : j){ */
        /*     for (auto y_s : j){ */
        /*       for (auto z_s : j){ */
        /*         for (auto roll_s : j){ */
        /*           for (auto pitch_s : j){ */
        /*             for (auto yaw_s : j){ */
        /*               e::Quaterniond rotation( */
        /*                   /1* e::AngleAxisd(yaw_s*rot_scale,    camera_view_[image_index]*e::Vector3d(0,0,1)) * *1/ */
        /*                   /1* e::AngleAxisd(pitch_s*rot_scale,  camera_view_[image_index]*e::Vector3d(0,1,0)) * *1/ */
        /*                   /1* e::AngleAxisd(roll_s*rot_scale,   camera_view_[image_index]*e::Vector3d(1,0,0)) *1/ */
        /*                   e::AngleAxisd(yaw_s*rot_scale,    e::Vector3d(0,0,1)) * */
        /*                   e::AngleAxisd(pitch_s*rot_scale,  e::Vector3d(0,1,0)) * */
        /*                   e::AngleAxisd(roll_s*rot_scale,   e::Vector3d(1,0,0)) */
        /*                   ); */
        /*               Pose pose = {.position=pose.first+e::Vector3d(x_s, y_s, z_s)*trans_scale, .orientation=rotation}; */ 
        /*               hypo_new.pose = pose; */

        /*               /1* auto model_curr = model_local.rotate(pose.first,  rotation); *1/ */
        /*               /1* model_curr = model_curr.translate(e::Vector3d(x_s, y_s, z_s)*trans_scale); *1/ */

        /*               /1* rpc.model=model_curr; *1/ */

        /*               Xe.push_back(abs(hypothesisError(hypo_new, rpc, {}, false, true)-error_init)); */
        /*               if (_debug_){ */
        /*                 ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: terror: " << Xe.back() << " at: [ " << x_s<< ", " << y_s<< ", " << z_s << ", "<< roll_s<< ", " << pitch_s << ", " << yaw_s << " ]"); */
        /*               } */
        /*               Y(0,k) = x_s*trans_scale; */
        /*               Y(1,k) = y_s*trans_scale; */
        /*               Y(2,k) = z_s*trans_scale; */
        /*               Y(3,k) = roll_s*rot_scale; */
        /*               Y(4,k) = pitch_s*rot_scale; */
        /*               Y(5,k) = yaw_s*rot_scale; */
        /*               /1* } *1/ */
        /*               k++; */
        /*           } */
        /*         } */
        /*       } */
        /*     } */
        /*   } */
        /* } */

        /* for (int i=0; i<6; i++){ */
        /*   std::vector<double> shifts(6,0.0);// X Y Z ROLL PITCH YAW */
        /*   for (auto s : j){ */
        /*     shifts.at(i) = s; */

        /*     e::Quaterniond rotation( */
        /*         /1* e::AngleAxisd(shifts.at(5)*rot_scale,    camera_view_[image_index]*e::Vector3d(0,0,1)) * *1/ */
        /*         /1* e::AngleAxisd(shifts.at(4)*rot_scale,  camera_view_[image_index]*e::Vector3d(0,1,0)) * *1/ */
        /*         /1* e::AngleAxisd(shifts.at(3)*rot_scale,   camera_view_[image_index]*e::Vector3d(1,0,0)) *1/ */
        /*         e::AngleAxisd(shifts.at(5)*rot_scale,    e::Vector3d(0,0,1)) * */
        /*         e::AngleAxisd(shifts.at(4)*rot_scale,  e::Vector3d(0,1,0)) * */
        /*         e::AngleAxisd(shifts.at(3)*rot_scale,   e::Vector3d(1,0,0)) */
        /*         ); */
        /*     auto model_curr = model_local.rotate(pose.first,  rotation); */
        /*     model_curr = model_curr.translate(e::Vector3d(shifts.at(0), shifts.at(1), shifts.at(2))*trans_scale); */

        /*     rpc.model=model_curr; */
        /*     Xe.push_back(abs(totalError(rpc, {}, false, true)-error_init)); */
        /*     if (_debug_){ */
        /*       ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: terror: " << Xe.back() << " at: [ " << shifts.at(0)<< ", " << shifts.at(1)<< ", " << shifts.at(2) << ", "<< shifts.at(3)<< ", " << shifts.at(4) << ", " << shifts.at(5) << " ]"); */
        /*     } */
        /*     Y(0,k) = shifts.at(0)*trans_scale; */
        /*     Y(1,k) = shifts.at(1)*trans_scale; */
        /*     Y(2,k) = shifts.at(2)*trans_scale; */
        /*     Y(3,k) = shifts.at(3)*rot_scale; */
        /*     Y(4,k) = shifts.at(4)*rot_scale; */
        /*     Y(5,k) = shifts.at(5)*rot_scale; */

        /*     k++; */

        /*   } */
        /* } */


        /* e::VectorXd W(Xe.size()); */
        /* int i = 0; */
        /* double Wsum = 0; */
        /* for (auto xe : Xe){ */
        /*   /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: xe("<<i<<") = " << xe); *1/ */
        /*   if (xe < 0.00001) */
        /*     W(i) = sqr(QPIX); */
        /*   else */
        /*     W(i) = (sqr(QPIX)/sqrt(xe)); */
        /*   Wsum += W(i); */
        /*   /1* if (W(i) > 0.1){ *1/ */
        /*   /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: W("<<i<<") = " << W(i)); *1/ */
        /*   /1* } *1/ */
        /*   i++; */
        /* } */
        /* /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: Wsum: [\n" << Wsum << "\n]"); *1/ */
        /* /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: W_orig: [\n" << W << "\n]"); *1/ */
        /* W /= (Wsum); */
        /* /1* W *= sqr(QPIX)*observed_points.size(); *1/ */
        /* /1* W *= sqr(0.8); *1/ */
        /* /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: W: [\n" << W << "\n]"); *1/ */
        /* /1* auto y = Y*(W/Wsum); *1/ */
        /* auto y = Y*W; */
        /* /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: y: [\n" << y << "\n]"); *1/ */
        /* auto Ye = (Y-y.replicate(1,W.size())); */
        /* /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: Ye: [\n" << Ye << "\n]"); *1/ */

        /* /1* i=0; *1/ */
        /* /1* for (auto xe : Xe){ *1/ */
        /* /1*   if ((Ye*W(i)) > 0.1){ *1/ */
        /* /1*     ROS_ERROR_STREAM("[UVDARPoseCalculator]: W("<<i<<") = " << W(i)); *1/ */
        /* /1*   } *1/ */
        /* /1*   i++; *1/ */
        /* /1* } *1/ */

        /* auto P = Ye*W.asDiagonal()*Ye.transpose(); */
        /* /1* e::JacobiSVD<e::MatrixXd> svd(P, e::ComputeThinU | e::ComputeThinV); *1/ */
        /* /1* if (P.topLeftCorner(3,3).determinant() > 0.001){ *1/ */

        /* /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: P: [\n" << P << "\n]"); *1/ */
        /* /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: P determinant: [\n" << P.topLeftCorner(3,3).determinant() << "\n]"); *1/ */
        /* /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: Singular values: [\n" << svd.singularValues() << "\n]"); *1/ */
        /* /1* ROS_INFO_STREAM("[UVDARPoseCalculator]: Matrix V: [\n" << svd.matrixV() << "\n]"); *1/ */
        /* /1* } *1/ */


        /* /1* output.setIdentity(6,6); *1/ */
        /* return {{pose.first+e::Vector3d(y(0,0), y(1,0), y(2,0)),pose.second},P}; */
        /* } */

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

        /* std::pair<std::pair<e::Vector3d, e::Quaterniond>, e::MatrixXd> getMeasurementUnionSimple(std::vector<Hypothesis> meas){ */
        /*   if (meas.size() < 1){ */
        /*     ROS_ERROR_STREAM("[UVDARPoseCalculator]: No hypotheses provided. Returning!"); */
        /*     return std::pair<std::pair<e::Vector3d, e::Quaterniond>, e::MatrixXd>(); */
        /*   } */
        /*   e::Vector3d mean_pos(0,0,0); */

        /*   for (auto &m : meas){ */
        /*     mean_pos += m.pose.position; */
        /*   } */
        /*   mean_pos /= (double)(meas.size()); */

        /*   e::Quaterniond mean_rot = getAverageOrientation(meas.getVerified()); */

        /*   e::MatrixXd Mp(3,(unsigned int)(meas.size())); */
        /*   e::MatrixXd Mo(3,(unsigned int)(meas.size())); */

        /*   int i = 0; */
        /*   for (auto &m : meas){ */
        /*     Mp.block(0,i, 3,1) = m.pose.position-mean_pos; */
        /*     Mo.block(0,i, 3,1) = quaternionToRPY(m.pose.orientation*mean_rot.inverse()); */
        /*     i++; */
        /*   } */

        /*   e::Matrix3d Cp = (Mp*Mp.transpose())/(meas.size()-1); */
        /*   e::Matrix3d Co = (Mo*Mo.transpose())/(meas.size()-1); */

        /*   e::Matrix6d C = e::Matrix6d::Zero(); */
        /*   C.topLeftCorner(3,3) = Cp; */
        /*   C.bottomRightCorner(3,3) = Co; */

        /*   return {{mean_pos,mean_rot},C}; */

        /* } */

        //contains code from https://gist.github.com/PeteBlackerThe3rd/f73e9d569e29f23e8bd828d7886636a0
        e::Quaterniond getAverageOrientation(std::vector<Hypothesis> meas){
          if (meas.size() < 1){
            return e::Quaterniond::Identity();
            ROS_ERROR_STREAM("[UVDARPoseCalculator]: No measurements provided. Returning!");
          }

          // first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
          Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

          for (auto &m : meas){
            A += m.pose.orientation.coeffs()*m.pose.orientation.coeffs().transpose();
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


        std::optional<std::pair<Pose,e::MatrixXd>> getMeasurementElipsoidHull(AssociatedHypotheses meas){
          if (_debug_)
            ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Hypo count: " << meas.hypotheses.size());
          if (meas.hypotheses.size() < 1){
            ROS_ERROR_STREAM("[UVDARPoseCalculator]: No hypotheses provided. Returning!");
            return std::nullopt;
          }

          if (_debug_)
            ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Verified count: " << meas.verified_count);
          if (meas.verified_count < 1){
            ROS_ERROR_STREAM("[UVDARPoseCalculator]: No verified hypotheses provided. Returning!");
            return std::nullopt;
          }
          
          if (meas.verified_count == 1){
            e::Vector3d singleton_position;
            e::Quaterniond singleton_orientation;
            for (auto &h : meas.hypotheses){
              if (h.flag == verified){
                singleton_position = h.pose.position;
                singleton_orientation = h.pose.orientation;
              }
            }
            e::Matrix6d C = e::Matrix6d::Zero();
            C.topLeftCorner(3,3) = 0.5*e::Matrix3d::Identity();
            C.bottomRightCorner(3,3) = 0.5*e::Matrix3d::Identity();
            std::pair<Pose,e::MatrixXd> output = {{.position=singleton_position,.orientation=singleton_orientation},C};
            return output;
          }

          if (_debug_)
            ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Verified count: " << meas.verified_count);

          // position
          std::vector<e::Vector3d> meas_pos;

          e::Vector3d mean_pos(0.0,0.0,0.0);

          for (auto &m : meas.hypotheses){
            if (m.flag == verified){
              meas_pos.push_back(m.pose.position);
              mean_pos += m.pose.position;
            }
          }
          mean_pos /= ((double)(meas.hypotheses.size()));
          std::vector<e::Vector3d> meas_pos_diff;
          if (_debug_)
            ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Verified count: " << meas.verified_count);

          // orientation
          e::Quaterniond mean_rot = getAverageOrientation(meas.getVerified());
          std::vector<e::Vector3d> meas_rpy_diff;

          for (auto &m : meas.hypotheses){
            if (m.flag == verified){
              meas_pos_diff.push_back(m.pose.position-mean_pos);
              meas_rpy_diff.push_back(quaternionToRPY(m.pose.orientation*mean_rot.inverse()));
            }
          }

          if (_debug_)
            ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Verified count: " << meas.verified_count);

          if (_debug_){
            if (meas_pos_diff.size() > 0)
              ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: meas_pos_diff: " << meas_pos_diff.at(0));
            else
              ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: meas_pos_diff is empty");
          }
          /* auto Hp = get3DEnclosingEllipsoid(meas_pos_diff,0.001); */
          auto Hp = get3DEnclosingEllipsoid(meas_pos_diff);
          if ( (Hp.first.array().isNaN().any()) ||(Hp.second.array().isNaN().any()) ){
            ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Position ellipsoid contains NaNs, returning...");
            return std::nullopt;
          }

          if (_debug_){
            if (meas_rpy_diff.size() > 0)
              ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: meas_rpy_diff: " << meas_rpy_diff.at(0));
            else
              ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: meas_rpy_diff is empty");
          }

          auto Ho = get3DEnclosingEllipsoid(meas_rpy_diff);
          if ( (Ho.first.array().isNaN().any()) ||(Ho.second.array().isNaN().any()) ){
            ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Orientation ellipsoid contains NaNs, returning...");
            return std::nullopt;
          }
          
          e::Vector3d mean_pos_shift = Hp.first;
          
          e::Quaterniond mean_rot_shift =
                      e::AngleAxisd(Ho.first(0), e::Vector3d::UnitX()) *
                      e::AngleAxisd(Ho.first(1), e::Vector3d::UnitY()) *
                      e::AngleAxisd(Ho.first(2), e::Vector3d::UnitZ());

          e::Matrix6d C = e::Matrix6d::Zero();
          C.topLeftCorner(3,3) = Hp.second;
          C.bottomRightCorner(3,3) = Ho.second;

          std::pair<Pose,e::MatrixXd> output = {{.position=mean_pos+mean_pos_shift,.orientation=mean_rot*mean_rot_shift},C};
          return output;
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

          /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: N " << N); */
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
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: u_i:\n" << u.transpose()); */
          while (count < 1000){
            e::Matrix4d X = Q * u.asDiagonal() * Q.transpose();       // X = \sum_i ( u_i * q_i * q_i')  is a (d+1)x(d+1) matrix
            e::VectorXd m = (Q.transpose() * X.inverse() * Q).diagonal();  // M the diagonal vector of an NxN matrix


            int jp, jm;
            /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: A"); */
            /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: X " << X); */
            /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: Q " << Q); */
            /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: u " << u); */
            /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: m " << m); */
            double maximum = m.maxCoeff(&jp);
            /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: B"); */
            double minimum = m.minCoeff(&jm);
            /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: C"); */
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
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: P:\n" << P); */
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: u:\n" << u.transpose()); */
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
          /* e::Matrix3d A = (1.0/(double)(d)) * (P * U * P.transpose() - (P*u)*(P*u).transpose() ).inverse(); */
          /* e::Matrix3d A = (1.0/(double)(d)) * (P * U * P.transpose() - (P*u)*(P*u).transpose() ).inverse(); */

          /* if (A.array().isNaN().any()){ */
          /*   e::Vector3d c_z = P * u; */
          /*   e::Matrix3d C_z = (double)(d) * (P * U * P.transpose() - (P*u)*(P*u).transpose() ); */
          /*   return {c_z,C_z}; */
          /* } */

          //Now to convert it into covariance matrix
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: A:\n" << A); */
          /* e::JacobiSVD<e::MatrixXd> svd(A, e::ComputeThinU | e::ComputeThinV); */
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: sgval of A:\n" << svd.singularValues().transpose()); */
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: sgvec of A:\n" << svd.matrixV()); */

          /* e::Vector3d sgvs = svd.singularValues(); */
          /* e::Matrix3d D = ((sgvs.array()<0.00000001).select(std::numeric_limits<double>::max(),sgvs)).cwiseInverse().asDiagonal(); */
          /* e::Matrix3d V = svd.matrixV(); */

            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Singular values: [\n" << svd.singularValues() << "\n]"); */
            /* ROS_INFO_STREAM("[UVDARPoseCalculator]: Matrix V: [\n" << svd.matrixV() << "\n]"); */

          //
          /* e::EigenSolver<e::Matrix3d> es(A); */
          /* e::Matrix3d D = es.eigenvalues().real().cwiseInverse().asDiagonal(); */
          /* e::Matrix3d V = es.eigenvectors().real(); */

          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: D:\n" << D); */

          /* e::Matrix3d C = V*D*V.inverse(); */
          e::Matrix3d C = (double)(d) * (P * U * P.transpose() - (P*u)*(P*u).transpose() );

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

        e::Vector3d directionFromCamPoint(cv::Point2d point, int image_index){
          double v_i[2] = {(double)(point.y), (double)(point.x)};
          double v_w_raw[3];
          cam2world(v_w_raw, v_i, &(_oc_models_[image_index]));
          return e::Vector3d(v_w_raw[1], v_w_raw[0], -v_w_raw[2]);
        }

        std::pair<cv::Point2d, double> camPointFromModelPoint(LEDMarker marker, int image_index){
          /* auto marker_cam_pos = opticalFromMarker(marker); */
          auto output_position = camPointFromObjectPoint(marker.pose.position, image_index);
          e::Vector3d view_vector = -(marker.pose.position.normalized());
          /* e::Vector3d led_vector = marker_cam_pos.second*e::Vector3d(1,0,0); */
          /* e::Vector3d led_vector = fastMatrixVectorProduct(marker.second,e::Vector3d(1,0,0)); // can accelerate by just selecting the first row... */
          e::Vector3d led_vector = marker.pose.orientation*e::Vector3d(1,0,0); // can accelerate by just selecting the first row...
          double cos_view_angle = view_vector.dot(led_vector);
          /* ROS_INFO_STREAM("[UVDARPoseCalculator]: view_vector: " << view_vector.transpose() << " led_vector: " << led_vector.transpose() << " cos_view_angle: " << cos_view_angle); */
          return {output_position, cos_view_angle};
        }

        cv::Point2i camPointFromGlobal(e::Vector3d ipt, int image_index, geometry_msgs::TransformStamped tocam_tf){

          std::optional<e::Vector3d> position_transformed;
          position_transformed = transform(ipt, tocam_tf).value();
          LEDMarker tmpmarker;
          tmpmarker.pose.position = position_transformed.value();
          auto projection = camPointFromModelPoint(tmpmarker, image_index);
          return projection.first;
        }

        e::Vector3d camPositionFromGlobal(e::Vector3d ipt, geometry_msgs::TransformStamped tocam_tf){

          std::optional<e::Vector3d> position_transformed;
          position_transformed = transform(ipt, tocam_tf).value();
          return position_transformed.value();
        }

        std::tuple<ImagePointIdentified,double,double> camPointFromGlobal(LEDMarker marker, int image_index, geometry_msgs::TransformStamped tocam_tf){

          /* Pose pose_marker = {.position = marker.position, .orientation=marker.orientation}; */
          Pose pose_transformed;
          pose_transformed = transform(marker.pose, tocam_tf).value();
          LEDMarker tmpmarker;
          tmpmarker.pose = pose_transformed;
          auto projection = camPointFromModelPoint(tmpmarker, image_index);
          return {{.ID = marker.signal_id, .position = projection.first},pose_transformed.position.norm(),projection.second};
        }

        /* std::pair<e::Vector3d, e::Matrix3d> opticalFromMarker(LEDMarker marker){ */
        /*   /1* e::Quaterniond q_rotator(0.5,0.5,-0.5,0.5); *1/ */
        /*   e::Matrix3d m_rotator; */
        /*   m_rotator << \ */
        /*     0, -1, 0, \ */
        /*     0, 0, -1, \ */
        /*     1, 0, 0; */
        /*   return {fastMatrixVectorProduct(m_rotator,marker.position), fastMatrixMatrixProduct(m_rotator,marker.orientation.toRotationMatrix())}; */
        /* } */

        /* Pose opticalFromBase(Pose pose){ */
        /*   auto output_pose = pose; */

        /*   output_pose.position = rot_base_to_optical*pose.position; */
        /*   output_pose.orientation = rot_base_to_optical*pose.orientation; */
        /*   return output_pose; */
        /* } */

        /* Pose globalFromCam(Pose pose, int image_index){ */
        /*   Pose pose_transformed; */
        /*   pose_transformed = transform(pose, fromcam_tf_[image_index]).value(); */
        /*   return pose_transformed; */
        /* } */

        /* e::Vector3d globalFromCam(e::Vector3d position, int image_index){ */
        /*   e::Vector3d position_transformed; */
        /*   position_transformed = transform(position, fromcam_tf_[image_index]).value(); */
        /*   return position_transformed; */
        /* } */


        /* std::pair<Pose,e::MatrixXd> opticalFromBase(Pose pose, e::MatrixXd covariance){ */
        /*   auto output_pose = pose; */
        /*   auto output_covariance = covariance; */
        /*   /1* e::Quaterniond rotator(0.5,0.5,-0.5,0.5); *1/ */
        /*   /1* rotator = _center_fix_[image_index]*rotator; *1/ */
        /*   output_pose.position = rot_base_to_optical*pose.position; */
        /*   output_pose.orientation = rot_base_to_optical*pose.orientation; */
        /*   output_covariance.topLeftCorner(3,3) = rot_base_to_optical.toRotationMatrix()*output_covariance.topLeftCorner(3,3)*rot_base_to_optical.toRotationMatrix().transpose(); */
        /*   output_covariance.bottomRightCorner(3,3) = rot_base_to_optical.toRotationMatrix()*output_covariance.bottomRightCorner(3,3)*rot_base_to_optical.toRotationMatrix().transpose(); */
        /*   return {output_pose, output_covariance}; */
        /* } */

        /* e::Vector3d baseFromOptical(e::Vector3d input){ */
        /*   return e::Vector3d(input.z(), -input.x(), -input.y()); */
        /* } */

        cv::Point2d camPointFromObjectPoint(e::Vector3d point, int image_index){
          double v_w[3] = {point.y(), point.x(),-point.z()};
          double v_i_raw[2];
          world2cam(v_i_raw, v_w, &(_oc_models_[image_index]));
          return cv::Point2d(v_i_raw[1], v_i_raw[0]);
        }

        std::optional<Pose> transform(Pose input, geometry_msgs::TransformStamped tf){

          geometry_msgs::Pose gms_pose;
          gms_pose.position.x = input.position.x();
          gms_pose.position.y = input.position.y();
          gms_pose.position.z = input.position.z();
          
          gms_pose.orientation.w = input.orientation.w();
          gms_pose.orientation.x = input.orientation.x();
          gms_pose.orientation.y = input.orientation.y();
          gms_pose.orientation.z = input.orientation.z();

          std::optional<geometry_msgs::Pose> gms_transformed;
          {
            std::scoped_lock lock(transformer_mutex);
            gms_transformed = transformer_.transform(gms_pose, tf).value();
          }
          if (!gms_transformed){
            return std::nullopt;
          }
          else {
            Pose output;
            output.position = e::Vector3d(gms_transformed.value().position.x,gms_transformed.value().position.y,gms_transformed.value().position.z);
            output.orientation = e::Quaterniond(gms_transformed.value().orientation.w,gms_transformed.value().orientation.x,gms_transformed.value().orientation.y,gms_transformed.value().orientation.z);

            return output;
          }
        }

        std::optional<e::Vector3d> transform(e::Vector3d input, geometry_msgs::TransformStamped tf){

          geometry_msgs::Pose gms_pose;
          gms_pose.position.x = input.x();
          gms_pose.position.y = input.y();
          gms_pose.position.z = input.z();
          
          gms_pose.orientation.w = 1;
          gms_pose.orientation.x = 0;
          gms_pose.orientation.y = 0;
          gms_pose.orientation.z = 0;

          std::optional<geometry_msgs::Pose> gms_transformed;
          {
            std::scoped_lock lock(transformer_mutex);
            gms_transformed = transformer_.transform(gms_pose, tf);
          }
          if (!gms_transformed){
            return std::nullopt;
          }
          else {
            e::Vector3d output;
            output = e::Vector3d(gms_transformed.value().position.x,gms_transformed.value().position.y,gms_transformed.value().position.z);

            return output;
          }
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

          if (std::find(_signal_ids_.begin(), _signal_ids_.end(), s_i) != _signal_ids_.end()){
            return s_i/signals_per_target_;
          }
          else {
            return -1;
          }
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
        bool avgIsNearEdge(std::vector<ImagePointIdentified> points, int margin, int image_index){


          cv::Point2d mean(0,0);
          for (auto &point : points){
            mean += cv::Point2d(point.position);
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
          for (auto &latest_img_data : latest_input_data_){
            /* std::scoped_lock lock(*(mutex_separated_points_[image_index])); */
            cv::Point start_point = cv::Point(start_widths[image_index]+image_index, 0);
            if (image_index > 0){
              cv::line(output_image, start_point+cv::Point2i(-1,0), start_point+cv::Point2i(-1,max_image_height-1),cv::Scalar(255,255,255));
            }

            for (auto &point : latest_img_data.points){
              cv::Point center = start_point + cv::Point2i(point.x,point.y);
              cv::Scalar color = ColorSelector::markerColor(point.value);
              cv::circle(output_image, center, 5, color);
            }
            image_index++;
          }

          return 0;
        }

        //}
        //

        ros::Time newer(ros::Time a, ros::Time b){
          if (a > b)
            return a;
          else
            return b;
        }


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

        Profiler profiler_main_;
        Profiler profiler_thread_;


        using blinkers_seen_callback_t = boost::function<void (const uvdar_core::ImagePointsWithFloatStampedConstPtr& msg)>;
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

        /* std::vector<ros::Publisher> pub_measured_poses_; */
        ros::Publisher pub_measured_poses_;
        /* std::vector<ros::Publisher> pub_constituent_poses_; */
        ros::Publisher pub_hypotheses_;
        ros::Publisher pub_hypotheses_tentative_;

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
        /* double _beacon_height_; */

        bool _quadrotor_;
        /* bool _beacon_; */
        bool _custom_model_;

        std::string _output_frame_;

        bool _separate_by_distance_;
        double _max_cluster_distance_;

        LEDModel model_;

        double maxdiameter_, mindiameter_;

        bool initialized_ = false;
        std::vector<bool> input_data_initialized;
        /* std::vector<bool> batch_processsed_; */
        bool batch_processsed_ = true;

        std::vector<std::vector<e::Vector3d>> axis_vectors_;
        std::vector<e::Vector3d> axis_vectors_global_;
        std::vector<std::vector<std::pair<e::AngleAxisd,LEDModel>>> initial_orientations_;
        std::vector<std::pair<e::AngleAxisd,LEDModel>> initial_orientations_global_;

        /* std::vector<std::shared_ptr<std::mutex>>  mutex_separated_points_; */
        /* std::vector<std::vector<std::pair<int,std::vector<cv::Point3d>>>> separated_points_; */

        double led_projection_coefs_[3] = {1.3398, 31.4704, 0.0154}; //empirically measured coefficients of the decay of blob radius wrt. distance of a LED in our UVDAR cameras.


        std::mutex input_mutex;
        std::mutex hypothesis_mutex;
        std::mutex transformer_mutex;
        std::mutex threadconfig_mutex;
        mrs_lib::Transformer transformer_;
        /* std::vector<std::optional<geometry_msgs::TransformStamped>> tf_fcu_to_cam; */
        /* std::vector<e::Quaterniond> camera_view_; */

        /* e::Quaterniond rot_base_to_optical = e::Quaterniond(0.5,0.5,-0.5,0.5); */
        /* e::Quaterniond rot_optical_to_base = e::Quaterniond(0.5,-0.5,0.5,-0.5); */

        std::unique_ptr<mrs_lib::ThreadTimer>  initializer_thread_=nullptr;
        std::unique_ptr<mrs_lib::ThreadTimer>  particle_filtering_thread_=nullptr;
        ros::Timer timer_initializer_;
        ros::Timer timer_particle_filter_;

        std::vector<InputData> latest_input_data_;
        /* std::vector<geometry_msgs::TransformStamped> tocam_tf_; */
        /* std::vector<geometry_msgs::TransformStamped> fromcam_tf_; */
        /* std::vector<InputData> previous_input_data_; */

        std::vector<AssociatedHypotheses> hypothesis_buffer_;

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
