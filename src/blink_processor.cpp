#undef _DEBUG

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <ht4dbt/ht4d.h>
#include <color_selector/color_selector.h>
#include <frequency_classifier/frequency_classifier.h>
#include <std_msgs/Float32.h>
#include <mrs_msgs/ImagePointsWithFloatStamped.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/image_publisher.h>
#include <stdint.h>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <atomic>
#include <fstream>

namespace uvdar {

  /**
   * @brief A nodelet for extracting blinking frequencies and image positions of intermittently appearing image points
   */
  class UVDARBlinkProcessor : public nodelet::Nodelet {
    public:

      /**
       * @brief Initializer - loads parameters and initializes necessary structures
       */
      /* onInit() //{ */
      void onInit() {
        ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

        mrs_lib::ParamLoader param_loader(nh_, "UVDARBlinkProcessor");

        param_loader.loadParam("uav_name", _uav_name_, std::string());
        param_loader.loadParam("debug", _debug_, bool(false));
        param_loader.loadParam("visual_debug", _visual_debug_, bool(false));
        if ( _visual_debug_) {
          ROS_WARN_STREAM("[UVDARBlinkProcessor]: You are using visual debugging. This option is only meant for development. Activating it significantly increases load on the system and the user should do so with care.");
        }

        param_loader.loadParam("gui", _gui_, bool(false));
        param_loader.loadParam("publish_visualization", _publish_visualization_, bool(false));
        param_loader.loadParam("visualization_rate", _visualization_rate_, float(2.0));

        param_loader.loadParam("accumulator_length", _accumulator_length_, int(23));
        param_loader.loadParam("pitch_steps", _pitch_steps_, int(16));
        param_loader.loadParam("yaw_steps", _yaw_steps_, int(8));
        param_loader.loadParam("max_pixel_shift", _max_pixel_shift_, int(1));
        param_loader.loadParam("reasonable_radius", _reasonable_radius_, int(6));
        param_loader.loadParam("nullify_radius", _nullify_radius_, int(5));

        int _proces_rate;
        param_loader.loadParam("blink_process_rate", _proces_rate, int(10));

        std::vector<std::string> _points_seen_topics;
        param_loader.loadParam("points_seen_topics", _points_seen_topics, _points_seen_topics);
        if (_points_seen_topics.empty()) {
          ROS_WARN("[UVDARBlinkProcessor]: No topics of points_seen_topics were supplied. Returning.");
          return;
        }

        // Create callbacks for each camera //{
        for (size_t i = 0; i < _points_seen_topics.size(); ++i) {

          // Subscribe to corresponding topics
          points_seen_callback_t callback = [image_index=i,this] (const mrs_msgs::ImagePointsWithFloatStampedConstPtr& pointsMessage) { 
            InsertPoints(pointsMessage, image_index);
          };
          cals_points_seen_.push_back(callback);
          sub_points_seen_.push_back(nh_.subscribe(_points_seen_topics[i], 1, cals_points_seen_[i]));

          points_seen_callback_t sun_callback = [image_index=i,this] (const mrs_msgs::ImagePointsWithFloatStampedConstPtr& sunPointsMessage) { 
            InsertSunPoints(sunPointsMessage, image_index);
          };
          cals_sun_points_.push_back(callback);
          sub_sun_points_.push_back(nh_.subscribe(_points_seen_topics[i]+"/sun", 1, cals_sun_points_[i]));
        }
        //}

        parseSequenceFile("/home/viktor/mrs_workspace/src/uav_modules/ros_packages/uvdar_meta/uvdar_core/config/BlinkingSequence-8-3-3-2-8.txt");


        // Prepare data structures //{
        sun_points_.resize(_points_seen_topics.size());
        for (size_t i = 0; i < _points_seen_topics.size(); ++i) {
          ht4dbt_trackers_.push_back(
                std::make_shared<HT4DBlinkerTracker>(
                  _accumulator_length_, _pitch_steps_, _yaw_steps_, _max_pixel_shift_, cv::Size(0, 0), _nullify_radius_, _reasonable_radius_
                )
              );
          ht4dbt_trackers_.back()->setDebug(_debug_, _visual_debug_);
          ht4dbt_trackers_.back()->setSequences(_sequences_);

          blink_data_.push_back(BlinkData());

          mutex_camera_image_.push_back(std::make_shared<std::mutex>());
          camera_image_sizes_.push_back(cv::Size(-1,-1));
        }
        //}


        std::vector<std::string> _blinkers_seen_topics;
        std::vector<std::string> _estimated_framerate_topics;
        nh_.param("blinkers_seen_topics", _blinkers_seen_topics, _blinkers_seen_topics);
        nh_.param("estimated_framerate_topics", _estimated_framerate_topics, _estimated_framerate_topics);

        if (_blinkers_seen_topics.size() != _points_seen_topics.size()) {
          ROS_ERROR_STREAM("[UVDARBlinkProcessor] The number of poinsSeenTopics (" << _points_seen_topics.size() 
              << ") is not matching the number of blinkers_seen_topics (" << _blinkers_seen_topics.size() << ")!");
          return;
        }
        if (_estimated_framerate_topics.size() != _points_seen_topics.size()) {
          ROS_ERROR_STREAM("[UVDARBlinkProcessor] The number of poinsSeenTopics (" << _points_seen_topics.size() 
              << ") is not matching the number of blinkers_seen_topics (" << _estimated_framerate_topics.size() << ")!");
          return;
        }

        for (size_t i = 0; i < _blinkers_seen_topics.size(); ++i) {
          pub_blinkers_seen_.push_back(nh_.advertise<mrs_msgs::ImagePointsWithFloatStamped>(_blinkers_seen_topics[i], 1));
          pub_estimated_framerate_.push_back(nh_.advertise<std_msgs::Float32>(_estimated_framerate_topics[i], 1));
        }

        //}
        
        if (_gui_ || _publish_visualization_){ //frequency classifier is used here only for visualization purposes
          // load the frequencies
          param_loader.loadParam("frequencies", _frequencies_);
          if (_frequencies_.empty()){
            std::vector<double> default_frequency_set{5, 6, 8, 10, 15, 30};
            ROS_WARN("[UVDARBlinkProcessor]: No frequencies were supplied, using the default frequency set. This set is as follows: ");
            for (auto f : default_frequency_set){
              ROS_WARN_STREAM("[UVDARBlinkProcessor]: " << f << " hz");
            }
            _frequencies_ = default_frequency_set;
          }
          ufc_ = std::make_unique<UVDARFrequencyClassifier>(_frequencies_);


          current_visualization_done_ = false;
          timer_visualization_ = nh_.createTimer(ros::Rate(_visualization_rate_), &UVDARBlinkProcessor::VisualizationThread, this, false);

          nh_.param("use_camera_for_visualization", _use_camera_for_visualization_, bool(true));
          if (_use_camera_for_visualization_){
            /* subscribe to cameras //{ */

            std::vector<std::string> _camera_topics;
            nh_.param("camera_topics", _camera_topics, _camera_topics);
            if (_camera_topics.empty()) {
              ROS_WARN("[UVDARBlinkProcessor]: No topics of cameras were supplied");
              _use_camera_for_visualization_ = false;
            }
            else {
              images_current_.resize(_camera_topics.size());
              // Create callbacks for each camera
              for (size_t i = 0; i < _camera_topics.size(); ++i) {
                current_images_received_.push_back(false);
                image_callback_t callback = [image_index=i,this] (const sensor_msgs::ImageConstPtr& image_msg) { 
                  callbackImage(image_msg, image_index);
                };
                cals_image_.push_back(callback);
                // Subscribe to corresponding topics
                sub_image_.push_back(nh_.subscribe(_camera_topics[i], 1, cals_image_[i]));
              }
            }
            //}
          }

          if (_publish_visualization_){
            pub_visualization_ = std::make_unique<mrs_lib::ImagePublisher>(boost::make_shared<ros::NodeHandle>(nh_));
          }
        }

        for (size_t i = 0; i < _points_seen_topics.size(); ++i) {
          timer_process_.push_back(nh_.createTimer(ros::Duration(1.0/(double)(_proces_rate)), boost::bind(&UVDARBlinkProcessor::ProcessThread, this, _1, i), false, true));
        }

        initialized_ = true;
        ROS_INFO("[UVDARBlinkProcessor]: initialized");
  }

  //}


  private:

  /**
   * @brief Callback to insert new image points to the accumulator of the HT4D process
   *
   * @param msg The input message with image points
   * @param Image_index index of the camera image producing this message
   */
  /* InsertPoints //{ */
  void InsertPoints(const mrs_msgs::ImagePointsWithFloatStampedConstPtr& msg, size_t image_index) {
    if (!initialized_) return;


    blink_data_[image_index].sample_count++;

    if ((blink_data_[image_index].sample_count % 10) == 0) { //update the estimate of frequency every 10 samples
      blink_data_[image_index].framerate_estimate = 10000000000.0 / (double)((msg->stamp - blink_data_[image_index].last_sample_time).toNSec());
      if (_debug_){
        ROS_INFO_STREAM("Updating frequency: " << blink_data_[image_index].framerate_estimate << " Hz");
      }
      blink_data_[image_index].last_sample_time = msg->stamp;
      ht4dbt_trackers_[image_index]->updateFramerate(blink_data_[image_index].framerate_estimate);
      blink_data_[image_index].sample_count = 0;
    }

    if (_debug_) {
      ROS_INFO_STREAM("Received contours: " << msg->points.size());
    }

    std::vector<cv::Point2i> points;
    for (auto& point : msg->points) {
      points.push_back(cv::Point2d(point.x, point.y));
    }
    ht4dbt_trackers_[image_index]->insertFrame(points);

    if ((!_use_camera_for_visualization_) || ((!_gui_) && (!_publish_visualization_))){
      if ( (camera_image_sizes_[image_index].width <= 0 ) || (camera_image_sizes_[image_index].width <= 0 )){
        camera_image_sizes_[image_index].width = msg->image_width;
        camera_image_sizes_[image_index].height = msg->image_height;
        ht4dbt_trackers_[image_index]->updateResolution(cv::Size(msg->image_width, msg->image_height));
      }
    }
  }

  //}


  /**
   * @brief Callback to insert points corresponding to pixels with sun to local variable for visualization
   *
   * @param msg The input message with image points
   * @param image_index Index of the camera producing the image
   */
  /* InsertSunPoints //{ */
  void InsertSunPoints(const mrs_msgs::ImagePointsWithFloatStampedConstPtr& msg, size_t image_index) {
    if (!initialized_) return;

    int                      countSeen;
    std::vector<cv::Point2i> points;

    for (auto& point : msg->points) {
      points.push_back(cv::Point2d(point.x, point.y));
    }

    {
      std::scoped_lock lock(mutex_sun);
      sun_points_[image_index] = points;
    }
  }

  //}


  /**
   * @brief Thread function that processes the accumulated image points and periodically retrieves estimated origin points and frequency estimates of blinking markers
   *
   * @param te TimerEvent for the timer spinning this thread
   * @param Image_index Index of the camera producing the image
   */
  /* ProcessThread //{ */
  void ProcessThread([[maybe_unused]] const ros::TimerEvent& te, size_t image_index) {
      if (!initialized_){
        return;
      }

      mrs_msgs::ImagePointsWithFloatStamped msg;

      if (_debug_){
        ROS_INFO("Processing accumulated points.");
      }

      ros::Time local_last_sample_time = blink_data_[image_index].last_sample_time;
      {
        std::scoped_lock lock(*(blink_data_[image_index].mutex_retrieved_blinkers));
        blink_data_[image_index].retrieved_blinkers = ht4dbt_trackers_[image_index]->getResults();
        blink_data_[image_index].pitch              = ht4dbt_trackers_[image_index]->getPitch();
        blink_data_[image_index].yaw                = ht4dbt_trackers_[image_index]->getYaw();
      }

      msg.stamp         = local_last_sample_time;
      msg.image_width   = camera_image_sizes_[image_index].width;
      msg.image_height  = camera_image_sizes_[image_index].height;

      for (auto& blinker : blink_data_[image_index].retrieved_blinkers) {
        mrs_msgs::Point2DWithFloat point;
        point.x     = blinker.first.x;
        point.y     = blinker.first.y;
        point.value = blinker.second;
        msg.points.push_back(point);
      }

      pub_blinkers_seen_[image_index].publish(msg);

      std_msgs::Float32 msgFramerate;
      msgFramerate.data = blink_data_[image_index].framerate_estimate;
      pub_estimated_framerate_[image_index].publish(msgFramerate);

      current_visualization_done_ = false;

  }

  //}


  /**
   * @brief Thread function for optional visualization of the detected blinking markers
   *
   * @param te TimerEvent for the timer spinning this thread
   */
  /* VisualizationThread() //{ */
  void VisualizationThread([[maybe_unused]] const ros::TimerEvent& te) {
    if (initialized_){
      if(generateVisualization(image_visualization_) >= 0){
        if ((image_visualization_.cols != 0) && (image_visualization_.rows != 0)){
          if (_publish_visualization_){
            pub_visualization_->publish("uvdar_blink_visualization", 0.01, image_visualization_, true);
          }
          if (_gui_){
            cv::imshow("ocv_uvdar_blink_" + _uav_name_, image_visualization_);
            cv::waitKey(25);
          }

          if (_visual_debug_){
            cv::Mat image_visual_debug_;
            image_visual_debug_ = ht4dbt_trackers_[0]->getVisualization();
            if ((image_visual_debug_.cols > 0) && (image_visual_debug_.rows > 0)){
              cv::imshow("ocv_uvdar_hough_space_" + _uav_name_, image_visual_debug_);
            }
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
    if (_use_camera_for_visualization_ && !images_received_)
      return -2;

    if (current_visualization_done_)
      return 1;

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
      return -3;
    }

    output_image = cv::Mat(cv::Size(sum_image_width+((int)(camera_image_sizes_.size())-1), max_image_height),CV_8UC3);
    output_image = cv::Scalar(255, 255, 255);

    if ( (output_image.cols <= 0) || (output_image.rows <= 0) ){
      return -1;
    }

    int image_index = 0;
    for ([[maybe_unused]] auto curr_size : camera_image_sizes_){
      std::scoped_lock lock(*(mutex_camera_image_[image_index]));
      cv::Point start_point = cv::Point(start_widths[image_index]+image_index, 0);
      if (_use_camera_for_visualization_){
        if (current_images_received_[image_index]){
          cv::Mat image_rgb;
          ROS_INFO_STREAM("[UVDARBlinkProcessor]: Channel count: " << images_current_[image_index].channels());
          cv::cvtColor(images_current_[image_index], image_rgb, cv::COLOR_GRAY2BGR);
          image_rgb.copyTo(output_image(cv::Rect(start_point.x,0,images_current_[image_index].cols,images_current_[image_index].rows)));
        }
      }
      else {
        output_image(cv::Rect(start_point.x,0,camera_image_sizes_[image_index].width,camera_image_sizes_[image_index].height)) = cv::Scalar(0,0,0);
      }

      {
        std::scoped_lock lock(*(blink_data_[image_index].mutex_retrieved_blinkers));
        for (int j = 0; j < (int)(blink_data_[image_index].retrieved_blinkers.size()); j++) {
          cv::Point center =
            cv::Point(
                blink_data_[image_index].retrieved_blinkers[j].first.x, 
                blink_data_[image_index].retrieved_blinkers[j].first.y 
                )
            + start_point;
          int signal_index = blink_data_[image_index].retrieved_blinkers[j].second;
          if (signal_index >= 0) {
            std::string signal_text = std::to_string(std::max((int)blink_data_[image_index].retrieved_blinkers[j].second, 0));
            cv::putText(output_image, cv::String(signal_text.c_str()), center + cv::Point(-5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
            cv::Scalar color = ColorSelector::markerColor(signal_index);
            cv::circle(output_image, center, 5, color);
            double yaw, pitch, len;
            yaw              = blink_data_[image_index].yaw[j];
            pitch            = blink_data_[image_index].pitch[j];
            len              = cos(pitch);
            cv::Point target = center - (cv::Point(len * cos(yaw) * 20, len * sin(yaw) * 20.0));
            cv::line(output_image, center, target, cv::Scalar(0, 0, 255), 2);
          }
          else {
            cv::circle(output_image, center, 2, cv::Scalar(160,160,160));
            /* output_image.at<cv::Vec3b>(center) = cv::Vec3b(255,255,255); */
          }
        }
      }
      for (int j = 0; j < (int)(sun_points_[image_index].size()); j++) {
        cv::Point sun_current = sun_points_[image_index][j]+start_point;
        cv::circle(output_image, sun_current, 10, cv::Scalar(0,0,255));
        cv::circle(output_image, sun_current, 2,  cv::Scalar(0,0,255));
        cv::putText(output_image, cv::String("Sun"), sun_current + cv::Point(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
      }


      image_index++;
    }

    // draw the legend
    for (int i = 0; i < (int)(_frequencies_.size()); ++i) {
      cv::Scalar color = ColorSelector::markerColor(i);
      cv::circle(output_image, cv::Point(10, 10 + 15 * i), 5, color);
      cv::putText(output_image, cv::String(std::to_string((int)(_frequencies_[i]))), cv::Point(15, 15 + 15 * i), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
    }

    current_visualization_done_ = true;
    return 0;
  }
  //}


  /**
   * @brief Updates the latest camera image for background in the optional visualization
   *
   * @param image_msg The input image message
   * @param image_index Index of the camera image producing this message
   */
  /* callbackImage //{ */
  void callbackImage(const sensor_msgs::ImageConstPtr& image_msg, size_t image_index) {
    cv_bridge::CvImageConstPtr image;
    image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
    {
      std::scoped_lock lock(*(mutex_camera_image_[image_index]));
      images_current_[image_index] = image->image; 
      current_images_received_[image_index] = true;
      current_visualization_done_ = false;
    }
    if ( (camera_image_sizes_[image_index].width <= 0 ) || (camera_image_sizes_[image_index].width <= 0 )){
      camera_image_sizes_[image_index] = image->image.size();
      ht4dbt_trackers_[image_index]->updateResolution(image->image.size());
    }

    images_received_ = true;
  }
  //}

  /**
   * @brief Loads the file with lines describing useful blinking singals
   *
   * @param sequence_file The input file name
   *
   * @return Success status
   */
  /* parseSequenceFile //{ */
  bool parseSequenceFile(std::string sequence_file){
    ROS_WARN_STREAM("[UVDARBlinkProcessor]: Add sanitation - sequences must be of equal, non-zero length");
    ROS_INFO_STREAM("[UVDARBlinkProcessor]: Loading sequence from file: [ " + sequence_file + " ]");
    std::ifstream ifs;
    ifs.open(sequence_file);
    std::string word;
    std::string line;

    std::vector<std::vector<bool>> sequences;
    if (ifs.good()) {
      ROS_INFO("[UVDARBlinkProcessor]: Loaded Sequences: [: ");
      while (getline( ifs, line )){
        if (line[0] == '#'){
          continue;
        }
        std::string show_string = "";
        std::vector<bool> sequence;
        std::stringstream iss(line); 
        std::string token;
        while(std::getline(iss, token, ',')) {
          sequence.push_back(token=="1");
          if (sequence.back()){
            show_string += "1,";
          }
          else {
            show_string += "0,";
          }
        }
        sequences.push_back(sequence);
        ROS_INFO_STREAM("[UVDARBlinkProcessor]:   [" << show_string << "]");
      }
      ROS_INFO("[UVDARBlinkProcessor]: ]");
      ifs.close();

      _sequences_ = sequences;
    }
    else {
      ROS_ERROR_STREAM("[UVDARBlinkProcessor]: Failed to load sequence file " << sequence_file << "! Returning.");
      ifs.close();
      return false;
    }
    return true;
  }
  //}

  /* attributes //{ */
  std::atomic_bool initialized_ = false;
  std::atomic_bool current_visualization_done_ = false;
  bool images_received_ = false;

  std::string              _uav_name_;
  bool                     _debug_;
  std::vector<cv::Mat>     images_current_;
  std::vector<bool>        current_images_received_;

  std::vector<ros::Timer> timer_process_;
  std::vector<std::shared_ptr<std::mutex>>  mutex_camera_image_;
  bool                     _use_camera_for_visualization_;
  using image_callback_t = boost::function<void (const sensor_msgs::ImageConstPtr&)>;
  std::vector<image_callback_t> cals_image_;
  std::vector<ros::Subscriber> sub_image_;

  using points_seen_callback_t = boost::function<void (const mrs_msgs::ImagePointsWithFloatStampedConstPtr&)>;
  std::vector<points_seen_callback_t> cals_points_seen_;
  std::vector<points_seen_callback_t> cals_sun_points_;
  std::vector<ros::Subscriber> sub_points_seen_;
  std::vector<ros::Subscriber> sub_sun_points_;

  std::vector<ros::Publisher> pub_blinkers_seen_;
  std::vector<ros::Publisher> pub_estimated_framerate_;

  ros::Timer timer_visualization_;
  bool _visual_debug_;
  bool _gui_;
  bool _publish_visualization_;
  float _visualization_rate_;
  cv::Mat image_visualization_;
  std::vector<cv::Size> camera_image_sizes_;
  std::unique_ptr<mrs_lib::ImagePublisher> pub_visualization_;

  struct BlinkData {
    std::vector<std::pair<cv::Point2d,int>>      retrieved_blinkers;
    std::vector<double>           pitch;
    std::vector<double>           yaw;
    std::shared_ptr<std::mutex>   mutex_retrieved_blinkers;
    ros::Time                     last_sample_time;
    unsigned int                  sample_count = 0;
    double                        framerate_estimate = 72;

    BlinkData(){mutex_retrieved_blinkers = std::make_shared<std::mutex>();}
    ~BlinkData(){mutex_retrieved_blinkers.reset();}
  };

  std::vector<BlinkData> blink_data_;
  std::vector<std::shared_ptr<HT4DBlinkerTracker>> ht4dbt_trackers_;

  std::vector<std::vector<cv::Point>> sun_points_;
  std::mutex mutex_sun;


  std::vector<std::vector<bool>> _sequences_;

  std::vector<double> _frequencies_;
  std::unique_ptr<UVDARFrequencyClassifier> ufc_;

  int _accumulator_length_;
  int _pitch_steps_;
  int _yaw_steps_;
  int _max_pixel_shift_;
  int _reasonable_radius_;
  int _nullify_radius_;



  //}
};

} //namsepace uvdar

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uvdar::UVDARBlinkProcessor, nodelet::Nodelet)
