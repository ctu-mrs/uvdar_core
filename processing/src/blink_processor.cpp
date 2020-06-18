#undef _DEBUG

#define camera_delay 0.50

#define min_frequency 3
#define max_frequency 36.0
#define boundary_ratio 0.7

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <ht3dbt/ht4d.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
/* #include <std_msgs/MultiArrayDimension.h> */
/* #include <std_msgs/UInt32MultiArray.h> */
#include <mrs_msgs/ImagePointsWithFloatStamped.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/image_publisher.h>
#include <stdint.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <atomic>

#include <frequency_classifier/frequency_classifier.h>

namespace enc = sensor_msgs::image_encodings;

namespace uvdar {

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
    param_loader.loadParam("visualization_rate", _visualization_rate_, double(2.0));

    param_loader.loadParam("accumulator_length", _accumulator_length_, int(23));
    param_loader.loadParam("pitch_steps", _pitch_steps_, int(16));
    param_loader.loadParam("yaw_steps", _yaw_steps_, int(8));
    param_loader.loadParam("max_pixel_shift", _max_pixel_shift_, int(1));
    param_loader.loadParam("reasonable_radius", _reasonable_radius_, int(3));
    param_loader.loadParam("nullify_radius", _nullify_radius_, int(5));
    param_loader.loadParam("blink_process_rate", _proces_rate_, int(10));

    std::vector<std::string> _points_seen_topics;
    param_loader.loadParam("points_seen_topics", _points_seen_topics, _points_seen_topics);
    if (_points_seen_topics.empty()) {
      ROS_WARN("[UVDARBlinkProcessor]: No topics of points_seen_topics were supplied");
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
    

    // Prepare data structures //{
    sun_points_.resize(_points_seen_topics.size());
    for (size_t i = 0; i < _points_seen_topics.size(); ++i) {
      ht3dbt_trackers.push_back(
          new HT4DBlinkerTracker(_accumulator_length_, _pitch_steps_, _yaw_steps_, _max_pixel_shift_, cv::Size(752, 480), _nullify_radius_, _reasonable_radius_));
      ht3dbt_trackers.back()->setDebug(_debug_, _visual_debug_);

      process_spin_rate_.push_back(new ros::Rate((double)_proces_rate_));

      blink_data_.push_back(BlinkData());
      
      mutex_camera_image_.push_back(std::make_unique<std::mutex>());
    }
    //}



    std::vector<std::string> _blinkers_seen_topics;
    std::vector<std::string> _estimated_framerate_topics;
    nh_.param("blinkersSeenTopics", _blinkers_seen_topics, _blinkers_seen_topics);
    nh_.param("estimatedFramerateTopics", _estimated_framerate_topics, _estimated_framerate_topics);

    if (_blinkers_seen_topics.size() != _points_seen_topics.size()) {
      ROS_ERROR_STREAM("[UVDARBlinkProcessor] The number of poinsSeenTopics (" << _points_seen_topics.size() 
          << ") is not matching the number of blinkers_seen_topics (" << _blinkers_seen_topics.size() << ")!");
    }
    if (_estimated_framerate_topics.size() != _points_seen_topics.size()) {
      ROS_ERROR_STREAM("[UVDARBlinkProcessor] The number of poinsSeenTopics (" << _points_seen_topics.size() 
          << ") is not matching the number of blinkers_seen_topics (" << _estimated_framerate_topics.size() << ")!");
    }

    for (size_t i = 0; i < _blinkers_seen_topics.size(); ++i) {
      pub_blinkers_seen_.push_back(nh_.advertise<mrs_msgs::ImagePointsWithFloatStamped>(_blinkers_seen_topics[i], 1));
    }
    for (size_t i = 0; i < _estimated_framerate_topics.size(); ++i) {
      pub_estimated_framerate_.push_back(nh_.advertise<std_msgs::Float32>(_estimated_framerate_topics[i], 1));
    }

      //}

    nh_.param("beacon",_beacon_,bool(false));

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

      for (auto camera : _points_seen_topics){
        camera_image_sizes_.push_back(cv::Size(0,0));
      }

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
              ProcessRaw(image_msg, image_index);
            };
            cals_image_.push_back(callback);
            // Subscribe to corresponding topics
            sub_image_.push_back(nh_.subscribe(_camera_topics[i], 1, cals_image_[i]));
          }
        }

      }

      if (_publish_visualization_){
        pub_visualization_ = std::make_unique<mrs_lib::ImagePublisher>(boost::make_shared<ros::NodeHandle>(nh_));
      }
    }

    for (size_t i = 0; i < _points_seen_topics.size(); ++i) {
      process_threads.emplace_back(&UVDARBlinkProcessor::ProcessThread, this, i);
    }

    /* if (_gui_ || _visual_debug_) { */
    /*   show_thread  = std::thread(&UVDARBlinkProcessor::ShowThread, this); */
    /* } */

    /* if (_publish_visualization_) { */
    /*   image_transport::ImageTransport it(nh_); */
    /*   imPub = it.advertise("visualization", 1); */
    /*   visualization_thread  = std::thread(&UVDARBlinkProcessor::VisualizeThread, this); */
    /* } */

    initialized_ = true;
    ROS_INFO("[UVDARBlinkProcessor]: initialized");
  }

  //}


private:

  /* InsertPoints //{ */

  void InsertPoints(const mrs_msgs::ImagePointsWithFloatStampedConstPtr& msg, size_t image_index) {
    if (!initialized_) return;

    std::vector<cv::Point2i> points;

    /* BlinkData& data = blink_data_[image_index]; */
    /* auto* ht3dbt = ht3dbt_trackers[image_index]; */

    blink_data_[image_index].timeSamples++;

    if (blink_data_[image_index].timeSamples >= 10) {
      ros::Time nowTime = ros::Time::now();

      blink_data_[image_index].framerateEstim = 10000000000.0 / (double)((nowTime - blink_data_[image_index].lastSignal).toNSec());
      if (_debug_)
        std::cout << "Updating frequency: " << blink_data_[image_index].framerateEstim << " Hz" << std::endl;
      blink_data_[image_index].lastSignal = nowTime;
      ht3dbt_trackers[image_index]->updateFramerate(blink_data_[image_index].framerateEstim);
      blink_data_[image_index].timeSamples = 0;
    }

    if (_debug_) {
      ROS_INFO_STREAM("Received contours: " << msg->points.size());
    }
    if (msg->points.size() < 1) {
      blink_data_[image_index].foundTarget = false;
    } else {
      blink_data_[image_index].foundTarget = true;
      blink_data_[image_index].lastSeen    = ros::Time::now();
    }

    for (auto& point : msg->points) {
      points.push_back(cv::Point2d(point.x, point.y));
    }

    if (!_use_camera_for_visualization_){
      camera_image_sizes_[image_index].width = msg->image_width;
      camera_image_sizes_[image_index].height = msg->image_height;
    }

    {
      lastPointsTime = msg->stamp;
      ht3dbt_trackers[image_index]->insertFrame(points);
    }
  }

  //}

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

  /* ProcessThread //{ */

  void ProcessThread(size_t image_index) {
    std::vector<int>  msgdata;
    mrs_msgs::ImagePointsWithFloatStamped msg;
    clock_t                    begin, end;
    double                     elapsedTime;

    while (!initialized_) 
      process_spin_rate_[image_index]->sleep();

    auto* ht3dbt = ht3dbt_trackers[image_index];
    /* auto& retrievedBlinkers = blink_data_[image_index].retrievedBlinkers; */

    process_spin_rate_[image_index]->reset();
    while (ros::ok()) {
      /* if (ht3dbt->isCurrentBatchProcessed()){ */
      /*   /1* if (_debug_) *1/ */
      /*     ROS_INFO("Skipping batch, already processed."); */
      /*   continue; */
      /*   /1* ros::Duration(10).sleep(); *1/ */
      /* } */

      if (_debug_)
        ROS_INFO("Processing accumulated points.");

      begin = std::clock();
      ros::Time local_lastPointsTime = lastPointsTime;

      {
        /* std::scoped_lock lock(*(blink_data_[image_index].retrievedBlinkersMutex)); */
        blink_data_[image_index].retrievedBlinkers = ht3dbt_trackers[image_index]->getResults();
        blink_data_[image_index].pitch = ht3dbt_trackers[image_index]->getPitch();
        blink_data_[image_index].yaw = ht3dbt_trackers[image_index]->getYaw();
      }
      end         = std::clock();
      elapsedTime = double(end - begin) / CLOCKS_PER_SEC;
      if (_debug_)
        std::cout << "Processing: " << elapsedTime << " s, " << 1.0 / elapsedTime << " Hz" << std::endl;

      msg.stamp = local_lastPointsTime;
      msg.image_width =  camera_image_sizes_[image_index].width;
      msg.image_height = camera_image_sizes_[image_index].height;
      msg.points.clear();
      for (auto& blinker : blink_data_[image_index].retrievedBlinkers) {
        mrs_msgs::Point2DWithFloat point;
        point.x     = blinker.x;
        point.y     = blinker.y;
        point.value = blinker.z;
        msg.points.push_back(point);
      }
      pub_blinkers_seen_[image_index].publish(msg);

      std_msgs::Float32 msgFramerate;
      msgFramerate.data = blink_data_[image_index].framerateEstim;
      pub_estimated_framerate_[image_index].publish(msgFramerate);

      process_spin_rate_[image_index]->sleep();

      /* if (!_use_camera_for_visualization_) */
        current_visualization_done_ = false;
    }
  }

  //}


  /* color selector functions //{ */

  cv::Scalar rainbow(double value, double max) {
    unsigned char r, g, b;

    //rainbow gradient
    double fraction = value / max;
    r = 255 * (fraction < 0.25 ? 1 : fraction > 0.5 ? 0 : 2 - fraction * 4);
    g = 255 * (fraction < 0.25 ? fraction * 4 : fraction < 0.75 ? 1 : 4 - fraction * 4);
    b = 255 * (fraction < 0.5 ? 0 : fraction < 0.75 ? fraction * 4 - 2 : 1);

    return cv::Scalar(b, g, r);
  }

  cv::Scalar marker_color(int index, double max=14.0){
    if (index < 7){
      cv::Scalar selected;
    //MATLAB colors
    switch(index){
      case 0: selected = cv::Scalar(0.7410,        0.4470,   0);
              break;
      case 1: selected = cv::Scalar(0.0980,   0.3250,   0.8500);
              break;
      case 2: selected = cv::Scalar(0.1250,   0.6940,   0.9290);
              break;
      case 3: selected = cv::Scalar(0.5560,   0.1840,   0.4940);
              break;
      case 4: selected = cv::Scalar(0.1880,   0.6740,   0.4660);
              break;
      case 5: selected = cv::Scalar(0.9330,   0.7450,   0.3010);
              break;
      case 6: selected = cv::Scalar(0.1840,   0.0780,   0.6350);
        
    }
    return 255*selected;
    }
    else
      return rainbow((double)(index - 7),max); 
  }

  /* //} */

  /* /1* VisualizationThread() //{ *1/ */

  /* void VisualizationThread() { */
  /*   ROS_INFO("Visualize thread"); */

  /*   ros::Rate r(_visualization_rate_); */
  /*   sensor_msgs::ImagePtr msg; */
  /*   while (ros::ok()) { */
  /*     if (initialized_){ */
  /*       std::scoped_lock lock(mutex_show); */

  /*       if (GenerateVisualization() >= 0){ */

  /*         msg = cv_bridge::CvImage(std_msgs::Header(), enc::BGR8, image_view_).toImageMsg(); */
  /*         imPub.publish(msg); */
  /*       } */
  /*     } */
  /*     r.sleep(); */
  /*   } */
  /* } */

  /* //} */

  /* /1* ShowThread() //{ *1/ */

  /* void ShowThread() { */
  /*   cv::Mat temp; */

  /*   while (ros::ok()) { */
  /*     if (initialized_){ */
  /*       std::scoped_lock lock(mutex_show); */


  /*       if (_gui_){ */
  /*         if (GenerateVisualization() >= 0){ */
  /*           cv::imshow("ocv_blink_retrieval_" + _uav_name_, image_view_); */
  /*         } */
  /*       } */
  /*       if (_visual_debug_){ */
  /*         temp = ht3dbt_trackers[0]->getVisualization(); */
  /*         if (temp.cols>0) */
  /*           cv::imshow("ocv2_hough_space_" + _uav_name_, temp); */
  /*       } */

  /*     } */
  /*     cv::waitKey(1000.0 / 10.0); */
  /*   } */
  /* } */

  /* //} */

  /* VisualizationThread() //{ */
  void VisualizationThread([[maybe_unused]] const ros::TimerEvent& te) {
    if (initialized_){
      /* std::scoped_lock lock(mutex_visualization_); */
      if(GenerateVisualization(image_visualization_) >= 0){
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
            image_visual_debug_ = ht3dbt_trackers[0]->getVisualization();
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
  //
  /* GenerateVisualization() //{ */

    int GenerateVisualization(cv::Mat& output_image) {
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

    output_image = cv::Mat(cv::Size(sum_image_width+((int)(camera_image_sizes_.size())-1), max_image_height),CV_8UC3);
    output_image = cv::Scalar(255, 255, 255);

    if ( (output_image.cols == 0) || (output_image.rows == 0) ){
      return -1;
    }

    int image_index = 0;
    for ([[maybe_unused]] auto curr_size : camera_image_sizes_){
      std::scoped_lock lock(*(mutex_camera_image_[image_index]));
      cv::Point start_point = cv::Point(start_widths[image_index]+image_index, 0);
      if (_use_camera_for_visualization_){
        if (current_images_received_[image_index]){
          cv::Mat image_rgb;
          cv::cvtColor(images_current_[image_index], image_rgb, cv::COLOR_GRAY2BGR);
          image_rgb.copyTo(output_image(cv::Rect(start_point.x,0,images_current_[image_index].cols,images_current_[image_index].rows)));
        }
      }
      else {
          output_image(cv::Rect(start_point.x,0,camera_image_sizes_[image_index].width,camera_image_sizes_[image_index].height)) = cv::Scalar(0,0,0);
      }

      for (int j = 0; j < (int)(blink_data_[image_index].retrievedBlinkers.size()); j++) {
        cv::Point center =
          cv::Point(
              blink_data_[image_index].retrievedBlinkers[j].x, 
              blink_data_[image_index].retrievedBlinkers[j].y 
              )
          + start_point;
        int frequency_index = ufc_->findMatch(blink_data_[image_index].retrievedBlinkers[j].z);
        if (frequency_index >= 0) {
          std::string frequency_text = std::to_string(std::max((int)blink_data_[image_index].retrievedBlinkers[j].z, 0));
          cv::putText(output_image, cv::String(frequency_text.c_str()), center + cv::Point(-5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
          cv::Scalar color = marker_color(frequency_index);
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
      cv::Scalar color = marker_color(i);
      cv::circle(output_image, cv::Point(10, 10 + 15 * i), 5, color);
      cv::putText(output_image, cv::String(to_string_precision(_frequencies_[i],0)), cv::Point(15, 15 + 15 * i), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
    }

    current_visualization_done_ = true;
    return 0;
  }

  //}


    /* ProcessRaw //{ */
    void ProcessRaw(const sensor_msgs::ImageConstPtr& image_msg, size_t image_index) {
      cv_bridge::CvImageConstPtr image;
      image = cv_bridge::toCvShare(image_msg, enc::RGB8);
      {
      std::scoped_lock lock(*(mutex_camera_image_[image_index]));
        images_current_[image_index] = image->image; 
        current_images_received_[image_index] = true;
        current_visualization_done_ = false;
      }
      camera_image_sizes_[image_index] = image->image.size();

      images_received_ = true;
    }

  //}
  
  
  std::string to_string_precision(double input, unsigned int precision){
    std::string output = std::to_string(input);
    if (precision>=0){
      if (precision==0)
        return output.substr(0,output.find_first_of("."));
    }
    return "";
  }


  /* attributes //{ */

  std::atomic_bool initialized_ = false;
  std::atomic_bool current_visualization_done_ = false;
  std::atomic_bool images_received_ = false;

  std::string              _uav_name_;
  bool                     _debug_;
  bool                     _visual_debug_;
  bool                     _gui_;
  std::vector<cv::Mat>     images_current_;
  std::vector<bool>        current_images_received_;
  /* cv::Mat                  image_view_; */

  ros::Timer timer_visualization_;
  /* std::thread              show_thread; */
  /* std::thread              visualization_thread; */
  std::vector<std::thread> process_threads;
  std::vector<std::unique_ptr<std::mutex>>  mutex_camera_image_;

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
  //CHECK

  bool _publish_visualization_;
  float _visualization_rate_;
  cv::Mat image_visualization_;
  std::vector<cv::Size> camera_image_sizes_;
  std::unique_ptr<mrs_lib::ImagePublisher> pub_visualization_;

  struct BlinkData {
    bool                     foundTarget;
    /* int                      currTrackerCount; */
    std::vector<cv::Point3d> retrievedBlinkers;
    std::vector<double>      pitch;
    std::vector<double>      yaw;
    /* std::mutex*              retrievedBlinkersMutex; */
    ros::Time                lastSeen;
    ros::Time                lastSignal;
    int                      timeSamples = 0;
    double                   timeSum = 0;

    double framerateEstim = 72;

    /* BlinkData(): retrievedBlinkersMutex(new std::mutex{}) {}; */
    BlinkData(){};
    /* ~BlinkData() { delete retrievedBlinkersMutex; }; */
    ~BlinkData(){};
  };

  std::vector<BlinkData> blink_data_;
  std::vector<HT4DBlinkerTracker*> ht3dbt_trackers;

  std::vector<std::vector<cv::Point>> sun_points_;
  std::mutex mutex_sun;

  std::vector<double> _frequencies_;
  std::unique_ptr<UVDARFrequencyClassifier> ufc_;

  std::vector<ros::Rate*> process_spin_rate_;
  int        _proces_rate_;

  int _accumulator_length_;
  int _pitch_steps_;
  int _yaw_steps_;
  int _max_pixel_shift_;
  int _reasonable_radius_;
  int _nullify_radius_;

  bool _beacon_;

  ros::Time lastPointsTime;


  //}
};

/* int main(int argc, char** argv) { */
/*   ros::init(argc, argv, "blink_processor"); */
/*   ROS_INFO("Starting the Blink processor node"); */
/*   ros::NodeHandle nodeA; */
/*   UVDARBlinkProcessor  bp(nodeA); */

/*   ROS_INFO("Blink processor node initiated"); */

/*   ros::spin(); */
/*   /1* ros::MultiThreadedSpinner spinner(4); *1/ */
/*   /1* spinner.spin(); *1/ */
/*   /1* while (ros::ok()) *1/ */
/*   /1* { *1/ */
/*   /1*   ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.001)); *1/ */
/*   /1* } *1/ */
/*   return 0; */
/* } */

} //namsepace uvdar

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uvdar::UVDARBlinkProcessor, nodelet::Nodelet)
