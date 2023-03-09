#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/image_publisher.h>
#include <std_msgs/Float32.h>
#include <mrs_msgs/ImagePointsWithFloatStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <mutex>
#include <fstream>
#include <color_selector/color_selector.h>
#include <alternativeHT/alternativeHT.h>
#include <alternativeHT/imu_compensation.h>

namespace uvdar{

  class UVDAR_BP_Tim : public nodelet::Nodelet {
    public:

      UVDAR_BP_Tim(){};

      ~UVDAR_BP_Tim(){};
  

    private:

      virtual void onInit();
      void loadParams(const bool & );
      bool checkCameraTopicSizeMatch();
      bool parseSequenceFile(const std::string &);
      void initAlternativeHTDataStructure();
      void initIMUCompensation();

      void subscribeToPublishedPoints();
      void insertPointToAHT(const mrs_msgs::ImagePointsWithFloatStampedConstPtr &, const size_t &);
      void InsertSunPoints(const mrs_msgs::ImagePointsWithFloatStampedConstPtr&, size_t);


      // functions for visualization
      void initGUI(); 
      void ProcessThread([[maybe_unused]] const ros::TimerEvent&, size_t);
      void VisualizationThread([[maybe_unused]] const ros::TimerEvent&);
      int generateVisualization(cv::Mat& );
      void callbackImage(const sensor_msgs::ImageConstPtr&, size_t);

      ros::NodeHandle private_nh_;
      std::atomic_bool initialized_ = false;  
      
                  
      int image_sizes_received_ = 0;
      std::vector<std::shared_ptr<alternativeHT>> aht_;
      std::unique_ptr<IMU_COMPENSATION> imu_comp;
      
      std::vector<std::vector<bool>> sequences_;
      std::vector<ros::Publisher> pub_blinkers_seen_;
      std::vector<ros::Publisher> pub_estimated_framerate_;

      std::mutex mutex_signals_;
      
  
      using points_seen_callback_t = boost::function<void (const mrs_msgs::ImagePointsWithFloatStampedConstPtr&)>;
      std::vector<points_seen_callback_t> cals_points_seen_;
      std::vector<points_seen_callback_t> cals_sun_points_;
      std::vector<ros::Subscriber> sub_points_seen_;
      std::vector<ros::Subscriber> sub_sun_points_;
      ros::Subscriber sub_imu_;


      // visualization variables
      std::vector<ros::Timer> timer_process_;
      ros::Timer timer_visualization_;
      cv::VideoWriter videoWriter_;
      using image_callback_t = boost::function<void (const sensor_msgs::ImageConstPtr&)>;
      std::vector<image_callback_t> cals_image_;
      std::vector<ros::Subscriber> sub_image_;
      std::atomic_bool current_visualization_done_ = false;
      std::vector<cv::Mat> images_current_;
      cv::Mat image_visualization_;
      std::vector<bool> current_images_received_;
      std::vector<cv::Size> camera_image_sizes_;
      std::unique_ptr<mrs_lib::ImagePublisher> pub_visualization_;
      std::vector<std::shared_ptr<std::mutex>>  mutex_camera_image_;
      
      std::vector<std::string> points_seen_topics_imu_compensation_;
      std::vector<ros::Publisher> pub_points_seen_compensated_imu_;
      std::vector<points_seen_callback_t> cals_points_seen_imu_;
       std::vector<ros::Subscriber> subs_points_seen_imu_;
       std::vector<mrs_msgs::ImagePointsWithFloatStamped> vectPoints_;

      std::vector<std::vector<cv::Point>> sun_points_;
      std::mutex mutex_sun;
      
      
      // loaded Params
      std::string _uav_name_;   
      bool        _debug_;
      bool        _visual_debug_;
      bool        _gui_;
      bool        _publish_visualization_;
      float       _visualization_rate_;
      bool        _use_camera_for_visualization_;
      std::vector<std::string> _blinkers_seen_topics;
      std::vector<std::string> _estimated_framerate_topics;
      std::vector<std::string> _points_seen_topics;
      std::string _sequence_file;
      bool        _imu_compensation_;
      std::string _imu_topic_;
      float _decayFactor_; 
      int _polynomialDegree_;

      struct BlinkData {
      std::vector<std::pair<PointState,int>>      retrieved_blinkers;
      std::shared_ptr<std::mutex>   mutex_retrieved_blinkers;
      ros::Time                     last_sample_time;
      ros::Time                     last_sample_time_diagnostic;
      unsigned int                  sample_count = -1;
      double                        framerate_estimate = 72;

      BlinkData(){mutex_retrieved_blinkers = std::make_shared<std::mutex>();}
      ~BlinkData(){mutex_retrieved_blinkers.reset();}
    };

      // std::vector<std::vector<std::pair<PointState, int>>> blink_data;
    std::vector<BlinkData> blink_data_; 

  };
      
  void UVDAR_BP_Tim::onInit(){

    private_nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();
    NODELET_DEBUG("[UVDAR_BP_Tim]: Initializing Nodelet...");

    const bool printParams = false;

    loadParams(printParams);

    const bool match = checkCameraTopicSizeMatch();
    if(!match) return;

    parseSequenceFile(_sequence_file);
    initAlternativeHTDataStructure();
    if(_imu_compensation_){
      imu_comp = std::make_unique<IMU_COMPENSATION>(_imu_topic_);
    }
    std::string pubTopic_extension = "_imu_comp";
    points_seen_topics_imu_compensation_ = _points_seen_topics; 
    for(auto & topicName : points_seen_topics_imu_compensation_){
        topicName += pubTopic_extension;
        pub_points_seen_compensated_imu_.push_back(private_nh_.advertise<mrs_msgs::ImagePointsWithFloatStamped>(topicName, 1));
        mrs_msgs::ImagePointsWithFloatStamped p;
        vectPoints_.push_back(p);
    }

    subscribeToPublishedPoints();

    initGUI(); 

    initialized_ = true;
  }

  void UVDAR_BP_Tim::loadParams(const bool &printParams) {

    mrs_lib::ParamLoader param_loader(private_nh_, printParams, "UVDAR_BP_Tim");

    param_loader.loadParam("uav_name", _uav_name_, std::string());
    param_loader.loadParam("debug", _debug_, bool(false));
    param_loader.loadParam("visual_debug", _visual_debug_, bool(false));
    if(_visual_debug_){
          ROS_WARN_STREAM("[UVDAR_BP_Tim]: You are using visual debugging. This option is only meant for development. Activating it significantly increases load on the system and the user should do so with care.");
    }

    param_loader.loadParam("gui", _gui_, bool(true));                                     
    param_loader.loadParam("publish_visualization", _publish_visualization_, bool(true));
    param_loader.loadParam("visualization_rate", _visualization_rate_, float(2.0));       

    param_loader.loadParam("points_seen_topics", _points_seen_topics, _points_seen_topics);

    param_loader.loadParam("sequence_file", _sequence_file, std::string());

    private_nh_.param("blinkers_seen_topics", _blinkers_seen_topics, _blinkers_seen_topics);
    private_nh_.param("estimated_framerate_topics", _estimated_framerate_topics, _estimated_framerate_topics);
    private_nh_.param("use_camera_for_visualization", _use_camera_for_visualization_, bool(true));

    private_nh_.param("imu_topic", _imu_topic_, std::string());
    param_loader.loadParam("imu_compensation", _imu_compensation_, bool(false));

    param_loader.loadParam("decayFactor", _decayFactor_, float(1));
    param_loader.loadParam("polyOrder", _polynomialDegree_, int(2));

  }

  bool UVDAR_BP_Tim::checkCameraTopicSizeMatch() {

    for (size_t i = 0; i < _blinkers_seen_topics.size(); ++i) {
      pub_blinkers_seen_.push_back(private_nh_.advertise<mrs_msgs::ImagePointsWithFloatStamped>(_blinkers_seen_topics[i], 1));
      pub_estimated_framerate_.push_back(private_nh_.advertise<std_msgs::Float32>(_estimated_framerate_topics[i], 1));
    }

    if (_blinkers_seen_topics.size() != _points_seen_topics.size()){
      ROS_ERROR_STREAM("[UVDAR_BP_Tim]: The number of pointsSeenTopics (" << _points_seen_topics.size() << ") is not matching the number of blinkers_seen_topics (" << _blinkers_seen_topics.size() << ")!");
      return false;
    }
    if (_estimated_framerate_topics.size() != _points_seen_topics.size()){
      ROS_ERROR_STREAM("[UVDAR_BP_Tim]: The number of pointsSeenTopics (" << _points_seen_topics.size() << ") is not matching the number of blinkers_seen_topics (" << _estimated_framerate_topics.size() << ")!");
      return false;
    }
    return true;
  }

  bool UVDAR_BP_Tim::parseSequenceFile(const std::string &sequence_file) {

    ROS_WARN_STREAM("[UVDAR_BP_Tim]: Add sanitation - sequences must be of equal, non-zero length");
    ROS_INFO_STREAM("[UVDAR_BP_Tim]: Loading sequence from file: [ " + sequence_file + " ]");
    std::ifstream ifs;
    ifs.open(sequence_file);
    std::string word;
    std::string line;
    std::vector<std::vector<bool>> sequences;
    if (ifs.good()){
      ROS_INFO("[UVDAR_BP_Tim]: Loaded Sequences: [: ");
      while (getline(ifs, line)){
        if (line[0] == '#'){
          continue;
        }
        std::string show_string = "";
        std::vector<bool> sequence;
        std::stringstream iss(line);
        std::string token;
        while (std::getline(iss, token, ',')){
          if (true){
            sequence.push_back(token == "1");
          }else{
            // Manchester Coding - IEEE 802.3 Conversion: 1 = [0,1]; 0 = [1,0]
            if (token == "1"){
              sequence.push_back(false);
              sequence.push_back(true);
            }else{
              sequence.push_back(true);
              sequence.push_back(false);
            }
          }
        }

        for (const auto boolVal : sequence){
          if (boolVal)
            show_string += "1,";
          else
            show_string += "0,";
        }

        sequences.push_back(sequence);
        ROS_INFO_STREAM("[UVDAR_BP_Tim]: [" << show_string << "]");
      }
      ROS_INFO("[UVDAR_BP_Tim]: ]");
      ifs.close();
      sequences_ = sequences;
    }else{
      ROS_ERROR_STREAM("[UVDAR_BP_Tim]: Failed to load sequence file " << sequence_file << "! Returning.");
      ifs.close();
      return false;
    }
    return true;
  }

  void UVDAR_BP_Tim::initAlternativeHTDataStructure(){

    for (size_t i = 0; i < _points_seen_topics.size(); ++i) {
      aht_.push_back(std::make_shared<alternativeHT>(_decayFactor_, _polynomialDegree_));
      aht_[i]->setSequences(sequences_);
      aht_[i]->setDebugFlags(_debug_, _visual_debug_);

      // std::vector<std::pair<PointState, int>> pair;
      // signals_.push_back(pair);

      blink_data_.push_back(BlinkData());


      // sun_points_.resize(_points_seen_topics.size());

      mutex_camera_image_.push_back(std::make_shared<std::mutex>());
      camera_image_sizes_.push_back(cv::Size(-1,-1));
    }

  }

  void UVDAR_BP_Tim::subscribeToPublishedPoints() {

    for (size_t i = 0; i < _points_seen_topics.size(); ++i){

      if(_imu_compensation_){
        points_seen_callback_t callbackIMU = [image_index = i, this](const mrs_msgs::ImagePointsWithFloatStampedConstPtr &pointsMessage){
            imu_comp->compensateByIMUMovement(pointsMessage, image_index, pub_points_seen_compensated_imu_);
        };
        cals_points_seen_imu_.push_back(callbackIMU);
        subs_points_seen_imu_.push_back(private_nh_.subscribe(_points_seen_topics[i], 1, cals_points_seen_imu_[i]));
      }

      // Subscribe to corresponding topics
      points_seen_callback_t callback = [image_index = i, this](const mrs_msgs::ImagePointsWithFloatStampedConstPtr &pointsMessage){
              insertPointToAHT(pointsMessage, image_index);
      };
      cals_points_seen_.push_back(callback);
      if(_imu_compensation_){
        // sub_points_seen_.push_back(private_nh_.subscribe(_points_seen_topics[i], 1, cals_points_seen_[i]));
        sub_points_seen_.push_back(private_nh_.subscribe(points_seen_topics_imu_compensation_[i], 1, cals_points_seen_[i]));
      }else{
        sub_points_seen_.push_back(private_nh_.subscribe(_points_seen_topics[i], 1, cals_points_seen_[i]));
      }
      points_seen_callback_t sun_callback = [image_index=i,this] (const mrs_msgs::ImagePointsWithFloatStampedConstPtr& sunPointsMessage){
        InsertSunPoints(sunPointsMessage, image_index);
      };
      cals_sun_points_.push_back(callback);
      sub_sun_points_.push_back(private_nh_.subscribe(_points_seen_topics[i] + "/sun", 1, cals_sun_points_[i]));
    } 
    sun_points_.resize(_points_seen_topics.size());

    for (size_t i = 0; i < _blinkers_seen_topics.size(); ++i) {
          pub_blinkers_seen_.push_back(private_nh_.advertise<mrs_msgs::ImagePointsWithFloatStamped>(_blinkers_seen_topics[i], 1));
          pub_estimated_framerate_.push_back(private_nh_.advertise<std_msgs::Float32>(_estimated_framerate_topics[i], 1));
    }

  }

  void UVDAR_BP_Tim::initGUI(){
     if (_gui_ || _publish_visualization_){ 
        // video = cv::VideoWriter("BLA.avi",cv::VideoWriter::fourcc('M','J','P','G'),5, cv::Size(2258,480));

          // load the frequencies
        current_visualization_done_ = false;
        timer_visualization_ = private_nh_.createTimer(ros::Rate(_visualization_rate_), &UVDAR_BP_Tim::VisualizationThread, this, false);
        
        if (_use_camera_for_visualization_){
            /* subscribe to cameras //{ */

            std::vector<std::string> _camera_topics;
            private_nh_.param("camera_topics", _camera_topics, _camera_topics);
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
                sub_image_.push_back(private_nh_.subscribe(_camera_topics[i], 1, cals_image_[i]));
              }
            }
            //}
          }
      }
      for (size_t i = 0; i < _points_seen_topics.size(); ++i) {
        timer_process_.push_back(private_nh_.createTimer(ros::Duration(1.0/(double)(10)), boost::bind(&UVDAR_BP_Tim::ProcessThread, this, _1, i), false, true));
      }
  }


  void UVDAR_BP_Tim::insertPointToAHT(const mrs_msgs::ImagePointsWithFloatStampedConstPtr &ptsMsg, const size_t & img_index) {
    if (!initialized_) return;

    blink_data_[img_index].sample_count++;

    
    if ((blink_data_[img_index].sample_count % 10) == 0) { //update the estimate of frequency every 10 samples
      blink_data_[img_index].framerate_estimate = 10000000000.0 / (double)((ptsMsg->stamp - blink_data_[img_index].last_sample_time_diagnostic).toNSec());
      if (_debug_){
        ROS_INFO_STREAM("[UVDAR_BP_Tim]: Updating frequency: " << blink_data_[img_index].framerate_estimate << " Hz");
      }

      aht_[img_index]->updateFramerate(blink_data_[img_index].framerate_estimate);
      blink_data_[img_index].sample_count = 0;

      blink_data_[img_index].last_sample_time_diagnostic = ptsMsg->stamp;
    }

    if (_debug_)
        ROS_INFO_STREAM("[UVDAR_BP_Tim]: td: " << ptsMsg->stamp - blink_data_[img_index].last_sample_time);
    if (blink_data_[img_index].last_sample_time >= ptsMsg->stamp){
      ROS_ERROR_STREAM("[UVDAR_BP_Tim]: Points arrived out of order!: prev: "<< blink_data_[img_index].last_sample_time << "; curr: " << ptsMsg->stamp);
    }
    double dt = (ptsMsg->stamp - blink_data_[img_index].last_sample_time).toSec();
    if (dt > (1.5/(blink_data_[img_index].framerate_estimate)) ){
      int new_frame_count = (int)(dt*(blink_data_[img_index].framerate_estimate) + 0.5) - 1;
      ROS_ERROR_STREAM("[UVDAR_BP_Tim]: Missing frames! Inserting " << new_frame_count << " empty frames!"); 
      const mrs_msgs::ImagePointsWithFloatStampedConstPtr null_points;
      for (int i = 0; i < new_frame_count; i++){
        //TODO: DO SOMETHING
        // aht_[img_index]->processBuffer(null_points);
      }
    }
    blink_data_[img_index].last_sample_time = ptsMsg->stamp;

    if (_debug_) {
      ROS_INFO_STREAM("[UVDAR_BP_Tim]: Received contours: " << ptsMsg->points.size());
    }

    aht_[img_index]->processBuffer(ptsMsg);

    if ((!_use_camera_for_visualization_) || ((!_gui_) && (!_publish_visualization_))){
      if ( (camera_image_sizes_[img_index].width <= 0 ) || (camera_image_sizes_[img_index].width <= 0 )){
        camera_image_sizes_[img_index].width = ptsMsg->image_width;
        camera_image_sizes_[img_index].height = ptsMsg->image_height;
        if (image_sizes_received_ < (int)(camera_image_sizes_.size())){
          image_sizes_received_++; 

        }
      }
    }

    
    mrs_msgs::ImagePointsWithFloatStamped msg;
    ros::Time local_last_sample_time = blink_data_[img_index].last_sample_time;
    {
      std::scoped_lock lock(*(blink_data_[img_index].mutex_retrieved_blinkers));
      blink_data_[img_index].retrieved_blinkers = aht_[img_index]->getResults();
      
      for (auto& signal : blink_data_[img_index].retrieved_blinkers) {
        mrs_msgs::Point2DWithFloat point;
        point.x     = signal.first.point.x;
        point.y     = signal.first.point.y;
        if (signal.second <= (int)sequences_.size()){
          point.value = signal.second;
        }
        else {
          point.value = -2;
        }
        msg.points.push_back(point);
      }
    msg.stamp         = local_last_sample_time;
    msg.image_width   = camera_image_sizes_[img_index].width;
    msg.image_height  = camera_image_sizes_[img_index].height;
    
// TODO: pose estimator finds no appropriate hypothesis
    // pub_blinkers_seen_[img_index].publish(msg);

    std_msgs::Float32 msgFramerate;
    msgFramerate.data = blink_data_[img_index].framerate_estimate;
    pub_estimated_framerate_[img_index].publish(msgFramerate);

    }
    
  } 

  void UVDAR_BP_Tim::InsertSunPoints(const mrs_msgs::ImagePointsWithFloatStampedConstPtr& msg, size_t image_index) {
    if (!initialized_) return;

    std::vector<cv::Point2i> points;

    for (auto& point : msg->points) {
      points.push_back(cv::Point2d(point.x, point.y));
    }

    {
      std::scoped_lock lock(mutex_sun);
      sun_points_[image_index] = points;
    }
  }


  void UVDAR_BP_Tim::ProcessThread([[maybe_unused]] const ros::TimerEvent& te, [[maybe_unused]] size_t image_index) {
    if (!initialized_){
      return;
    }
    current_visualization_done_ = false;
  }
  void UVDAR_BP_Tim::VisualizationThread([[maybe_unused]] const ros::TimerEvent& te) {
    if(initialized_){
      if(generateVisualization(image_visualization_) >= 0){
        if((image_visualization_.cols != 0) && (image_visualization_.rows != 0)){
          if(_publish_visualization_){
            pub_visualization_->publish("uvdar_blink_visualization", 0.01, image_visualization_, true);
          }
          if (_gui_){
            cv::imshow("ocv_uvdar_blink_" + _uav_name_, image_visualization_);
            // videoWriter_.write(image_visualization_);
            cv::waitKey(25);
          }
        }
      }
    }
  }

  int UVDAR_BP_Tim::generateVisualization(cv::Mat & output_image) {
    if (image_sizes_received_<(int)(camera_image_sizes_.size())){
      return -2;
    }

    if (current_visualization_done_)
      return 1;

    int max_image_height = 0;
    int sum_image_width = 0;
    std::vector<int> start_widths;
    int i =0;
    for (auto curr_size : camera_image_sizes_){
      if ((curr_size.width < 0) || (curr_size.height < 0)){
        ROS_ERROR_STREAM("[UVDARBlinkProcessor]: Size of image " << i << " was not received! Returning.");
        return -4;
      }
      if (max_image_height < curr_size.height){
        max_image_height = curr_size.height;
      }
      start_widths.push_back(sum_image_width);
      sum_image_width += curr_size.width;
      i++;
    }

    if ( (sum_image_width <= 0) || (max_image_height <= 0) ){
      return -3;
    }

    output_image = cv::Mat(cv::Size(sum_image_width+((int)(camera_image_sizes_.size())-1), max_image_height),CV_8UC3);
    output_image = cv::Scalar(255, 255, 255);

    if((output_image.cols <= 0) || (output_image.rows <= 0)){
      return -1;
    }

    int image_index = 0;

    for ([[maybe_unused]] auto curr_size : camera_image_sizes_){
      std::scoped_lock lock(*(mutex_camera_image_[image_index]));
      cv::Point start_point = cv::Point(start_widths[image_index]+image_index, 0);
      if (_use_camera_for_visualization_){
        if (current_images_received_[image_index]){
          cv::Mat image_rgb;
          /* ROS_INFO_STREAM("[UVDARBlinkProcessor]: Channel count: " << images_current_[image_index].channels()); */
          cv::cvtColor(images_current_[image_index], image_rgb, cv::COLOR_GRAY2BGR);
          image_rgb.copyTo(output_image(cv::Rect(start_point.x,0,images_current_[image_index].cols,images_current_[image_index].rows)));
        }
      }else{
        output_image(cv::Rect(start_point.x,0,camera_image_sizes_[image_index].width,camera_image_sizes_[image_index].height)) = cv::Scalar(0,0,0);
      }
      
      {
      std::scoped_lock lock(*(blink_data_[image_index].mutex_retrieved_blinkers));

      for(int j = 0; j < (int)(blink_data_[image_index].retrieved_blinkers.size()); j++){
        // cv::Point center = cv::Point(signals_[image_index][j].point.first.x, signals_[image_index][j].first.y) + start_point;
        cv::Point center = cv::Point(blink_data_[image_index].retrieved_blinkers[j].first.point.x, blink_data_[image_index].retrieved_blinkers[j].first.point.y) + start_point;
        int signal_index = blink_data_[image_index].retrieved_blinkers[j].second;
        if(signal_index == -2 || signal_index == -3) {
          continue;
        }
        if(signal_index >= 0){
          std::string signal_text = std::to_string(std::max(signal_index, 0));
          cv::putText(output_image, cv::String(signal_text.c_str()), center + cv::Point(-5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
          cv::Scalar color = ColorSelector::markerColor(signal_index);
          cv::circle(output_image, center, 5, color);
        }else{
          cv::circle(output_image, center, 2, cv::Scalar(160,160,160));
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
    for (int i = 0; i < (int)sequences_.size(); ++i) {
      cv::Scalar color = ColorSelector::markerColor(i);
      cv::circle(output_image, cv::Point(10, 10 + 15 * i), 5, color);
      cv::putText(output_image, cv::String(std::to_string(i)), cv::Point(15, 15 + 15 * i), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
    }

    current_visualization_done_ = true;
    return 0;
  }


  void UVDAR_BP_Tim::callbackImage(const sensor_msgs::ImageConstPtr& image_msg, size_t image_index) {
    cv_bridge::CvImageConstPtr image;
    image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
    {
      std::scoped_lock lock(*(mutex_camera_image_[image_index]));
      images_current_[image_index] = image->image; 
      current_images_received_[image_index] = true;
      current_visualization_done_ = false;
    }
    if((camera_image_sizes_[image_index].width <= 0 ) || (camera_image_sizes_[image_index].width <= 0)){
      camera_image_sizes_[image_index] = image->image.size();
      if(image_sizes_received_ < (int)(camera_image_sizes_.size())){
        image_sizes_received_++;
      }
    }
  }


}// namespace uvdar

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uvdar::UVDAR_BP_Tim, nodelet::Nodelet);