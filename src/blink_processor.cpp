#undef Success
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <ht4dbt/ht4d_cpu.h>
/* #include <ht4dbt/ht4d_gpu.h> */
#include <ami/ami.h>
#include <color_selector/color_selector.h>
#include <uvdar_core/ImagePointsWithFloatStamped.h>
#include <std_msgs/Float32.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/image_publisher.h>
#include <uvdar_core/AMIDataForLogging.h>
#include <uvdar_core/AMISeqVariables.h>
#include <uvdar_core/AMIAllSequences.h>
#include <uvdar_core/AMISeqPoint.h>
#include <mutex>
#include <thread>
#include <atomic>
#include <fstream>

namespace uvdar{

  /**
   * @brief A nodelet for extracting blinking frequencies and image positions of intermittently appearing image points
   */
  class UVDARBlinkProcessor : public nodelet::Nodelet {
    public:

      UVDARBlinkProcessor(){};

      ~UVDARBlinkProcessor(){};
  

    private:
      /**
      * @brief Initializer - loads parameters and initializes necessary structures
      * 
      */
      virtual void onInit();

      /**
       * @brief load all dynamic params from launch file
       * 
       */
      void loadParams(const bool & );
      
      /**
       * @brief check values/sizes of the loaded params 
       * 
       * @return true = sucess; false = problem occured -> shutdown of nodelet 
       */
      bool checkLoadedParams();

      /**
      * @brief Loads the file with lines describing useful blinking singals
      *
      * @param sequence_file The input file name
      *
      * @return Success status
      */
      bool parseSequenceFile(const std::string &);
      
      /**
       * @brief setup AMI data structure 
       * 
       * @return true if sequences could be set
       * @return false if sequences couln't be set
       */
      bool initAMI();
      void init4DHT();

      void setupCallbackAndPublisher();

      /**
      * @brief Callback to insert new image points to:
      * - the AMI and processes+publishes receiving points
      * - if 4DHT activated: the accumulator of the HT4D process
      *
      * @param msg The input message with image points
      * @param Image_index index of the camera image producing this message
      */
      void insertPoints(const uvdar_core::ImagePointsWithFloatStampedConstPtr &, const size_t &);

      /**
      * @brief Callback to insert points corresponding to pixels with sun to local variable for visualization
      *
      * @param msg The input message with image points
      * @param image_index Index of the camera producing the image
      */
      void insertSunPoints(const uvdar_core::ImagePointsWithFloatStampedConstPtr&, const size_t&);

      /**
       * @brief setup callbacks,subscribers and threads for optional visualization 
       * 
       */
      void setupVisualization(); 


      /**
      * @brief Thread function: 
      * - 4DHT: processes the accumulated image points and periodically retrieves estimated origin points and frequency estimates of blinking markers
      * - AMI: Only used for visualization
      * 
      * @param image_index Index of the camera producing the image
      */
      void ProcessThread(const int&);

      /**
      * @brief Thread function for optional visualization of the detected blinking markers
      *
      * @param te TimerEvent for the timer spinning this thread
      */
      void VisualizationThread([[maybe_unused]] const ros::TimerEvent&);

      /**
      * @brief Method for generating annotated image for optional visualization
      *
      * @param output_image The generated visualization image
      *
      * @return Success status ( 0 - success, 1 - visualization does not need to be generated as the state has not changed, negative - failed, usually due to missing requirements
      */
      int generateVisualization(cv::Mat& );

      /**
      * @brief Updates the latest camera image for background in the optional visualization
      *
      * @param image_msg The input image message
      * @param image_index Index of the camera image producing this message
      */
      void callbackImage(const sensor_msgs::ImageConstPtr&, size_t);

      ros::NodeHandle nh_;
      std::atomic_bool initialized_ = false;  
      
                  
      int image_sizes_received_ = 0;
      std::vector<std::shared_ptr<AMI>> ami_;
      
      std::vector<std::vector<bool>> sequences_;
      std::vector<ros::Publisher> pub_blinkers_seen_;
      std::vector<ros::Publisher> pub_ami_logging_;
      std::vector<ros::Publisher> pub_AMI_all_seq_info;
      std::vector<ros::Publisher> pub_estimated_framerate_;

      using points_seen_callback_t = boost::function<void (const uvdar_core::ImagePointsWithFloatStampedConstPtr&)>;
      std::vector<points_seen_callback_t> cals_points_seen_;
      std::vector<points_seen_callback_t> cals_sun_points_;
      std::vector<ros::Subscriber> sub_points_seen_;
      std::vector<ros::Subscriber> sub_sun_points_;


      // visualization variables
      std::vector<ros::Timer> timer_process_;
      ros::Timer timer_visualization_;
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
      

      std::vector<std::vector<cv::Point>> sun_points_;
      std::mutex mutex_sun;
      
      ros::Time last_publish_ami_logging_;

      // dynamic loaded params
      std::string _uav_name_;   
      bool        _debug_;
      bool        _gui_;
      bool        _publish_visualization_;
      float       _visualization_rate_;
      std::vector<std::string> _camera_topics_;
      bool        _use_camera_for_visualization_;
      std::vector<std::string> _blinkers_seen_topics_;
      std::vector<std::string> _estimated_framerate_topics_;
      bool _pub_tracking_stats_;
      std::vector<std::string> _points_seen_topics_;
      std::vector<std::string> _ami_logging_topics_;
      std::vector<std::string> _ami_all_seq_info_topics;
      

      bool _manchester_code_;
      bool _use_4DHT_;
      std::string _sequence_file;

      // params for AMI
      cv::Point _max_px_shift_;
      int _max_zeros_consecutive_;
      int _stored_seq_len_factor_;
      int _max_buffer_length_;
      int _poly_order_;
      float _decay_factor_; 
      double _conf_probab_percent_;
      int _allowed_BER_per_seq_;
      int _loaded_var_pub_rate_; 
      double _draw_predict_window_sec_;

      // params for 4DHT
      int _accumulator_length_;
      int _pitch_steps_;
      int _yaw_steps_;
      int _max_pixel_shift_;
      int _reasonable_radius_;
      int _nullify_radius_;
      bool _visual_debug_;
      int _process_rate_;

      // for extracting the received sequences from the AMI/4DHT
      struct BlinkData{
        ros::Time                     last_sample_time;
        ros::Time                     last_sample_time_diagnostic;
        unsigned int                  sample_count = -1;
        double                        framerate_estimate = 72;
        std::vector<std::pair<seqPointer,int>> retrieved_blinkers;
        std::vector<std::pair<cv::Point2d,int>>      retrieved_blinkers_4DHT;
        std::vector<double>           pitch_4DHT;
        std::vector<double>           yaw_4DHT;

        std::shared_ptr<std::mutex>   mutex_retrieved_blinkers;
        BlinkData(){mutex_retrieved_blinkers = std::make_shared<std::mutex>();}
        ~BlinkData(){mutex_retrieved_blinkers.reset();}
      };

      std::vector<BlinkData> blink_data_;
      std::vector<std::shared_ptr<AMI>> AMI_trackers_;
      std::vector<std::shared_ptr<HT4DBlinkerTrackerCPU>> ht4dbt_trackers_;
  };
      
  void UVDARBlinkProcessor::onInit(){
  

    nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    last_publish_ami_logging_ = ros::Time::now();

    const bool print_params_console = false;
    loadParams(print_params_console);

    if(!checkLoadedParams()){
      ROS_ERROR("[UVDARBlinkProcessor]: Shutting down - Problem while loading the params!");
      initialized_ = false;
      return;
    } 


    parseSequenceFile(_sequence_file);
    setupCallbackAndPublisher();
    
    sun_points_.resize(_points_seen_topics_.size());
    if(!_use_4DHT_){
      if(!initAMI()){
        ROS_ERROR("[UVDARBlinkProcessor]: Shutting down - AMI wasn't initialized correctly!");
        initialized_ = false;
        return;
      } 
    }else{
      init4DHT();
    }

    for (int i = 0; i < (int)_points_seen_topics_.size(); ++i) {
      blink_data_.push_back(BlinkData());
      mutex_camera_image_.push_back(std::make_shared<std::mutex>());
      camera_image_sizes_.push_back(cv::Size(-1,-1));
    }

    
    setupVisualization();  


    initialized_ = true;
    ROS_INFO("[UVDARBlinkProcessor]: initialized");

  }

  void UVDARBlinkProcessor::loadParams(const bool &print_params_console) {

    mrs_lib::ParamLoader param_loader(nh_, print_params_console, "UVDARBlinkProcessor");

    param_loader.loadParam("uav_name", _uav_name_, std::string());
    param_loader.loadParam("debug", _debug_, bool(false));


    param_loader.loadParam("gui", _gui_, bool(true));                                     
    param_loader.loadParam("publish_visualization", _publish_visualization_, bool(true));
    param_loader.loadParam("visualization_rate", _visualization_rate_, float(2.0));    
    nh_.param("use_camera_for_visualization", _use_camera_for_visualization_, bool(true));

    /***** topic name params *****/
    nh_.param("camera_topics", _camera_topics_, _camera_topics_);
    param_loader.loadParam("points_seen_topics", _points_seen_topics_, _points_seen_topics_);
    nh_.param("blinkers_seen_topics", _blinkers_seen_topics_, _blinkers_seen_topics_);
    nh_.param("estimated_framerate_topics", _estimated_framerate_topics_, _estimated_framerate_topics_);
    nh_.param("ami_logging_topics", _ami_logging_topics_, _ami_logging_topics_);
    nh_.param("ami_all_seq_info_topics", _ami_all_seq_info_topics, _ami_all_seq_info_topics);

    param_loader.loadParam("sequence_file", _sequence_file, std::string());    
    param_loader.loadParam("manchester_code", _manchester_code_, bool(false));
    param_loader.loadParam("use_4DHT", _use_4DHT_, bool(false));
    param_loader.loadParam("pub_tracking_stats", _pub_tracking_stats_, bool(false));

    /***** AMI params *****/
    param_loader.loadParam("max_px_shift_x", _max_px_shift_.x, int(2));
    param_loader.loadParam("max_px_shift_y", _max_px_shift_.y, int(2));
    param_loader.loadParam("max_zeros_consecutive", _max_zeros_consecutive_, int(10));
    param_loader.loadParam("stored_seq_len_factor", _stored_seq_len_factor_, int(20));
    param_loader.loadParam("max_buffer_length", _max_buffer_length_, int(1000));
    param_loader.loadParam("poly_order", _poly_order_, int(4));
    param_loader.loadParam("decay_factor", _decay_factor_, float(0.1));
    param_loader.loadParam("confidence_probability", _conf_probab_percent_, double(75.0));
    param_loader.loadParam("allowed_BER_per_seq", _allowed_BER_per_seq_, int(0));
    param_loader.loadParam("loaded_var_pub_rate", _loaded_var_pub_rate_, int(20));
    param_loader.loadParam("draw_predict_window_sec", _draw_predict_window_sec_, double(0.0));
      
    /***** 4DHT params *****/
    param_loader.loadParam("accumulator_length", _accumulator_length_, int(23));
    param_loader.loadParam("pitch_steps", _pitch_steps_, int(16));
    param_loader.loadParam("yaw_steps", _yaw_steps_, int(8));
    param_loader.loadParam("max_pixel_shift", _max_pixel_shift_, int(1));
    param_loader.loadParam("reasonable_radius", _reasonable_radius_, int(6));
    param_loader.loadParam("nullify_radius", _nullify_radius_, int(5));
    param_loader.loadParam("blink_process_rate", _process_rate_, int(10));
    param_loader.loadParam("visual_debug", _visual_debug_, bool(false));
    if ( _visual_debug_) {
      ROS_WARN_STREAM("[UVDARBlinkProcessor]: You are using visual debugging. This option is only meant for development. Activating it significantly increases load on the system and the user should do so with care.");
    }

  }

  bool UVDARBlinkProcessor::checkLoadedParams() {
    
    if (_camera_topics_.empty()) {
      ROS_WARN("[UVDARBlinkProcessor]: No topics of cameras were supplied");
      _use_camera_for_visualization_ = false;
    }

    if (_points_seen_topics_.empty()) {
      ROS_WARN("[UVDARBlinkProcessor]: No topics of points_seen_topics were supplied. Returning.");
      return false;
    }

    if (_blinkers_seen_topics_.size() != _points_seen_topics_.size()) {
      ROS_ERROR_STREAM("[UVDARBlinkProcessor] The number of pointsSeenTopics (" << _points_seen_topics_.size() 
          << ") is not matching the number of blinkers_seen_topics (" << _blinkers_seen_topics_.size() << ")!");
      return false;
    }
    if (_estimated_framerate_topics_.size() != _points_seen_topics_.size()) {
      ROS_ERROR_STREAM("[UVDARBlinkProcessor] The number of pointsSeenTopics (" << _points_seen_topics_.size() 
          << ") is not matching the number of blinkers_seen_topics (" << _estimated_framerate_topics_.size() << ")!");
      return false;
    }

    // for AMI
    if( 100 <= _conf_probab_percent_){
      ROS_ERROR_STREAM("[UVDARBlinkProcessor]: Wanted confidence interval size equal or bigger than 100\% is set. A Confidence interval of 100\% is not settable! Returning."); 
      return false;
    } 
    if(_draw_predict_window_sec_ != 0.0){
      ROS_WARN("[UVDARBlinkProcessor]: \"_draw_predict_window_sec_\" value is primary for debug use. Please don't set it too high - Otherwise it might cause unecessary load to the system. Unit is seconds");
    }

    if(_use_4DHT_)ROS_WARN("[UVDARBlinkProcessor]: 4DHT activate! This algorithm is deprecated and slower compared to the default algorithm!");

    if(_pub_tracking_stats_)ROS_WARN("[UVDARBlinkProcessor]: - \"pub_tracking_stats\" will be published. Use this option only if seeking statistics related to the AMI/4DHT algorithm.");

    return true;
  }

  bool UVDARBlinkProcessor::parseSequenceFile(const std::string &sequence_file) {

    ROS_WARN_STREAM("[UVDARBlinkProcessor]: Add sanitation - sequences must be of equal, non-zero length");
    ROS_INFO_STREAM("[UVDARBlinkProcessor]: Loading sequence from file: [ " + sequence_file + " ]");
    std::ifstream ifs;
    ifs.open(sequence_file);
    std::string word;
    std::string line;
    std::vector<std::vector<bool>> sequences;
    if (ifs.good()){
      ROS_INFO("[UVDARBlinkProcessor]: Loaded Sequences: [: ");
      while (getline(ifs, line)){
        if (line[0] == '#'){
          continue;
        }
        std::string show_string = "";
        std::vector<bool> sequence;
        std::stringstream iss(line);
        std::string token;
        while (std::getline(iss, token, ',')){
          if (!_manchester_code_){
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

        for (const auto bool_val : sequence){
          if (bool_val)
            show_string += "1,";
          else
            show_string += "0,";
        }

        sequences.push_back(sequence);
        ROS_INFO_STREAM("[UVDARBlinkProcessor]: [" << show_string << "]");
      }
      ROS_INFO("[UVDARBlinkProcessor]: ]");
      ifs.close();
      sequences_ = sequences;
    }else{
      ROS_ERROR_STREAM("[UVDARBlinkProcessor]: Failed to load sequence file " << sequence_file << "! Returning.");
      ifs.close();
      return false;
    }
    return true;
  }

  bool UVDARBlinkProcessor::initAMI(){
    loadedParamsForAMI params_AMI;
    params_AMI.max_px_shift = _max_px_shift_;
    params_AMI.max_zeros_consecutive = _max_zeros_consecutive_;
    params_AMI.stored_seq_len_factor = _stored_seq_len_factor_;
    params_AMI.max_buffer_length = _max_buffer_length_;
    params_AMI.poly_order = _poly_order_;
    params_AMI.decay_factor = _decay_factor_;
    params_AMI.conf_probab_percent = _conf_probab_percent_;
    params_AMI.allowed_BER_per_seq = _allowed_BER_per_seq_;
    
    sun_points_.resize(_points_seen_topics_.size());

    for (int i = 0; i < (int)_points_seen_topics_.size(); ++i) {
      ami_.push_back(std::make_shared<AMI>(params_AMI));

      if(!ami_[i]->setSequences(sequences_)) return false;

      ami_[i]->setDebugFlags(_debug_);

    }
    return true;
  }

  void UVDARBlinkProcessor::init4DHT(){
    for (size_t i = 0; i < _points_seen_topics_.size(); ++i) {
      ht4dbt_trackers_.push_back(
        std::make_shared<HT4DBlinkerTrackerCPU>(
            _accumulator_length_, _pitch_steps_, _yaw_steps_, _max_pixel_shift_, cv::Size(0, 0), _allowed_BER_per_seq_,  _nullify_radius_, _reasonable_radius_
          )
        );
      ht4dbt_trackers_.back()->setDebug(_debug_, _visual_debug_);
      ht4dbt_trackers_.back()->setSequences(sequences_);

    }
  }

  void UVDARBlinkProcessor::setupCallbackAndPublisher() {

    for (size_t i = 0; i < _points_seen_topics_.size(); ++i){

      // Subscribe to corresponding topics
      points_seen_callback_t callback = [image_index = i, this](const uvdar_core::ImagePointsWithFloatStampedConstPtr &points_msg){
        insertPoints(points_msg, image_index);
      };
      cals_points_seen_.push_back(callback);
      sub_points_seen_.push_back(nh_.subscribe(_points_seen_topics_[i], 1, cals_points_seen_[i]));
      
      points_seen_callback_t sun_callback = [image_index=i,this] (const uvdar_core::ImagePointsWithFloatStampedConstPtr& sun_points_msg){
        insertSunPoints(sun_points_msg, image_index);
      };
      cals_sun_points_.push_back(callback);
      sub_sun_points_.push_back(nh_.subscribe(_points_seen_topics_[i] + "/sun", 1, cals_sun_points_[i]));
    } 

    for (size_t i = 0; i < _blinkers_seen_topics_.size(); ++i) {
      pub_blinkers_seen_.push_back(nh_.advertise<uvdar_core::ImagePointsWithFloatStamped>(_blinkers_seen_topics_[i], 1));
      pub_estimated_framerate_.push_back(nh_.advertise<std_msgs::Float32>(_estimated_framerate_topics_[i], 1));
      
      if(_pub_tracking_stats_){
        if(!_use_4DHT_){
          pub_ami_logging_.push_back(nh_.advertise<uvdar_core::AMIDataForLogging>(_ami_logging_topics_[i], 1));
          pub_AMI_all_seq_info.push_back(nh_.advertise<uvdar_core::AMIAllSequences>(_ami_all_seq_info_topics[i], 1));
        }
      }
    }
  }


  void UVDARBlinkProcessor::setupVisualization(){
    if (_gui_ || _publish_visualization_){ 

      current_visualization_done_ = false;
      timer_visualization_ = nh_.createTimer(ros::Rate(_visualization_rate_), &UVDARBlinkProcessor::VisualizationThread, this, false);

      if (_use_camera_for_visualization_){
        /* subscribe to cameras //{ */

        images_current_.resize(_camera_topics_.size());
        // Create callbacks for each camera
        for (size_t i = 0; i < _camera_topics_.size(); ++i) {
          current_images_received_.push_back(false);
          image_callback_t callback = [image_index=i,this] (const sensor_msgs::ImageConstPtr& image_msg) { 
            callbackImage(image_msg, image_index);
          };
          cals_image_.push_back(callback);
          // Subscribe to corresponding topics
          sub_image_.push_back(nh_.subscribe(_camera_topics_[i], 1, cals_image_[i]));
        }
      }
      
      if (_publish_visualization_){
        pub_visualization_ = std::make_unique<mrs_lib::ImagePublisher>(boost::make_shared<ros::NodeHandle>(nh_));
      }
    }

    for (size_t i = 0; i < _points_seen_topics_.size(); ++i) {
      timer_process_.push_back(nh_.createTimer(ros::Duration(1.0/(double)(_process_rate_)), boost::bind(&UVDARBlinkProcessor::ProcessThread, this, i), false, true));
    }
  }


  void UVDARBlinkProcessor::insertPoints(const uvdar_core::ImagePointsWithFloatStampedConstPtr &pts_msg, const size_t & img_index) {
    if (!initialized_) return;

    blink_data_[img_index].sample_count++;
    
    if ((blink_data_[img_index].sample_count % 10) == 0) { //update the estimate of frequency every 10 samples
      blink_data_[img_index].framerate_estimate = 10000000000.0 / (double)((pts_msg->stamp - blink_data_[img_index].last_sample_time_diagnostic).toNSec());
      if (_debug_){
        ROS_INFO_STREAM("[UVDARBlinkProcessor]: Updating frequency: " << blink_data_[img_index].framerate_estimate << " Hz");
      }

      if(!_use_4DHT_){
        ami_[img_index]->updateFramerate(blink_data_[img_index].framerate_estimate);
      }else{
        ht4dbt_trackers_[img_index]->updateFramerate(blink_data_[img_index].framerate_estimate);
      }

      blink_data_[img_index].sample_count = 0;

      blink_data_[img_index].last_sample_time_diagnostic = pts_msg->stamp;
    }

    if (_debug_)
        ROS_INFO_STREAM("[UVDARBlinkProcessor]: td: " << pts_msg->stamp - blink_data_[img_index].last_sample_time);
    if (blink_data_[img_index].last_sample_time >= pts_msg->stamp){
      ROS_ERROR_STREAM("[UVDARBlinkProcessor]: Points arrived out of order!: prev: "<< blink_data_[img_index].last_sample_time << "; curr: " << pts_msg->stamp);
    }
    double dt = (pts_msg->stamp - blink_data_[img_index].last_sample_time).toSec();
    if (dt > (1.5/(blink_data_[img_index].framerate_estimate)) ){
      int new_frame_count = (int)(dt*(blink_data_[img_index].framerate_estimate) + 0.5) - 1;
      if(!_use_4DHT_){
        ROS_ERROR_STREAM("[UVDARBlinkProcessor]: Missing frames! AMI will automatically insert " << new_frame_count << " empty frames!"); 
      }else{
        ROS_ERROR_STREAM("[UVDARBlinkProcessor]: Missing frames! Inserting " << new_frame_count << " empty frames to 4DHT!"); 
        std::vector<cv::Point2i> null_points;
        for (int i = 0; i < new_frame_count; i++){
          ht4dbt_trackers_[img_index]->insertFrame(null_points);
        }
      }

    }
    blink_data_[img_index].last_sample_time = pts_msg->stamp;

    if (_debug_) {
      ROS_INFO_STREAM("[UVDARBlinkProcessor]: Received contours: " << pts_msg->points.size());
    }

    if(!_use_4DHT_){
      ami_[img_index]->processBuffer(pts_msg);

    }else{
      std::vector<cv::Point2i> points;
      for (auto& point : pts_msg->points) {
        points.push_back(cv::Point2d(point.x, point.y));
      }
      ht4dbt_trackers_[img_index]->insertFrame(points);
    }

    if ((!_use_camera_for_visualization_) || ((!_gui_) && (!_publish_visualization_))){
      if ( (camera_image_sizes_[img_index].width <= 0 ) || (camera_image_sizes_[img_index].height <= 0 )){
        camera_image_sizes_[img_index].width = pts_msg->image_width;
        camera_image_sizes_[img_index].height = pts_msg->image_height;
        if(_use_4DHT_){

          ht4dbt_trackers_[img_index]->updateResolution(cv::Size(pts_msg->image_width, pts_msg->image_height));
        }
        if (image_sizes_received_ < (int)(camera_image_sizes_.size())){
          image_sizes_received_++;
        }
      }
    }

    if(_use_4DHT_) return;
    
    uvdar_core::ImagePointsWithFloatStamped msg;
    uvdar_core::AMIDataForLogging ami_logging_msg;
    uvdar_core::AMIAllSequences ami_all_seq_msg;
    ros::Time local_last_sample_time = blink_data_[img_index].last_sample_time;
    {
      std::scoped_lock lock(*(blink_data_[img_index].mutex_retrieved_blinkers));
      blink_data_[img_index].retrieved_blinkers = ami_[img_index]->getResults();

      int valid_signal_cnt = 0 , invalid_signal_cnt = 0;
      for (auto& signal : blink_data_[img_index].retrieved_blinkers) {
        uvdar_core::Point2DWithFloat point;
        // take the last/most up-to-date point and publish to pose calculator
        auto last_point = signal.first->end()[-1];
        point.x = last_point.point.x;
        point.y = last_point.point.y;
        if ( 0 <= signal.second && signal.second <= (int)sequences_.size()){
          point.value = signal.second;
          valid_signal_cnt++;
        }
        else {
          point.value = -2;
          invalid_signal_cnt++;
        }
                  
        // publish values from AMI if the sequence is valid
        uvdar_core::AMISeqVariables ami_seq_msg;
        ami_seq_msg.inserted_time = last_point.insert_time;
        ami_seq_msg.signal_id = signal.second;

        ami_seq_msg.confidence_interval.x = last_point.x_statistics.confidence_interval;
        ami_seq_msg.confidence_interval.y = last_point.y_statistics.confidence_interval;
        ami_seq_msg.predicted_point.x = last_point.x_statistics.predicted_coordinate;
        ami_seq_msg.predicted_point.y = last_point.y_statistics.predicted_coordinate;
        

        for(auto coeff : last_point.x_statistics.coeff){
          ami_seq_msg.x_coeff_reg.push_back(static_cast<float>(coeff));
        }

        for(auto coeff : last_point.y_statistics.coeff){
          ami_seq_msg.y_coeff_reg.push_back(static_cast<float>(coeff));
        }

        for(auto point_state : *signal.first){
          uvdar_core::AMISeqPoint ps_msg;
          uvdar_core::Point2DWithFloat p;
          p.x = point_state.point.x;
          p.y = point_state.point.y;
          p.value = point_state.led_state;
          ps_msg.point = p;
          ps_msg.insert_time = point_state.insert_time;
          ami_seq_msg.sequence.push_back(ps_msg);
        }

        ami_seq_msg.poly_reg_computed.push_back(last_point.x_statistics.poly_reg_computed);
        ami_seq_msg.poly_reg_computed.push_back(last_point.y_statistics.poly_reg_computed);

        ami_seq_msg.extended_search.push_back(last_point.x_statistics.extended_search);
        ami_seq_msg.extended_search.push_back(last_point.y_statistics.extended_search);

        ami_all_seq_msg.sequences.push_back(ami_seq_msg);
        msg.points.push_back(point);
      }
      msg.stamp         = local_last_sample_time;
      msg.image_width   = camera_image_sizes_[img_index].width;
      msg.image_height  = camera_image_sizes_[img_index].height;

      if(_debug_){
        ROS_INFO("[UVDARBlinkProcessor]: Extracted %d valid signals and %d invalid signals", valid_signal_cnt, invalid_signal_cnt);
      }

      // publish the last point for the pose calculate
      pub_blinkers_seen_[img_index].publish(msg);
      if(_pub_tracking_stats_){
        // publish whole sequence with infos from AMI
        pub_AMI_all_seq_info[img_index].publish(ami_all_seq_msg);
      }
      std_msgs::Float32 msg_framerate;
      msg_framerate.data = blink_data_[img_index].framerate_estimate;
      pub_estimated_framerate_[img_index].publish(msg_framerate);


      if(_pub_tracking_stats_){
        // publish loaded variables every _loaded_var_pub_rate_ seconds
        double diff = ros::Time::now().toSec() - last_publish_ami_logging_.toSec();
        if( _loaded_var_pub_rate_ < diff ) {

          if ( ( _loaded_var_pub_rate_ + 0.1 ) <= diff ) 
          last_publish_ami_logging_ = ros::Time::now();

          ami_logging_msg.stamp = last_publish_ami_logging_;
          ami_logging_msg.pub_rate = _loaded_var_pub_rate_; 
          uvdar_core::Point2DWithFloat max_px_shift;
          ami_logging_msg.stored_seq_len_factor = _stored_seq_len_factor_;
          ami_logging_msg.max_buffer_length = _max_buffer_length_;
          ami_logging_msg.default_poly_order = _poly_order_;
          ami_logging_msg.max_zeros_consecutive = _max_zeros_consecutive_;
          max_px_shift.x = _max_px_shift_.x;
          max_px_shift.y = _max_px_shift_.y;
          ami_logging_msg.confidence_probab_t_dist = _conf_probab_percent_;
          ami_logging_msg.decay_factor_weight_func = _decay_factor_;
          ami_logging_msg.max_px_shift = max_px_shift;
          pub_ami_logging_[img_index].publish(ami_logging_msg);
        }
      }
    }
    
  } 

  void UVDARBlinkProcessor::insertSunPoints(const uvdar_core::ImagePointsWithFloatStampedConstPtr& msg, const size_t & image_index) {
    if (!initialized_) return;

    /* int                      countSeen; */
    std::vector<cv::Point2i> points;

    for (auto& point : msg->points) {
      points.push_back(cv::Point2d(point.x, point.y));
    }

    {
      std::scoped_lock lock(mutex_sun);
      sun_points_[image_index] = points;
    }
  }

  void UVDARBlinkProcessor::ProcessThread(const int& image_index) {
    if (!initialized_){
      return;
    }

    if(_use_4DHT_){
      uvdar_core::ImagePointsWithFloatStamped msg;

      if (_debug_){
        ROS_INFO("Processing accumulated points.");
      }

      ros::Time local_last_sample_time = blink_data_[image_index].last_sample_time;
      {

        std::scoped_lock lock(*(blink_data_[image_index].mutex_retrieved_blinkers));
          blink_data_[image_index].retrieved_blinkers_4DHT = ht4dbt_trackers_[image_index]->getResults();
          blink_data_[image_index].pitch_4DHT              = ht4dbt_trackers_[image_index]->getPitch();
          blink_data_[image_index].yaw_4DHT                = ht4dbt_trackers_[image_index]->getYaw();      

      }

      msg.stamp         = local_last_sample_time;
      msg.image_width   = camera_image_sizes_[image_index].width;
      msg.image_height  = camera_image_sizes_[image_index].height;

      for (auto& blinker : blink_data_[image_index].retrieved_blinkers_4DHT) {
        uvdar_core::Point2DWithFloat point;
        point.x     = blinker.first.x;
        point.y     = blinker.first.y;
        if ( 0 <= blinker.second && blinker.second <= (int)sequences_.size()){
          point.value = blinker.second;
        }
        else {
          point.value = -2;
        }
        msg.points.push_back(point);
      }

      pub_blinkers_seen_[image_index].publish(msg);

      std_msgs::Float32 msgFramerate;
      msgFramerate.data = blink_data_[image_index].framerate_estimate;
      pub_estimated_framerate_[image_index].publish(msgFramerate);
    }

    current_visualization_done_ = false;

  }

  void UVDARBlinkProcessor::VisualizationThread([[maybe_unused]] const ros::TimerEvent& te) {
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

          if (_use_4DHT_ && _visual_debug_){
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

  int UVDARBlinkProcessor::generateVisualization(cv::Mat & output_image) {
    if (image_sizes_received_<(int)(camera_image_sizes_.size()))
      return -2;

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
          /* ROS_INFO_STREAM("[UVDARBlinkProcessor]: Channel count: " << images_current_[image_index].channels()); */
          cv::cvtColor(images_current_[image_index], image_rgb, cv::COLOR_GRAY2BGR);
          image_rgb.copyTo(output_image(cv::Rect(start_point.x,0,images_current_[image_index].cols,images_current_[image_index].rows)));
        }
      }
      else {
        output_image(cv::Rect(start_point.x,0,camera_image_sizes_[image_index].width,camera_image_sizes_[image_index].height)) = cv::Scalar(0,0,0);
      }
      
      if(!_use_4DHT_){
        std::scoped_lock lock(*(blink_data_[image_index].mutex_retrieved_blinkers));

        for(int j = 0; j < (int)(blink_data_[image_index].retrieved_blinkers.size()); j++){
          cv::Scalar predict_colour(255,153,255);
          cv::Scalar seq_colour(160,160,160);
          
          cv::Point2d confidence_interval = cv::Point2d(
            blink_data_[image_index].retrieved_blinkers[j].first->end()[-1].x_statistics.confidence_interval,
            blink_data_[image_index].retrieved_blinkers[j].first->end()[-1].y_statistics.confidence_interval
          );
          cv::Point2d predicted = cv::Point2d(
            blink_data_[image_index].retrieved_blinkers[j].first->end()[-1].x_statistics.predicted_coordinate,
            blink_data_[image_index].retrieved_blinkers[j].first->end()[-1].y_statistics.predicted_coordinate
          );
          auto x_coeff = blink_data_[image_index].retrieved_blinkers[j].first->end()[-1].x_statistics.coeff;
          auto y_coeff = blink_data_[image_index].retrieved_blinkers[j].first->end()[-1].y_statistics.coeff;
          double curr_time = blink_data_[image_index].retrieved_blinkers[j].first->end()[-1].insert_time.toSec();
          bool x_poly_reg_computed = blink_data_[image_index].retrieved_blinkers[j].first->end()[-1].x_statistics.poly_reg_computed;
          bool y_poly_reg_computed = blink_data_[image_index].retrieved_blinkers[j].first->end()[-1].y_statistics.poly_reg_computed;
          bool x_extended_search = blink_data_[image_index].retrieved_blinkers[j].first->end()[-1].x_statistics.extended_search;
          bool y_extended_search = blink_data_[image_index].retrieved_blinkers[j].first->end()[-1].y_statistics.extended_search;

          std::vector<cv::Point> interpolated_prediction;
          
          if(x_extended_search || y_extended_search){
          
            double computed_time = curr_time;
  
            double step_size_sec = 1.0 / blink_data_[image_index].framerate_estimate;
            int point_size = _draw_predict_window_sec_/step_size_sec;

            // drawing of the prediction for "_draw_predict_window_sec_" length 
            for(int i = 0; i < point_size; ++i){
              cv::Point2d interpolated_point = cv::Point2d{0,0};
              double x_calculated = 0.0; 
              if(x_poly_reg_computed){
                for(int k = 0; k < (int)x_coeff.size(); ++k){
                x_calculated += x_coeff[k]*pow(computed_time, k);
                }
                interpolated_point.x = x_calculated;
              }else{
                interpolated_point.x = predicted.x;
              }
              if(y_poly_reg_computed){
                double y_calculated = 0.0;
                for(int k = 0; k < (int)y_coeff.size(); ++k){
                  y_calculated += y_coeff[k]*pow(computed_time, k);
                }
                interpolated_point.y = y_calculated;
              }else{
                interpolated_point.y = predicted.y;
              }
              cv::Point2d start_point_d; 
              start_point_d.x = start_point.x;
              start_point_d.y = start_point.y;
              interpolated_point = start_point_d + interpolated_point;
              cv::Point interpolated_point_i;
              interpolated_point_i.x = std::round(interpolated_point.x);
              interpolated_point_i.y = std::round(interpolated_point.y);
              computed_time += step_size_sec;
              interpolated_prediction.push_back(interpolated_point_i);              
            }
          }
  
          cv::Point center = cv::Point(blink_data_[image_index].retrieved_blinkers[j].first->end()[-1].point.x, blink_data_[image_index].retrieved_blinkers[j].first->end()[-1].point.y) + start_point;
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

          if(x_poly_reg_computed || y_poly_reg_computed){
            cv::Point center_predict;
            center_predict = cv::Point(std::round(predicted.x), std::round(predicted.y)) + start_point; 
  
            cv::circle(output_image, center_predict, 2, predict_colour);
            if(confidence_interval.x >= 1 && confidence_interval.y >= 1) {
              // convert to cv::Point
              cv::Point confidence_interval_int;
              confidence_interval_int.x = std::ceil(confidence_interval.x);
              confidence_interval_int.y = std::ceil(confidence_interval.y);
              // construct BB 
              cv::Point2d left_top =     center_predict - confidence_interval_int;   
              cv::Point2d right_bottom = center_predict + confidence_interval_int;

              cv::rectangle(output_image, left_top, right_bottom, predict_colour, 1);
            }
            if(interpolated_prediction.size() != 0){
              cv::polylines(output_image, interpolated_prediction, false, predict_colour, 1);
            }
          }
  
          // draw "past" stored sequence points 
          std::vector<cv::Point> draw_seq;  
          for(auto p : *blink_data_[image_index].retrieved_blinkers[j].first){
            if(p.led_state){
              cv::Point point;
              point.x = p.point.x;
              point.y = p.point.y;
              draw_seq.push_back(point+start_point);
            }
          }
          cv::polylines(output_image, draw_seq, false, seq_colour, 1);
        }
      }else{
        std::scoped_lock lock(*(blink_data_[image_index].mutex_retrieved_blinkers));
        for (int j = 0; j < (int)(blink_data_[image_index].retrieved_blinkers_4DHT.size()); j++) {
          cv::Point center =
            cv::Point(
                blink_data_[image_index].retrieved_blinkers_4DHT[j].first.x, 
                blink_data_[image_index].retrieved_blinkers_4DHT[j].first.y 
                )
            + start_point;
          int signal_index = blink_data_[image_index].retrieved_blinkers_4DHT[j].second;
          if (signal_index >= 0) {
            std::string signal_text = std::to_string(std::max((int)blink_data_[image_index].retrieved_blinkers_4DHT[j].second, 0));
            cv::putText(output_image, cv::String(signal_text.c_str()), center + cv::Point(-5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
            cv::Scalar color = ColorSelector::markerColor(signal_index);
            cv::circle(output_image, center, 5, color);
            double yaw, pitch, len;
            yaw              = blink_data_[image_index].yaw_4DHT[j];
            pitch            = blink_data_[image_index].pitch_4DHT[j];
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
    for (int i = 0; i < (int)sequences_.size(); ++i) {
      cv::Scalar color = ColorSelector::markerColor(i);
      cv::circle(output_image, cv::Point(10, 10 + 15 * i), 5, color);
      cv::putText(output_image, cv::String(std::to_string(i)), cv::Point(15, 15 + 15 * i), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
    }

    current_visualization_done_ = true;
    return 0;
  }

  void UVDARBlinkProcessor::callbackImage(const sensor_msgs::ImageConstPtr& image_msg, size_t image_index) {
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
      if(_use_4DHT_)ht4dbt_trackers_[image_index]->updateResolution(image->image.size());
      if (image_sizes_received_ < (int)(camera_image_sizes_.size())){
        image_sizes_received_++;
      }
    }
  }



}// namespace uvdar

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uvdar::UVDARBlinkProcessor, nodelet::Nodelet)
