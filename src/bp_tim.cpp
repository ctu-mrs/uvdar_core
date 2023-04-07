#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <alternativeHT/alternativeHT.h>
#include <alternativeHT/imu_compensation.h>
#include <color_selector/color_selector.h>
#include <mrs_msgs/ImagePointsWithFloatStamped.h>
#include <std_msgs/Float32.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/image_publisher.h>
#include <uvdar_core/AhtDataForLogging.h>
#include <uvdar_core/AhtSeqVariables.h>
#include <uvdar_core/AhtAllSequences.h>
#include <uvdar_core/AhtSeqPoint.h>
#include <mutex>
#include <thread>
#include <atomic>
#include <fstream>

namespace uvdar{

  /**
   * @brief A nodelet for extracting blinking frequencies and image positions of intermittently appearing image points - alternative approach to the 4DHT
   */
  class UVDAR_BP_Tim : public nodelet::Nodelet {
    public:

      UVDAR_BP_Tim(){};

      ~UVDAR_BP_Tim(){};
  

    private:
      // initializes nodelet
      virtual void onInit();
      // loads params from launch file
      void loadParams(const bool & );
      bool checkLoadedParams();
      bool parseSequenceFile(const std::string &);
      bool initAlternativeHTDataStructure();
      // initializes IMU Compensation topics - currently not fully developed
      void initIMUCompensation();

      void subscribeToPublishedPoints();
      void insertPointToAHT(const mrs_msgs::ImagePointsWithFloatStampedConstPtr &, const size_t &);
      void InsertSunPoints(const mrs_msgs::ImagePointsWithFloatStampedConstPtr&, size_t);


      // functions for visualization
      void initGUI(); 
      void ProcessThread();
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
      std::vector<ros::Publisher> pub_aht_logging_;
      std::vector<ros::Publisher> pub_aht_all_seq_info;
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
      std::vector<std::string> _camera_topics_;
      bool        _use_camera_for_visualization_;
      std::vector<std::string> _blinkers_seen_topics_;
      std::vector<std::string> _estimated_framerate_topics_;
      std::vector<std::string> _points_seen_topics_;
      std::vector<std::string> _aht_logging_topics_;
      std::vector<std::string> _aht_all_seq_info_topics;
      bool _manchester_code_;

      std::string _sequence_file;
      bool        _imu_compensation_;
      std::string _imu_topic_;
      // params for the aht
      cv::Point _max_px_shift_;
      int _max_zeros_consecutive_;
      int _max_ones_consecutive_;
      int _stored_seq_len_factor_;
      int _poly_order_;
      float _decay_factor_; 
      double _conf_probab_percent_;
      int _frame_tolerance_;
      int _allowed_BER_per_seq_;
      



      struct BlinkData {
        std::vector<std::pair<seqPointer,int>> retrieved_blinkers;
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
    NODELET_DEBUG("[UVDAR_BP_Tim]: Initializing UVDAR_BP_Tim Nodelet...");
    
    const bool print_params_console = false;

    loadParams(print_params_console);

    if(!checkLoadedParams()){
      ROS_ERROR("[UVDAR_BP_Tim]: Shutting down - checkLoadedParams() encountered an issue!");
      initialized_ = false;
      return;
    } 


    parseSequenceFile(_sequence_file);
    if(!initAlternativeHTDataStructure()){
      ROS_ERROR("[UVDAR_BP_Tim]: Shutting down - AlternativeHTDataStructure wasn't initialized correctly!");
      initialized_ = false;
      return;
    } 

    if(_imu_compensation_){
      imu_comp = std::make_unique<IMU_COMPENSATION>(_imu_topic_);
    }
    std::string pubTopic_extension = "_imu_comp";
    points_seen_topics_imu_compensation_ = _points_seen_topics_; 
    for(auto & topicName : points_seen_topics_imu_compensation_){
        topicName += pubTopic_extension;
        pub_points_seen_compensated_imu_.push_back(private_nh_.advertise<mrs_msgs::ImagePointsWithFloatStamped>(topicName, 1));
        mrs_msgs::ImagePointsWithFloatStamped p;
        vectPoints_.push_back(p);
    }

    subscribeToPublishedPoints();
    if(_gui_){
      initGUI();  
    }
    initialized_ = true;
  }

  void UVDAR_BP_Tim::loadParams(const bool &print_params_console) {

    mrs_lib::ParamLoader param_loader(private_nh_, print_params_console, "UVDAR_BP_Tim");

    param_loader.loadParam("uav_name", _uav_name_, std::string());
    param_loader.loadParam("debug", _debug_, bool(false));
    param_loader.loadParam("visual_debug", _visual_debug_, bool(false));
    if(_visual_debug_){
          ROS_WARN_STREAM("[UVDAR_BP_Tim]: You are using visual debugging. This option is only meant for development. Activating it significantly increases load on the system and the user should do so with care.");
    }

    param_loader.loadParam("gui", _gui_, bool(true));                                     
    param_loader.loadParam("publish_visualization", _publish_visualization_, bool(true));
    param_loader.loadParam("visualization_rate", _visualization_rate_, float(15.0));       
    private_nh_.param("camera_topics", _camera_topics_, _camera_topics_);

    param_loader.loadParam("points_seen_topics", _points_seen_topics_, _points_seen_topics_);

    param_loader.loadParam("sequence_file", _sequence_file, std::string());    
    param_loader.loadParam("manchester_code", _manchester_code_, bool(false));


    private_nh_.param("blinkers_seen_topics", _blinkers_seen_topics_, _blinkers_seen_topics_);
    private_nh_.param("estimated_framerate_topics", _estimated_framerate_topics_, _estimated_framerate_topics_);
    private_nh_.param("use_camera_for_visualization", _use_camera_for_visualization_, bool(true));
    private_nh_.param("aht_logging_topics", _aht_logging_topics_, _aht_logging_topics_);
    private_nh_.param("aht_all_seq_info_topics", _aht_all_seq_info_topics, _aht_all_seq_info_topics);
    private_nh_.param("imu_topic", _imu_topic_, std::string());

    param_loader.loadParam("imu_compensation", _imu_compensation_, bool(false));
    param_loader.loadParam("decay_factor", _decay_factor_, float(0.1));
    param_loader.loadParam("poly_order", _poly_order_, int(2));
    param_loader.loadParam("stored_seq_len_factor", _stored_seq_len_factor_, int(15));
    param_loader.loadParam("confidence_probability", _conf_probab_percent_, double(75.0));
    param_loader.loadParam("frame_tolerance", _frame_tolerance_, int(5));
    param_loader.loadParam("max_px_shift_x", _max_px_shift_.x, int(2));
    param_loader.loadParam("max_px_shift_y", _max_px_shift_.y, int(2));
    param_loader.loadParam("max_zeros_consecutive", _max_zeros_consecutive_, int(2));
    if(_max_zeros_consecutive_ > 2) ROS_WARN("The allowed consecutive zero bits is set to %d. This might cause more tracking failures. Please consider to set _max_px_shift_ in x and y direction higher to achieve similar tracking results.", _max_zeros_consecutive_); 
    param_loader.loadParam("max_ones_consecutive", _max_ones_consecutive_, int(2));
    param_loader.loadParam("allowed_BER_per_seq", _allowed_BER_per_seq_, int(0));
      
  }

  bool UVDAR_BP_Tim::checkLoadedParams() {

    if (_blinkers_seen_topics_.size() != _points_seen_topics_.size()){
      ROS_ERROR_STREAM("[UVDAR_BP_Tim]: The number of pointsSeenTopics (" << _points_seen_topics_.size() << ") is not matching the number of blinkers_seen_topics (" << _blinkers_seen_topics_.size() << ")! Returning.");
      return false;
    }
    if (_estimated_framerate_topics_.size() != _points_seen_topics_.size()){
      ROS_ERROR_STREAM("[UVDAR_BP_Tim]: The number of pointsSeenTopics (" << _points_seen_topics_.size() << ") is not matching the number of blinkers_seen_topics (" << _estimated_framerate_topics_.size() << ")! Returning.");
      return false;
    }

    if( 100 <= _conf_probab_percent_){
      ROS_ERROR("[UVDAR_BP_Tim]: Wanted confidence interval size equal or bigger than 100%% is set. A Confidence interval of 100%% is not settable! Returning."); 
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

  bool UVDAR_BP_Tim::initAlternativeHTDataStructure(){
    loadedParamsForAHT params_AHT;
    params_AHT.max_px_shift = _max_px_shift_;
    params_AHT.max_zeros_consecutive = _max_zeros_consecutive_;
    params_AHT.max_ones_consecutive = _max_ones_consecutive_;
    params_AHT.stored_seq_len_factor = _stored_seq_len_factor_;
    params_AHT.poly_order = _poly_order_;
    params_AHT.decay_factor = _decay_factor_;
    params_AHT.conf_probab_percent = _conf_probab_percent_;
    params_AHT.frame_tolerance = _frame_tolerance_;
    params_AHT.allowed_BER_per_seq = _allowed_BER_per_seq_;
    
    sun_points_.resize(_points_seen_topics_.size());

    for (int i = 0; i < (int)_points_seen_topics_.size(); ++i) {
      aht_.push_back(std::make_shared<alternativeHT>(params_AHT));

      if(!aht_[i]->setSequences(sequences_)) return false;
      aht_[i]->setDebugFlags(_debug_);

      blink_data_.push_back(BlinkData());

      mutex_camera_image_.push_back(std::make_shared<std::mutex>());
      camera_image_sizes_.push_back(cv::Size(-1,-1));
    }
    return true;
  }

  void UVDAR_BP_Tim::subscribeToPublishedPoints() {

    for (size_t i = 0; i < _points_seen_topics_.size(); ++i){

      if(_imu_compensation_){
        points_seen_callback_t callback_IMU = [image_index = i, this](const mrs_msgs::ImagePointsWithFloatStampedConstPtr &points_msg){
            imu_comp->compensateByIMUMovement(points_msg, image_index, pub_points_seen_compensated_imu_);
        };
        cals_points_seen_imu_.push_back(callback_IMU);
        subs_points_seen_imu_.push_back(private_nh_.subscribe(_points_seen_topics_[i], 1, cals_points_seen_imu_[i]));
      }

      // Subscribe to corresponding topics
      points_seen_callback_t callback = [image_index = i, this](const mrs_msgs::ImagePointsWithFloatStampedConstPtr &points_msg){
              insertPointToAHT(points_msg, image_index);
      };
      cals_points_seen_.push_back(callback);
      if(_imu_compensation_){
        sub_points_seen_.push_back(private_nh_.subscribe(points_seen_topics_imu_compensation_[i], 1, cals_points_seen_[i]));
      }else{
        sub_points_seen_.push_back(private_nh_.subscribe(_points_seen_topics_[i], 1, cals_points_seen_[i]));
      }
      points_seen_callback_t sun_callback = [image_index=i,this] (const mrs_msgs::ImagePointsWithFloatStampedConstPtr& sun_points_msg){
        InsertSunPoints(sun_points_msg, image_index);
      };
      cals_sun_points_.push_back(callback);
      sub_sun_points_.push_back(private_nh_.subscribe(_points_seen_topics_[i] + "/sun", 1, cals_sun_points_[i]));
    } 

    for (size_t i = 0; i < _blinkers_seen_topics_.size(); ++i) {
      pub_blinkers_seen_.push_back(private_nh_.advertise<mrs_msgs::ImagePointsWithFloatStamped>(_blinkers_seen_topics_[i], 1));
      pub_aht_logging_.push_back(private_nh_.advertise<uvdar_core::AhtDataForLogging>(_aht_logging_topics_[i], 1));
      pub_aht_all_seq_info.push_back(private_nh_.advertise<uvdar_core::AhtAllSequences>(_aht_all_seq_info_topics[i], 1));
      pub_estimated_framerate_.push_back(private_nh_.advertise<std_msgs::Float32>(_estimated_framerate_topics_[i], 1));
    }

  }

  void UVDAR_BP_Tim::initGUI(){
     if (_gui_ || _publish_visualization_){ 

          // load the frequencies
        current_visualization_done_ = false;
        timer_visualization_ = private_nh_.createTimer(ros::Rate(_visualization_rate_), &UVDAR_BP_Tim::VisualizationThread, this, false);
        
        if (_use_camera_for_visualization_){
            /* subscribe to cameras //{ */

            if (_camera_topics_.empty()) {
              ROS_WARN("[UVDAR_BP_Tim]: No topics of cameras were supplied");
              _use_camera_for_visualization_ = false;
            }
            else {
              images_current_.resize(_camera_topics_.size());
              // Create callbacks for each camera
              for (size_t i = 0; i < _camera_topics_.size(); ++i) {
                current_images_received_.push_back(false);
                image_callback_t callback = [image_index=i,this] (const sensor_msgs::ImageConstPtr& image_msg) { 
                  callbackImage(image_msg, image_index);
                };
                cals_image_.push_back(callback);
                // Subscribe to corresponding topics
                sub_image_.push_back(private_nh_.subscribe(_camera_topics_[i], 1, cals_image_[i]));
              }
            }
            //}
          }
      
          if (_publish_visualization_){
            pub_visualization_ = std::make_unique<mrs_lib::ImagePublisher>(boost::make_shared<ros::NodeHandle>(private_nh_));
          }
      }
      for (size_t i = 0; i < _points_seen_topics_.size(); ++i) {
        timer_process_.push_back(private_nh_.createTimer(ros::Duration(1.0/(double)(10)), boost::bind(&UVDAR_BP_Tim::ProcessThread, this), false, true));
      }
  }


  void UVDAR_BP_Tim::insertPointToAHT(const mrs_msgs::ImagePointsWithFloatStampedConstPtr &pts_msg, const size_t & img_index) {
    if (!initialized_) return;

    blink_data_[img_index].sample_count++;

    
    if ((blink_data_[img_index].sample_count % 10) == 0) { //update the estimate of frequency every 10 samples
      blink_data_[img_index].framerate_estimate = 10000000000.0 / (double)((pts_msg->stamp - blink_data_[img_index].last_sample_time_diagnostic).toNSec());
      if (_debug_){
        ROS_INFO_STREAM("[UVDAR_BP_Tim]: Updating frequency: " << blink_data_[img_index].framerate_estimate << " Hz");
      }

      aht_[img_index]->updateFramerate(blink_data_[img_index].framerate_estimate);
      blink_data_[img_index].sample_count = 0;

      blink_data_[img_index].last_sample_time_diagnostic = pts_msg->stamp;
    }

    if (_debug_)
        ROS_INFO_STREAM("[UVDAR_BP_Tim]: td: " << pts_msg->stamp - blink_data_[img_index].last_sample_time);
    if (blink_data_[img_index].last_sample_time >= pts_msg->stamp){
      ROS_ERROR_STREAM("[UVDAR_BP_Tim]: Points arrived out of order!: prev: "<< blink_data_[img_index].last_sample_time << "; curr: " << pts_msg->stamp);
    }
    double dt = (pts_msg->stamp - blink_data_[img_index].last_sample_time).toSec();
    if (dt > (1.5/(blink_data_[img_index].framerate_estimate)) ){
      int new_frame_count = (int)(dt*(blink_data_[img_index].framerate_estimate) + 0.5) - 1;
      ROS_ERROR_STREAM("[UVDAR_BP_Tim]: Missing frames! Alternative HT will automatically insert " << new_frame_count << " empty frames!"); 
    }
    blink_data_[img_index].last_sample_time = pts_msg->stamp;

    if (_debug_) {
      ROS_INFO_STREAM("[UVDAR_BP_Tim]: Received contours: " << pts_msg->points.size());
    }

    aht_[img_index]->processBuffer(pts_msg); // insert to alternative HT and process it

    if ((!_use_camera_for_visualization_) || ((!_gui_) && (!_publish_visualization_))){
      if ( (camera_image_sizes_[img_index].width <= 0 ) || (camera_image_sizes_[img_index].width <= 0 )){
        camera_image_sizes_[img_index].width = pts_msg->image_width;
        camera_image_sizes_[img_index].height = pts_msg->image_height;
        if (image_sizes_received_ < (int)(camera_image_sizes_.size())){
          image_sizes_received_++; 

        }
      }
    }

    
    mrs_msgs::ImagePointsWithFloatStamped msg;
    uvdar_core::AhtDataForLogging aht_logging_msg;
    uvdar_core::AhtAllSequences aht_all_seq_msg;
    ros::Time local_last_sample_time = blink_data_[img_index].last_sample_time;
    {
      std::scoped_lock lock(*(blink_data_[img_index].mutex_retrieved_blinkers));
      blink_data_[img_index].retrieved_blinkers = aht_[img_index]->getResults();

      for (auto& signal : blink_data_[img_index].retrieved_blinkers) {
        mrs_msgs::Point2DWithFloat point;
        auto last_point = signal.first->end()[-1];
        point.x = last_point.point.x;
        point.y = last_point.point.y;
        if (signal.second <= (int)sequences_.size()){
          point.value = signal.second;

        }
        else {
          point.value = -2;
        }
                  
          // publish values from aht if the sequence is valid
          uvdar_core::AhtSeqVariables aht_seq_msg;
          aht_seq_msg.inserted_time = last_point.insert_time;
          aht_seq_msg.signal_id = signal.second;

          aht_seq_msg.confidence_interval.x = last_point.x_statistics.confidence_interval;
          aht_seq_msg.confidence_interval.y = last_point.y_statistics.confidence_interval;
          aht_seq_msg.predicted_point.x = last_point.x_statistics.predicted_coordinate;
          aht_seq_msg.predicted_point.y = last_point.y_statistics.predicted_coordinate;

          for(auto coeff : last_point.x_statistics.coeff){
            aht_seq_msg.x_coeff_reg.push_back(static_cast<float>(coeff));
          }

          for(auto coeff : last_point.y_statistics.coeff){
            aht_seq_msg.y_coeff_reg.push_back(static_cast<float>(coeff));
          }

          for(auto point_state : *signal.first){
            uvdar_core::AhtSeqPoint ps_msg;
            mrs_msgs::Point2DWithFloat p;
            p.x = point_state.point.x;
            p.y = point_state.point.y;
            p.value = point_state.led_state;
            ps_msg.point = p;
            ps_msg.insert_time = point_state.insert_time;
            aht_seq_msg.sequence.push_back(ps_msg);
          }

          aht_seq_msg.poly_reg_computed.push_back(last_point.x_statistics.poly_reg_computed);
          aht_seq_msg.poly_reg_computed.push_back(last_point.y_statistics.poly_reg_computed);


          aht_all_seq_msg.sequences.push_back(aht_seq_msg);
        // publish for pose calculate
        msg.points.push_back(point);
      }
      msg.stamp         = local_last_sample_time;
      msg.image_width   = camera_image_sizes_[img_index].width;
      msg.image_height  = camera_image_sizes_[img_index].height;

      // publish loaded variables 
      mrs_msgs::Point2DWithFloat max_px_shift;
      max_px_shift.x = _max_px_shift_.x;
      max_px_shift.y = _max_px_shift_.y;
      aht_logging_msg.stamp = local_last_sample_time;
      aht_logging_msg.sequence_file = _sequence_file;
      aht_logging_msg.frame_tolerance_till_seq_rejected = _frame_tolerance_;
      aht_logging_msg.stored_seq_len_factor = _stored_seq_len_factor_;
      aht_logging_msg.default_poly_order = _poly_order_;
      aht_logging_msg.confidence_probab_t_dist = _conf_probab_percent_;
      aht_logging_msg.decay_factor_weight_func = _decay_factor_;
      aht_logging_msg.max_px_shift = max_px_shift;


      pub_blinkers_seen_[img_index].publish(msg);
      pub_aht_logging_[img_index].publish(aht_logging_msg);
      pub_aht_all_seq_info[img_index].publish(aht_all_seq_msg);

      std_msgs::Float32 msg_framerate;
      msg_framerate.data = blink_data_[img_index].framerate_estimate;
      pub_estimated_framerate_[img_index].publish(msg_framerate);

    }
    
  } 

  void UVDAR_BP_Tim::InsertSunPoints(const mrs_msgs::ImagePointsWithFloatStampedConstPtr& msg, size_t image_index) {
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


  void UVDAR_BP_Tim::ProcessThread() {
    if (!initialized_){
      return;
    }
    current_visualization_done_ = false;
  }

  void UVDAR_BP_Tim::VisualizationThread([[maybe_unused]] const ros::TimerEvent& te) {
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
        ROS_ERROR_STREAM("[UVDAR_BP]: Size of image " << i << " was not received! Returning.");
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


          std::vector<cv::Point> interpolated_prediction;
          
          if(x_poly_reg_computed || y_poly_reg_computed){
          
            double computed_time = curr_time;
            bool x_all_coeff_zero = std::all_of(x_coeff.begin(), x_coeff.end(), [](double coeff){return coeff == 0;});
            bool y_all_coeff_zero = std::all_of(y_coeff.begin(), y_coeff.end(), [](double coeff){return coeff == 0;});
  
            double prediction_window = 0.5;
            double step_size_sec = 0.02;
            int point_size = prediction_window/step_size_sec;

            // drawing of the prediction for "prediction_window" length
            for(int i = 0; i < point_size; ++i){
              // std::cout << "BP Time " << computed_time;
              cv::Point2d interpolated_point = cv::Point2d{0,0};
              double x_calculated = 0.0; 
              // if(!x_all_coeff_zero){
                for(int k = 0; k < (int)x_coeff.size(); ++k){
                x_calculated += x_coeff[k]*pow(computed_time, k);
                }
                interpolated_point.x = x_calculated;
              // }
              // else{
                // interpolated_point.x = std::round(predicted.x);
              // }
              // if(!y_all_coeff_zero){
                double y_calculated = 0.0;
                for(int k = 0; k < (int)y_coeff.size(); ++k){
                  y_calculated += y_coeff[k]*pow(computed_time, k);
                }
                interpolated_point.y = y_calculated;
              // }
              // else{
                // interpolated_point.y = std::round(predicted.y);
              // }
                cv::Point2d start_point_d; 
                start_point_d.x = start_point.x;
                start_point_d.y = start_point.y;
                interpolated_point = start_point_d + interpolated_point;
                cv::Point interpolated_point_i;
                interpolated_point_i.x = std::round(interpolated_point.x);
                interpolated_point_i.y = std::round(interpolated_point.y);
              computed_time += step_size_sec;
              // std::cout << std::fixed << computed_time << std::endl;
              // interpolated_point = interpolated_point + start_point;
              // std::cout << interpolated_point.x << " " << interpolated_point.y << "\n";
              // if(( prediction.x != 0 || prediction.y != 0) && (!x_all_coeff_zero || !y_all_coeff_zero) )std::cout << "prediction " << prediction.x << " " << prediction.y << "\n"; 
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
  
            cv::circle(output_image, center_predict, 1, predict_colour);
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
              // std::cout << "LINE" << interpolated_prediction.size() << "\n";
              // for(auto p : interpolated_prediction){
                // std::cout << p.x << " " << p.y << "\n";
              // }
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
    if ( (camera_image_sizes_[image_index].width <= 0 ) || (camera_image_sizes_[image_index].width <= 0 )){
      camera_image_sizes_[image_index] = image->image.size();
      if (image_sizes_received_ < (int)(camera_image_sizes_.size())){
        image_sizes_received_++;
      }
    }
  }

}// namespace uvdar

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uvdar::UVDAR_BP_Tim, nodelet::Nodelet);

