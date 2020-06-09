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
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt32MultiArray.h>
#include <mrs_msgs/Int32MultiArrayStamped.h>
#include <stdint.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <atomic>

namespace enc = sensor_msgs::image_encodings;

namespace uvdar {

class UVDARBlinkProcessor : public nodelet::Nodelet {
public:

  /* onInit() //{ */

  void onInit() {
    initialized_ = false;
    images_received_ = false;

    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    nh_.param("uav_name", _uav_name_, std::string());
    nh_.param("DEBUG", DEBUG, bool(false));
    nh_.param("VisDEBUG", _visual_debug_, bool(false));
    nh_.param("GUI", GUI, bool(false));
    if (GUI)
      ROS_INFO("[UVDARBlinkProcessor]: GUI is true");
    else
      ROS_INFO("[UVDARBlinkProcessor]: GUI is false");

    nh_.param("accumulatorLength", accumulatorLength, int(23));
    nh_.param("pitchSteps", pitchSteps, int(16));
    nh_.param("yawSteps", yawSteps, int(8));
    nh_.param("maxPixelShift", maxPixelShift, int(1));

    nh_.param("publishVisualization", publishVisualization, bool(false));
    nh_.param("visualizationRate", _visualization_rate_, int(5));

    nh_.param("reasonableRadius", _reasonable_radius_, int(3));
    nh_.param("nullifyRadius", _nullify_radius_, int(5));
    nh_.param("processRate", processRate, int(10));
    nh_.param("returnFrequencies", returnFrequencies, bool(false));


    nh_.param("legacy", _legacy, bool(false));
    if (_legacy){
      nh_.param("legacy_delay", _legacy_delay, double(0.2));
      ROS_INFO_STREAM("[UVDARBlinkProcessor]: Legacy mode in effect. Set delay is " << _legacy_delay << "s");
    }

    
    /* subscribe to pointsSeen //{ */

    /* if (_legacy){ */
    /*   pointsSubscriberLegacy = nh_.subscribe("pointsSeen", 1, &UVDARBlinkProcessor::insertPointsLegacy, this); */
    /* } */
    /* else{ */
    /*   pointsSubscriber = nh_.subscribe("pointsSeen", 1, &UVDARBlinkProcessor::insertPoints, this); */
    /* } */

    /* pointsPublisher  = nh_.advertise<uvdar::Int32MultiArrayStamped>("blinkersSeen", 1); */

    std::vector<std::string> _points_seen_topics;
    nh_.param("points_seen_topics", _points_seen_topics, _points_seen_topics);
    if (_points_seen_topics.empty()) {
      ROS_WARN("[UVDARBlinkProcessor]: No topics of pointsSeen were supplied");
    }
    blinkData.resize(_points_seen_topics.size());

    // Create callbacks for each camera
    pointsSeenCallbacks.resize(_points_seen_topics.size());
    pointsSeenCallbacksLegacy.resize(_points_seen_topics.size());

    sun_points_.resize(_points_seen_topics.size());

    for (size_t i = 0; i < _points_seen_topics.size(); ++i) {

    // Subscribe to corresponding topics
    /* ROS_INFO("[UVDARBlinkProcessor]: HERE A"); */
      if (_legacy){
        points_seen_callback_legacy_t callback = [imageIndex=i,this] (const std_msgs::UInt32MultiArrayConstPtr& pointsMessage) { 
          InsertPointsLegacy(pointsMessage, imageIndex);
        };
        pointsSeenCallbacksLegacy[i] = callback;
        /* pointsSeenCallbacksLegacy.push_back(callback); */
        pointsSeenSubscribers.push_back(nh_.subscribe(_points_seen_topics[i], 1, &points_seen_callback_legacy_t::operator(), &pointsSeenCallbacksLegacy[i]));
      }
      else {
        points_seen_callback_t callback = [imageIndex=i,this] (const mrs_msgs::Int32MultiArrayStampedConstPtr& pointsMessage) { 
          InsertPoints(pointsMessage, imageIndex);
        };
        pointsSeenCallbacks[i] = callback;
        points_seen_callback_t sun_callback = [imageIndex=i,this] (const mrs_msgs::Int32MultiArrayStampedConstPtr& sunPointsMessage) { 
          InsertSunPoints(sunPointsMessage, imageIndex);
        };
        pointsSeenCallbacks[i] = callback;
        /* pointsSeenCallbacks.push_back(callback); */
        pointsSeenSubscribers.push_back(nh_.subscribe(_points_seen_topics[i], 1, &points_seen_callback_t::operator(), &pointsSeenCallbacks[i]));
        sunPointsSeenSubscribers.push_back(nh_.subscribe(_points_seen_topics[i]+"/sun", 1, &points_seen_callback_t::operator(), &sunPointsSeenCallbacks[i]));
      }
    /* ROS_INFO("[UVDARBlinkProcessor]: HERE B"); */
    }

    //}
    
    for (size_t i = 0; i < _points_seen_topics.size(); ++i) {
      ht3dbt_trackers.push_back(
          new HT4DBlinkerTracker(accumulatorLength, pitchSteps, yawSteps, maxPixelShift, cv::Size(752, 480), _nullify_radius_, _reasonable_radius_));

      ht3dbt_trackers.back()->setDebug(DEBUG, _visual_debug_);
      processSpinRates.push_back(new ros::Rate((double)processRate));
    }

    /* initialize the publishers //{ */

    /* currImage = cv::Mat(cv::Size(752, 480), CV_8UC3, cv::Scalar(0, 0, 0)); */
    /* viewImage = currImage.clone(); */
    //CHECK

    nh_.param("UseCameraForVisualization", use_camera_for_visualization_, bool(true));
    if (use_camera_for_visualization_){
      /* subscribe to cameras //{ */

      std::vector<std::string> _camera_topics;
      nh_.param("camera_topics", _camera_topics, _camera_topics);
      if (_camera_topics.empty()) {
        ROS_WARN("[UVDARBlinkProcessor]: No topics of cameras were supplied");
        use_camera_for_visualization_ = false;
      }
      else {
        currentImagesReceived.resize(_camera_topics.size());
        currentImages.resize(_camera_topics.size());
        // Create callbacks for each camera
        imageCallbacks.resize(_camera_topics.size());
        for (size_t i = 0; i < _camera_topics.size(); ++i) {
          currentImagesReceived[i] = false;
          image_callback_t callback = [imageIndex=i,this] (const sensor_msgs::ImageConstPtr& image_msg) { 
            ProcessRaw(image_msg, imageIndex);
          };
          imageCallbacks[i] = callback;
          /* imageCallbacks.push_back(callback); */
          // Subscribe to corresponding topics
          imageSubscribers.push_back(nh_.subscribe(_camera_topics[i], 1, &image_callback_t::operator(), &imageCallbacks[i]));
        }
      }

    }

    std::vector<std::string> _blinkers_seen_topics;
    std::vector<std::string> _estimated_framerate_topics;
    nh_.param("blinkersSeenTopics", _blinkers_seen_topics, _blinkers_seen_topics);
    nh_.param("estimatedFramerateTopics", _estimated_framerate_topics, _estimated_framerate_topics);

    if (_blinkers_seen_topics.size() != _points_seen_topics.size()) {
      ROS_ERROR_STREAM("[UVDARBlinkProcessor] The number of poinsSeenTopics (" << _points_seen_topics.size() 
          << ") is not matching the number of _blinkers_seen_topics (" << _blinkers_seen_topics.size() << ")!");
    }
    if (_estimated_framerate_topics.size() != _points_seen_topics.size()) {
      ROS_ERROR_STREAM("[UVDARBlinkProcessor] The number of poinsSeenTopics (" << _points_seen_topics.size() 
          << ") is not matching the number of _blinkers_seen_topics (" << _estimated_framerate_topics.size() << ")!");
    }

    for (size_t i = 0; i < _blinkers_seen_topics.size(); ++i) {
      blinkersSeenPublishers.push_back(nh_.advertise<mrs_msgs::Int32MultiArrayStamped>(_blinkers_seen_topics[i], 1));
    }
    for (size_t i = 0; i < _estimated_framerate_topics.size(); ++i) {
      estimatedFrameratePublishers.push_back(nh_.advertise<std_msgs::Float32>(_estimated_framerate_topics[i], 1));
    }

      //}

    nh_.param("InvertedPoints", InvertedPoints, bool(false));
    nh_.param("frequencyCount", frequencyCount, int(4));

    nh_.param("beacon",_beacon_,bool(false));
    /* if (frequencyCount != 2){ */
    /*   ROS_ERROR("HEYYY"); */
    /*   return; */
    /* } */

    // load the frequencies
    frequencySet.resize(frequencyCount);
    std::vector<double> defaultFrequencySet{6, 10, 15, 30, 8, 12};
    for (int i = 0; i < frequencyCount; ++i) {
      nh_.param("frequency" + std::to_string(i + (_beacon_?0:1)), frequencySet[i], defaultFrequencySet.at(i));
    }

    prepareFrequencyClassifiers();

    for (size_t i = 0; i < _points_seen_topics.size(); ++i) {
      process_threads.emplace_back(&UVDARBlinkProcessor::ProcessThread, this, i);
    }

    if (GUI || publishVisualization) {
      current_visualization_done_ = false;
    }

    if (GUI || _visual_debug_) {
      show_thread  = std::thread(&UVDARBlinkProcessor::ShowThread, this);
    }

    if (publishVisualization) {
      image_transport::ImageTransport it(nh_);
      imPub = it.advertise("visualization", 1);
      visualization_thread  = std::thread(&UVDARBlinkProcessor::VisualizeThread, this);
    }

    initialized_ = true;
    ROS_INFO("[UVDARBlinkProcessor]: initialized");
  }

  //}


  /* prepareFrequencyClassifiers() //{ */

  void prepareFrequencyClassifiers() {
    for (int i = 0; i < (int)(frequencySet.size()); i++) {
      periodSet.push_back(1.0 / frequencySet[i]);
    }
    for (int i = 0; i < (int)(frequencySet.size()) - 1; i++) {
      periodBoundsBottom.push_back((periodSet[i] * (1.0 - boundary_ratio) + periodSet[i + 1] * boundary_ratio));
    }
    periodBoundsBottom.push_back(1.0 / max_frequency);

    periodBoundsTop.push_back(1.0 / min_frequency);
    for (int i = 1; i < (int)(frequencySet.size()); i++) {
      periodBoundsTop.push_back((periodSet[i] * boundary_ratio + periodSet[i - 1] * (1.0 - boundary_ratio)));
    }


    /* periodBoundsTop.back()           = 0.5 * periodSet.back() + 0.5 * periodSet[secondToLast]; */
    /* periodBoundsBottom[secondToLast] = 0.5 * periodSet.back() + 0.5 * periodSet[secondToLast]; */
  }

  //}

private:

  void InsertPointsLegacy(const std_msgs::UInt32MultiArrayConstPtr& msg, size_t imageIndex){
    /* if (DEBUG) */
    /*   ROS_INFO_STREAM("Getting message: " << *msg); */
    mrs_msgs::Int32MultiArrayStampedPtr msg_stamped(new mrs_msgs::Int32MultiArrayStamped);
    msg_stamped->stamp = ros::Time::now()-ros::Duration(_legacy_delay);
    msg_stamped->layout= msg->layout;
    std::vector<int> intVec(msg->data.begin(), msg->data.end());
    msg_stamped->data = intVec;
    /* msg_stamped->data = msg->data; */
    InsertPoints(msg_stamped, imageIndex);
    //CHECK
  }
  
  /* InsertPoints //{ */

  void InsertPoints(const mrs_msgs::Int32MultiArrayStampedConstPtr& msg, size_t imageIndex) {
    if (!initialized_) return;

    int                      countSeen;
    std::vector<cv::Point2i> points;
    countSeen = (int)((msg)->layout.dim[0].size);

    BlinkData& data = blinkData[imageIndex];
    auto* ht3dbt = ht3dbt_trackers[imageIndex];

    data.timeSamples++;

    if (data.timeSamples >= 10) {
      ros::Time nowTime = ros::Time::now();

      data.framerateEstim = 10000000000.0 / (double)((nowTime - data.lastSignal).toNSec());
      if (DEBUG)
        std::cout << "Updating frequency: " << data.framerateEstim << " Hz" << std::endl;
      data.lastSignal = nowTime;
      ht3dbt->updateFramerate(data.framerateEstim);
      data.timeSamples = 0;
    }

    if (DEBUG) {
      ROS_INFO("Received contours: %d", countSeen);
    }
    if (countSeen < 1) {
      data.foundTarget = false;
    } else {
      data.foundTarget = true;
      data.lastSeen    = ros::Time::now();
    }

    /* { */
    cv::Point currPoint;
    /* bool      hasTwin; */
    for (int i = 0; i < countSeen; i++) {
      if (msg->data[(i * 3) + 2] <= 200) {
        if (InvertedPoints)
          currPoint = cv::Point2d(currentImages[imageIndex].cols - msg->data[(i * 3)], currentImages[imageIndex].rows - msg->data[(i * 3) + 1]);
        else
          currPoint = cv::Point2d(msg->data[(i * 3)], msg->data[(i * 3) + 1]);

        /* hasTwin = false; */
        /* for (int n = 0; n < points.size(); n++) { */
        /*   if (cv::norm(currPoint - points[i]) < 5) { */
        /*     hasTwin = true; */
        /*     break; */
        /*   } */
        /* } */

        /* if (!hasTwin) */
        points.push_back(currPoint);
      }
    }
    /* } */

    {
      /* std::scoped_lock lock(*(blinkData[imageIndex].retrievedBlinkersMutex)); */
      lastPointsTime = msg->stamp;
      ht3dbt->insertFrame(points);
    }
  }

  //}

  /* InsertSunPoints //{ */

  void InsertSunPoints(const mrs_msgs::Int32MultiArrayStampedConstPtr& msg, size_t imageIndex) {
    if (!initialized_) return;

    int                      countSeen;
    std::vector<cv::Point2i> points;
    countSeen = (int)((msg)->layout.dim[0].size);

    cv::Point currPoint;
    for (int i = 0; i < countSeen; i++) {
      if (msg->data[(i * 3) + 2] <= 200) {
        if (InvertedPoints)
          currPoint = cv::Point2d(currentImages[imageIndex].cols - msg->data[(i * 3)], currentImages[imageIndex].rows - msg->data[(i * 3) + 1]);
        else
          currPoint = cv::Point2d(msg->data[(i * 3)], msg->data[(i * 3) + 1]);
        points.push_back(currPoint);
      }
    }

    {
      std::scoped_lock lock(mutex_sun);
      sun_points_[imageIndex] = points;
    }
  }

  //}

  /* ProcessThread //{ */

  void ProcessThread(size_t imageIndex) {
    std::vector<int>  msgdata;
    mrs_msgs::Int32MultiArrayStamped msg;
    clock_t                    begin, end;
    double                     elapsedTime;

    while (!initialized_) 
      processSpinRates[imageIndex]->sleep();

    auto* ht3dbt = ht3dbt_trackers[imageIndex];
    auto& retrievedBlinkers = blinkData[imageIndex].retrievedBlinkers;

    processSpinRates[imageIndex]->reset();
    while (ros::ok()) {
      /* if (ht3dbt->isCurrentBatchProcessed()){ */
      /*   /1* if (DEBUG) *1/ */
      /*     ROS_INFO("Skipping batch, already processed."); */
      /*   continue; */
      /*   /1* ros::Duration(10).sleep(); *1/ */
      /* } */

      if (DEBUG)
        ROS_INFO("Processing accumulated points.");

      begin = std::clock();
      ros::Time local_lastPointsTime = lastPointsTime;

      {
        /* std::scoped_lock lock(*(blinkData[imageIndex].retrievedBlinkersMutex)); */
        retrievedBlinkers = ht3dbt->getResults();
      }
      end         = std::clock();
      elapsedTime = double(end - begin) / CLOCKS_PER_SEC;
      if (DEBUG)
        std::cout << "Processing: " << elapsedTime << " s, " << 1.0 / elapsedTime << " Hz" << std::endl;

      msgdata.clear();
      msg.stamp = local_lastPointsTime;
      msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
      msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
      msg.layout.dim[0].size   = retrievedBlinkers.size();
      msg.layout.dim[0].label  = "count";
      msg.layout.dim[0].stride = retrievedBlinkers.size() * 3;
      msg.layout.dim[1].size   = 3;
      msg.layout.dim[1].label  = "value";
      msg.layout.dim[1].stride = 3;
      for (size_t i = 0; i < retrievedBlinkers.size(); i++) {
        msgdata.push_back(retrievedBlinkers[i].x);
        msgdata.push_back(retrievedBlinkers[i].y);
        if (returnFrequencies)
          msgdata.push_back(retrievedBlinkers[i].z);
        else
          msgdata.push_back(findMatch(retrievedBlinkers[i].z));
      }


      msg.data = msgdata;
      blinkersSeenPublishers[imageIndex].publish(msg);

      std_msgs::Float32 msgFramerate;
      msgFramerate.data = blinkData[imageIndex].framerateEstim;
      estimatedFrameratePublishers[imageIndex].publish(msgFramerate);

      processSpinRates[imageIndex]->sleep();

      if (!use_camera_for_visualization_)
        current_visualization_done_ = false;
    }
  }

  //}

  /* findMatch() //{ */

  int findMatch(double i_frequency) {
    double period = 1.0 / i_frequency;
    for (int i = 0; i < (int)(periodSet.size()); i++) {
      /* std::cout << period << " " <<  periodBoundsTop[i] << " " << periodBoundsBottom[i] << " " << periodSet[i] << std::endl; */
      if ((period > periodBoundsBottom[i]) && (period < periodBoundsTop[i])) {
        return i;
      }
    }
    return -1;
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

  //}

  /* VisualizeThread() //{ */

  void VisualizeThread() {
    ROS_INFO("Visualize thread");

    ros::Rate r(_visualization_rate_);
    sensor_msgs::ImagePtr msg;
    while (ros::ok()) {
      if (initialized_){
        std::scoped_lock lock(mutex_show);

        if (GenerateVisualization() >= 0){

          msg = cv_bridge::CvImage(std_msgs::Header(), enc::BGR8, viewImage).toImageMsg();
          imPub.publish(msg);
        }
      }
      r.sleep();
    }
  }

  //}

  /* ShowThread() //{ */

  void ShowThread() {
    cv::Mat temp;

    while (ros::ok()) {
      if (initialized_){
        std::scoped_lock lock(mutex_show);


        if (GUI){
          if (GenerateVisualization() >= 0){
            cv::imshow("ocv_blink_retrieval_" + _uav_name_, viewImage);
          }
        }
        if (_visual_debug_){
          temp = ht3dbt_trackers[0]->getVisualization();
          if (temp.cols>0)
            cv::imshow("ocv2_hough_space_" + _uav_name_, temp);
        }

      }
      cv::waitKey(1000.0 / 10.0);
    }
  }

  //}

  /* GenerateVisualization() //{ */

    int GenerateVisualization() {
      if (use_camera_for_visualization_ && !images_received_)
        return -2;

      if (current_visualization_done_)
        return 1;
      
      int image_count = blinkData.size();
      int image_width, image_height;

      if (!use_camera_for_visualization_){
        image_width = 752;
        image_height = 480;
        viewImage = cv::Mat(
            image_height, 
            (image_width + 1) * image_count - 1, 
            CV_8UC3,
            cv::Scalar(0,0,0));
      }
      else{
        image_width = currentImages[0].cols;
        image_height = currentImages[0].rows;
        viewImage = cv::Mat(
            currentImages[0].rows, 
            (currentImages[0].cols + 1) * image_count - 1, 
            CV_8UC3,
            cv::Scalar(255, 255, 255));
      }


      if (viewImage.rows == 0 || viewImage.cols == 0) return -1;

      /* loop through all trackers and update the data //{ */

      for (int imageIndex = 0; imageIndex < image_count; ++imageIndex) {
        cv::line(viewImage, cv::Point2i(image_width, 0), cv::Point2i(image_width,image_height-1),cv::Scalar(255,255,255));

        auto* ht3dbt = ht3dbt_trackers[imageIndex];
        BlinkData& data = blinkData[imageIndex];

        int differenceX = (image_width + 1) * imageIndex;

        if (use_camera_for_visualization_){
          if (currentImagesReceived[imageIndex])
            currentImages[imageIndex].copyTo(viewImage(cv::Rect(differenceX,0,image_width,image_height)));
          /* ROS_INFO_STREAM("RECT: " << cv::Rect(differenceX,0,image_width,image_height)); */
          /* cv::imshow("TMP",currentImages[imageIndex]); */
          /* cv::waitKey(100); */
        }



        data.currTrackerCount = ht3dbt->getTrackerCount();

        {
          /* std::scoped_lock lock(*(data.retrievedBlinkersMutex)); */
          //CHECK separate mutexes for each image
        
          auto& rbs = data.retrievedBlinkers;
          for (int i = 0; i < (int)(rbs.size()); i++) {
            cv::Point center = cv::Point(rbs[i].x + differenceX, rbs[i].y);

            int        freqIndex = findMatch(rbs[i].z);
            /* std::cout << "f: " << retrievedBlinkers[i].z << std::endl; */
            if (freqIndex >= 0) {
              char       freqText[4];
              /* sprintf(freqText,"%d",std::max((int)retrievedBlinkers[i].z,0)); */
              sprintf(freqText, "%d", std::max((int)rbs[i].z, 0));
              cv::putText(viewImage, cv::String(freqText), center + cv::Point(-5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
              cv::Scalar color = marker_color(freqIndex);
              cv::circle(viewImage, center, 5, color);
              double yaw, pitch, len;
              yaw              = ht3dbt->getYaw(i);
              pitch            = ht3dbt->getPitch(i);
              len              = cos(pitch);
              cv::Point target = center - (cv::Point(len * cos(yaw) * 20, len * sin(yaw) * 20.0));
              cv::line(viewImage, center, target, cv::Scalar(0, 0, 255), 2);
            }
            else {
              cv::circle(viewImage, center, 2, cv::Scalar(160,160,160));
              /* viewImage.at<cv::Vec3b>(center) = cv::Vec3b(255,255,255); */
            }
          }

        }
      
      std::vector<cv::Point> sun_points_local;
      {
        std::scoped_lock lock(mutex_sun);
        sun_points_local = sun_points_[imageIndex];
      }

      for (auto point: sun_points_local){
        cv::Point sun_current = point + cv::Point(differenceX, 0);
         cv::circle(viewImage, sun_current, 10, cv::Scalar(255,255,255));
         cv::circle(viewImage, sun_current, 2, cv::Scalar(255,255,255));
         cv::putText(viewImage, cv::String("Sun"), sun_current + cv::Point(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
      }
      
      }

      //}

      // draw the legend
      for (int i = 0; i < (int)(frequencySet.size()); ++i) {
        cv::Scalar color = marker_color(i);
        cv::circle(viewImage, cv::Point(10, 10 + 15 * i), 5, color);
        cv::putText(viewImage, cv::String(to_string_precision(frequencySet[i],0)), cv::Point(15, 15 + 15 * i), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
      }

      current_visualization_done_ = true;
      return 0;
  }

  //}

  /* ProcessCompressed() & ProcessRaw() //{ */

  void ProcessCompressed(const sensor_msgs::CompressedImageConstPtr& image_msg, size_t imageIndex) {
    cv_bridge::CvImagePtr image;
    if (image_msg != NULL) {
      image = cv_bridge::toCvCopy(image_msg, enc::RGB8);
      {
        std::scoped_lock lock(mutex_show);
        currentImages[imageIndex] = image->image; 
       currentImagesReceived[imageIndex] = true;
        current_visualization_done_ = false;
      }
    images_received_ = true;
    }
  }

  void ProcessRaw(const sensor_msgs::ImageConstPtr& image_msg, size_t imageIndex) {
    cv_bridge::CvImageConstPtr image;
    image = cv_bridge::toCvShare(image_msg, enc::RGB8);
    {
      std::scoped_lock lock(mutex_show);
       currentImages[imageIndex] = image->image; 
       currentImagesReceived[imageIndex] = true;
       current_visualization_done_ = false;
    }
    /* cv::imshow("TMP",currentImages[imageIndex]); */
    /* cv::waitKey(1); */

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

  std::atomic_bool initialized_;
  std::atomic_bool current_visualization_done_;
  std::atomic_bool images_received_;

  std::string              _uav_name_;
  bool                     currBatchProcessed;
  bool                     DEBUG;
  bool                     _visual_debug_;
  bool                     GUI;
  bool                     InvertedPoints;
  std::vector<cv::Mat>     currentImages;
  std::vector<bool>        currentImagesReceived;
  cv::Mat                  viewImage;
  std::thread              show_thread;
  std::thread              visualization_thread;
  std::vector<std::thread> process_threads;
  std::mutex               mutex_show;

  bool                     use_camera_for_visualization_;

  using image_callback_t = std::function<void (const sensor_msgs::ImageConstPtr&)>;
  std::vector<image_callback_t> imageCallbacks;
  std::vector<ros::Subscriber> imageSubscribers;

  using points_seen_callback_t = std::function<void (const mrs_msgs::Int32MultiArrayStampedConstPtr&)>;
  using points_seen_callback_legacy_t = std::function<void (const std_msgs::UInt32MultiArrayConstPtr&)>;
  std::vector<points_seen_callback_t> pointsSeenCallbacks;
  std::vector<points_seen_callback_t> sunPointsSeenCallbacks;
  std::vector<points_seen_callback_legacy_t> pointsSeenCallbacksLegacy;
  std::vector<ros::Subscriber> pointsSeenSubscribers;
  std::vector<ros::Subscriber> sunPointsSeenSubscribers;

  std::vector<ros::Publisher> blinkersSeenPublishers;
  std::vector<ros::Publisher> estimatedFrameratePublishers;
  //CHECK

  bool publishVisualization;
  int _visualization_rate_;
  image_transport::Publisher imPub;

  struct BlinkData {
    bool                     foundTarget;
    int                      currTrackerCount;
    std::vector<cv::Point3d> retrievedBlinkers;
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

  std::vector<BlinkData> blinkData;
  std::vector<HT4DBlinkerTracker*> ht3dbt_trackers;

  std::vector<std::vector<cv::Point>> sun_points_;
  std::mutex mutex_sun;

  bool returnFrequencies;

  std::vector<double> frequencySet;
  std::vector<double> periodSet;
  std::vector<double> periodBoundsTop;
  std::vector<double> periodBoundsBottom;

  std::vector<ros::Rate*> processSpinRates;
  int        processRate;

  int accumulatorLength;
  int pitchSteps;
  int yawSteps;
  int maxPixelShift;
  int _reasonable_radius_;
  int _nullify_radius_;

  int frequencyCount;

  bool _beacon_;

  ros::Time lastPointsTime;

  bool _legacy;
  double _legacy_delay;
  //CHECK

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
