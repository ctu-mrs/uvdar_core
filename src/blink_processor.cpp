#undef _DEBUG

#define camera_delay 0.50

#define min_frequency 3
#define max_frequency 36.0
#define boundary_ratio 0.7

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <ht3dbt/ht3d.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/MultiArrayDimension.h>
/* #include <std_msgs/UInt32MultiArray.h> */
#include <uvdar/Int32MultiArrayStamped.h>
#include <stdint.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>

namespace enc = sensor_msgs::image_encodings;

namespace uvdar {

class BlinkProcessor : public nodelet::Nodelet {
public:
  void onInit() {

    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    nh_.param("uav_name", uav_name, std::string());
    nh_.param("DEBUG", DEBUG, bool(false));
    nh_.param("VisDEBUG", VisDEBUG, bool(false));
    nh_.param("GUI", GUI, bool(false));
    if (GUI)
      ROS_INFO("[BlinkProcessor]: GUI is true");
    else
      ROS_INFO("[BlinkProcessor]: GUI is false");

    nh_.param("accumulatorLength", accumulatorLength, int(23));
    nh_.param("pitchSteps", pitchSteps, int(16));
    nh_.param("yawSteps", yawSteps, int(8));
    nh_.param("maxPixelShift", maxPixelShift, int(1));

    nh_.param("publishVisualization", publishVisualization, bool(false));
    nh_.param("visualizationRate", visualizatinRate, int(5));

    ht3dbt = new HT3DBlinkerTracker(accumulatorLength, pitchSteps, yawSteps, maxPixelShift, cv::Size(752, 480));

    ht3dbt->setDebug(DEBUG, VisDEBUG);
    /* ht3dbt->setDebug(true, VisDEBUG); */

    nh_.param("processRate", processRate, int(10));
    processSpinRate = new ros::Rate((double)processRate);

    nh_.param("returnFrequencies", returnFrequencies, bool(false));

    pointsSubscriber = nh_.subscribe("pointsSeen", 1, &BlinkProcessor::InsertPoints, this);
    pointsPublisher  = nh_.advertise<uvdar::Int32MultiArrayStamped>("blinkersSeen", 1);

    frameratePublisher  = nh_.advertise<std_msgs::Float32>("estimatedFramerate", 1);

    nh_.param("CameraImageCompressed", ImgCompressed, bool(false));
    nh_.param("InvertedPoints", InvertedPoints, bool(false));

    currImage = cv::Mat(cv::Size(752, 480), CV_8UC3, cv::Scalar(0, 0, 0));
    viewImage = currImage.clone();
    ImageSubscriber = nh_.subscribe("camera", 1, &BlinkProcessor::ProcessRaw, this);

    timeSum     = 0.0;
    timeSamples = 0;

    framerateEstim = 72;

    ROS_INFO("[BlinkProcessor]: dog");

    nh_.param("frequencyCount", frequencyCount, int(4));
    /* if (frequencyCount != 2){ */
    /*   ROS_ERROR("HEYYY"); */
    /*   return; */
    /* } */
    int tempFreq;

    if (frequencySet.size() < frequencyCount) {
      nh_.param("frequency1", tempFreq, int(6));
      frequencySet.push_back(double(tempFreq));
    }
    if (frequencySet.size() < frequencyCount) {
      nh_.param("frequency2", tempFreq, int(10));
      frequencySet.push_back(double(tempFreq));
    }
    if (frequencySet.size() < frequencyCount) {
      nh_.param("frequency3", tempFreq, int(15));
      frequencySet.push_back(double(tempFreq));
    }
    if (frequencySet.size() < frequencyCount) {
      nh_.param("frequency4", tempFreq, int(30));
      frequencySet.push_back(double(tempFreq));
    }
    if (frequencySet.size() < frequencyCount) {
      nh_.param("frequency5", tempFreq, int(8));
      frequencySet.push_back(double(tempFreq));
    }
    if (frequencySet.size() < frequencyCount) {
      nh_.param("frequency6", tempFreq, int(12));
      frequencySet.push_back(double(tempFreq));
    }

    prepareFrequencyClassifiers();

    process_thread = std::thread(&BlinkProcessor::ProcessThread, this);
    if (GUI) {
      show_thread  = std::thread(&BlinkProcessor::ShowThread, this);
    }

    if (publishVisualization) {
      visualization_thread = std::thread(&BlinkProcessor::VisualizeThread, this);
      image_transport::ImageTransport it(nh_);
      imPub = it.advertise("visualization", 1);
    }

    ROS_INFO("[BlinkProcessor]: initialized");
  }

  void prepareFrequencyClassifiers() {
    for (int i = 0; i < frequencySet.size(); i++) {
      periodSet.push_back(1.0 / frequencySet[i]);
    }
    for (int i = 0; i < frequencySet.size() - 1; i++) {
      periodBoundsBottom.push_back((periodSet[i] * (1.0 - boundary_ratio) + periodSet[i + 1] * boundary_ratio));
    }
    periodBoundsBottom.push_back(1.0 / max_frequency);


    periodBoundsTop.push_back(1.0 / min_frequency);
    for (int i = 1; i < frequencySet.size(); i++) {
      periodBoundsTop.push_back((periodSet[i] * boundary_ratio + periodSet[i - 1] * (1.0 - boundary_ratio)));
    }


    /* periodBoundsTop.back()           = 0.5 * periodSet.back() + 0.5 * periodSet[secondToLast]; */
    /* periodBoundsBottom[secondToLast] = 0.5 * periodSet.back() + 0.5 * periodSet[secondToLast]; */
  }

  ~BlinkProcessor() {
  }


private:
  void InsertPoints(const uvdar::Int32MultiArrayStampedConstPtr& msg) {
    int                      countSeen;
    std::vector<cv::Point2i> points;
    countSeen = (int)((msg)->layout.dim[0].size);


    timeSamples++;

    if (timeSamples >= 10) {
      ros::Time nowTime = ros::Time::now();

      framerateEstim = 10000000000.0 / (double)((nowTime - lastSignal).toNSec());
      if (DEBUG)
        std::cout << "Updating frequency: " << framerateEstim << " Hz" << std::endl;
      lastSignal = nowTime;
      ht3dbt->updateFramerate(framerateEstim);
      timeSamples = 0;
    }

    if (DEBUG) {
      ROS_INFO("Received contours: %d", countSeen);
    }
    if (countSeen < 1) {
      foundTarget = false;
    } else {
      foundTarget = true;
      lastSeen    = ros::Time::now();
    }

    /* { */
    cv::Point currPoint;
    /* bool      hasTwin; */
    for (int i = 0; i < countSeen; i++) {
      if (msg->data[(i * 3) + 2] <= 200) {
        if (InvertedPoints)
          currPoint = cv::Point2d(currImage.cols - msg->data[(i * 3)], currImage.rows - msg->data[(i * 3) + 1]);
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

    /* ROS_INFO("Here"); */
    lastPointsTime = msg->stamp;
    ht3dbt->insertFrame(points);
  }

  void ProcessThread() {
    std::vector<int>  msgdata;
    uvdar::Int32MultiArrayStamped msg;
    clock_t                    begin, end;
    double                     elapsedTime;
    processSpinRate->reset();
    for (;;) {
      if (ht3dbt->isCurrentBatchProcessed())
        continue;

      if (DEBUG)
        ROS_INFO("Processing accumulated points");

      begin = std::clock();
      ros::Time local_lastPointsTime = lastPointsTime;
      {
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
      for (int i = 0; i < retrievedBlinkers.size(); i++) {
        msgdata.push_back(retrievedBlinkers[i].x);
        msgdata.push_back(retrievedBlinkers[i].y);
        if (returnFrequencies)
          msgdata.push_back(retrievedBlinkers[i].z);
        else
          msgdata.push_back(findMatch(retrievedBlinkers[i].z));
      }

      msg.data = msgdata;
      pointsPublisher.publish(msg);

      std_msgs::Float32 msgFramerate;
      msgFramerate.data = framerateEstim;
      frameratePublisher.publish(msgFramerate);


      processSpinRate->sleep();
    }
  }


  int findMatch(double i_frequency) {
    double period = 1.0 / i_frequency;
    for (int i = 0; i < periodSet.size(); i++) {
      /* std::cout << period << " " <<  periodBoundsTop[i] << " " << periodBoundsBottom[i] << " " << periodSet[i] << std::endl; */
      if ((period > periodBoundsBottom[i]) && (period < periodBoundsTop[i])) {
        return i;
      }
    }
    return -1;
  }


  cv::Scalar rainbow(double value, double max) {
    unsigned char r, g, b;
    r = 255 * (std::max(0.0, (1 - (value / (max / 2.0)))));
    if (value < (max / 2.0))
      g = 255 * (std::max(0.0, value / (max / 2.0)));
    else
      g = 255 * (std::max(0.0, 1 - (value - (max / 2.0)) / (max / 2.0)));
    b   = 255 * (std::max(0.0, (((value - (max / 2.0)) / (max / 2.0)))));
    return cv::Scalar(b, g, r);
  }

  void VisualizeThread() {
    ros::Rate r(visualizatinRate);
    sensor_msgs::ImagePtr msg;
    while (ros::ok()) {
      r.sleep();
      currTrackerCount = ht3dbt->getTrackerCount();
      mutex_show.lock();
      viewImage = currImage;
      mutex_show.unlock();


      cv::circle(viewImage, cv::Point(10, 10), 5, cv::Scalar(255, 100, 0));
      cv::circle(viewImage, cv::Point(10, 25), 5, cv::Scalar(0, 50, 255));
      cv::circle(viewImage, cv::Point(10, 40), 5, cv::Scalar(0, 200, 255));
      cv::circle(viewImage, cv::Point(10, 55), 5, cv::Scalar(255, 0, 100));
      cv::putText(viewImage, cv::String("6"), cv::Point(15, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
      cv::putText(viewImage, cv::String("10"), cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
      cv::putText(viewImage, cv::String("15"), cv::Point(15, 45), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
      cv::putText(viewImage, cv::String("30"), cv::Point(15, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));


      int rbs = retrievedBlinkers.size();
      for (int i = 0; i < rbs; i++) {
        cv::Point center = cv::Point(retrievedBlinkers[i].x, retrievedBlinkers[i].y);


        int        freqIndex = findMatch(retrievedBlinkers[i].z);
        char       freqText[4];
        char       freqTextRefined[4];
        cv::Scalar markColor;
        sprintf(freqText, "%d", std::max((int)retrievedBlinkers[i].z, 0));
        cv::putText(viewImage, cv::String(freqText), center + cv::Point(-5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
        if (freqIndex >= 0) {
          switch (freqIndex) {
            case 0:
              markColor = cv::Scalar(255, 100, 0);
              break;
            case 1:
              markColor = cv::Scalar(0, 50, 255);
              break;
            case 2:
              markColor = cv::Scalar(0, 200, 255);
              break;
            case 3:
              markColor = cv::Scalar(255, 0, 100);
              break;
          }

          cv::circle(viewImage, center, 5, markColor);
        }
        double yaw, pitch, len;
        yaw              = ht3dbt->getYaw(i);
        pitch            = ht3dbt->getPitch(i);
        len              = cos(pitch);
        cv::Point target = center - (cv::Point(len * cos(yaw) * 20, len * sin(yaw) * 20.0));
        cv::line(viewImage, center, target, cv::Scalar(0, 0, 255), 2);
      }
      msg = cv_bridge::CvImage(std_msgs::Header(), enc::BGR8, viewImage).toImageMsg();
      imPub.publish(msg);



    }
  }
  void ShowThread() {
    /* ros::Time prevTime = ros::Time::now(); */
    /* ros::Time currTime = ros::Time::now(); */
    for (;;) {
      /* currTime = ros::Time::now(); */
      /* if ((currTime - prevTime) < ros::Duration(1.0 / 10.0)) { */
      /*   continue; */
      /* } */
      currTrackerCount = ht3dbt->getTrackerCount();
      mutex_show.lock();
      viewImage = currImage;
      mutex_show.unlock();


      cv::circle(viewImage, cv::Point(10, 10), 5, cv::Scalar(255, 100, 0));
      cv::circle(viewImage, cv::Point(10, 25), 5, cv::Scalar(0, 50, 255));
      cv::circle(viewImage, cv::Point(10, 40), 5, cv::Scalar(0, 200, 255));
      cv::circle(viewImage, cv::Point(10, 55), 5, cv::Scalar(255, 0, 100));
      cv::putText(viewImage, cv::String("6"), cv::Point(15, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
      cv::putText(viewImage, cv::String("10"), cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
      cv::putText(viewImage, cv::String("15"), cv::Point(15, 45), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
      cv::putText(viewImage, cv::String("30"), cv::Point(15, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));


      int rbs = retrievedBlinkers.size();
      for (int i = 0; i < rbs; i++) {
        cv::Point center = cv::Point(retrievedBlinkers[i].x, retrievedBlinkers[i].y);


        /* std::cout << "f: " << retrievedBlinkers[i].z << std::endl; */
        int        freqIndex = findMatch(retrievedBlinkers[i].z);
        char       freqText[4];
        char       freqTextRefined[4];
        cv::Scalar markColor;
        /* sprintf(freqText,"%d",std::max((int)retrievedBlinkers[i].z,0)); */
        sprintf(freqText, "%d", std::max((int)retrievedBlinkers[i].z, 0));
        cv::putText(viewImage, cv::String(freqText), center + cv::Point(-5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
        if (freqIndex >= 0) {
          /* sprintf(freqTextRefined,"%d",(int)frequencySet[freqIndex]); */
          /* cv::putText(viewImage, cv::String(freqTextRefined), cv::Point(center.x, center.y+10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255)); */
          switch (freqIndex) {
            case 0:
              markColor = cv::Scalar(255, 100, 0);
              break;
            case 1:
              markColor = cv::Scalar(0, 50, 255);
              break;
            case 2:
              markColor = cv::Scalar(0, 200, 255);
              break;
            case 3:
              markColor = cv::Scalar(255, 0, 100);
              break;
          }

          cv::circle(viewImage, center, 5, markColor);
        }
        double yaw, pitch, len;
        yaw              = ht3dbt->getYaw(i);
        pitch            = ht3dbt->getPitch(i);
        len              = cos(pitch);
        cv::Point target = center - (cv::Point(len * cos(yaw) * 20, len * sin(yaw) * 20.0));
        cv::line(viewImage, center, target, cv::Scalar(0, 0, 255), 2);
      }
      /* cv::Scalar currColor; */
      /* for (int j = 0; j<256;j++){ */
      /*   currColor = rainbow(double(j),255.0); */
      /*   viewImage.at<cv::Vec3b>(viewImage.rows-1, j) = cv::Vec3b(currColor[0],currColor[1],currColor[2]); */
      /* } */
      /* ROS_INFO("W:%d, H:%d", viewImage.size().width,viewImage.size().height); */
      if (!VisDEBUG && GUI)
        cv::imshow("ocv_blink_retrieval_"+uav_name, viewImage);

      if (!VisDEBUG)
        cv::waitKey(1000.0 / 25.0);
      /* prevTime = currTime; */
    }
  }

  void ProcessCompressed(const sensor_msgs::CompressedImageConstPtr& image_msg) {
    cv_bridge::CvImagePtr image;
    if (image_msg != NULL) {
      image = cv_bridge::toCvCopy(image_msg, enc::RGB8);
      mutex_show.lock();
      { currImage = image->image; }
      mutex_show.unlock();
    }
  }

  void ProcessRaw(const sensor_msgs::ImageConstPtr& image_msg) {
    cv_bridge::CvImagePtr image;
    image = cv_bridge::toCvCopy(image_msg, enc::RGB8);
    mutex_show.lock();
    { currImage = image->image; }
    mutex_show.unlock();
  }

  std::string              uav_name;
  bool                     currBatchProcessed;
  bool                     DEBUG;
  bool                     VisDEBUG;
  bool                     GUI;
  bool                     ImgCompressed;
  bool                     InvertedPoints;
  cv::Mat                  currImage;
  cv::Mat                  viewImage;
  std::thread              show_thread;
  std::thread              visualization_thread;
  std::thread              process_thread;
  std::mutex               mutex_show;
  ros::Subscriber          pointsSubscriber;
  ros::Publisher           pointsPublisher;
  ros::Publisher           frameratePublisher;
  ros::Subscriber          ImageSubscriber;
  bool publishVisualization;
  int visualizatinRate;
  image_transport::Publisher imPub;
  bool                     foundTarget;
  int                      currTrackerCount;
  std::vector<cv::Point3d> retrievedBlinkers;
  ros::Time                lastSeen;
  ros::Time                lastSignal;
  int                      timeSamples;
  double                   timeSum;

  bool returnFrequencies;

  double framerateEstim;

  std::vector<double> frequencySet;
  std::vector<double> periodSet;
  std::vector<double> periodBoundsTop;
  std::vector<double> periodBoundsBottom;

  HT3DBlinkerTracker* ht3dbt;

  ros::Rate* processSpinRate;
  int        processRate;

  int accumulatorLength;
  int pitchSteps;
  int yawSteps;
  int maxPixelShift;

  int frequencyCount;

  ros::Time lastPointsTime;
};

/* int main(int argc, char** argv) { */
/*   ros::init(argc, argv, "blink_processor"); */
/*   ROS_INFO("Starting the Blink processor node"); */
/*   ros::NodeHandle nodeA; */
/*   BlinkProcessor  bp(nodeA); */

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
PLUGINLIB_EXPORT_CLASS(uvdar::BlinkProcessor, nodelet::Nodelet)
