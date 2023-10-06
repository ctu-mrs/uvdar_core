#include <ros/ros.h>
#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt32MultiArray.h>
#include <uvdar_core/Int32MultiArrayStamped.h>
#include <stdint.h>
#include <mutex>
#include <thread>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

namespace uvdar {

class MaskGenerator {
public:

  /* onInit() //{ */

  MaskGenerator(ros::NodeHandle &nh_) {
    initialized_ = false;

    nh_.param("uav_name", _uav_name_, std::string());
    nh_.param("mrs_id", _mrs_id_, std::string());
    nh_.param("DEBUG", DEBUG, bool(false));
    nh_.param("GUI", GUI, bool(false));
    if (GUI)
      ROS_INFO("[MaskGenerator]: GUI is true");
    else
      ROS_INFO("[MaskGenerator]: GUI is false");

    nh_.param("expansionSize", _expansion_size_, int(8));

    std::vector<std::string> _points_seen_topics;
    nh_.param("pointsSeenTopics", _points_seen_topics, _points_seen_topics);
    if (_points_seen_topics.empty()) {
      ROS_WARN("[MaskGenerator]: No topics of pointsSeen were supplied");
    }
    maskData.resize(_points_seen_topics.size());

    std::vector<std::string> _mask_file_names;
    nh_.param("maskFileNames", _mask_file_names, _mask_file_names);

    pointsSeenCallbacks.resize(_points_seen_topics.size());

    for (size_t i = 0; i < _points_seen_topics.size(); ++i) {
      maskData[i].camera_id = _mask_file_names[i];
      points_seen_callback_t callback = [imageIndex=i,this] (const uvdar_core::Int32MultiArrayStampedConstPtr& pointsMessage) { 
        InsertPoints(pointsMessage, imageIndex);
      };
      pointsSeenCallbacks[i] = callback;
      /* pointsSeenCallbacks.push_back(callback); */
      pointsSeenSubscribers.push_back(nh_.subscribe(_points_seen_topics[i], 1, &points_seen_callback_t::operator(), &pointsSeenCallbacks[i]));
      /* ROS_INFO("[MaskGenerator]: HERE B"); */
    }


    if (GUI) {
      show_thread  = std::thread(&MaskGenerator::ShowThread, this);
    }


    initialized_ = true;
    ROS_INFO("[MaskGenerator]: initialized");
  }

  //}


private:
  
  /* InsertPoints //{ */

  void InsertPoints(const uvdar_core::Int32MultiArrayStampedConstPtr& msg, size_t imageIndex) {
    if (!initialized_) return;

    std::scoped_lock lock(mutex_show);

    int                      countSeen;
    countSeen = (int)((msg)->layout.dim[0].size);

    MaskData& data = maskData[imageIndex];


    if (DEBUG) {
      ROS_INFO("Received contours: %d", countSeen);
    }
    if (countSeen < 1) {
      return;
    } 
    cv::Point current_point;
    for (int i = 0; i < countSeen; i++) {
      if (msg->data[(i * 3) + 2] <= 200) {
        current_point = cv::Point2d(msg->data[(i * 3)], msg->data[(i * 3) + 1]);
        cv::circle(data.mask_img, current_point, _expansion_size_, cv::Scalar(0), -1);
      }
    }
  }

  //}


  /* ShowThread() //{ */

  void ShowThread() {

    char key;
    while (ros::ok()) {
      if (initialized_){
        std::scoped_lock lock(mutex_show);

        if (GUI){
          if (GenerateVisualization() >= 0){
            cv::imshow("ocv_generated_mask_" + _uav_name_, viewImage);
          }
        }
      }
      key = cv::waitKey(1000.0 / 25.0);
      if ( key == 'm')
        break;
    }
    for (auto &data : maskData){
      cv::imwrite(ros::package::getPath("uvdar")+"/masks/"+_mrs_id_+"_"+data.camera_id+".bmp", data.mask_img);
    }

  }

  //}

  /* GenerateVisualization() //{ */

    int GenerateVisualization() {
      int image_count = maskData.size();
      if (image_count < 1){
        ROS_WARN("Image set size is 0!");
        return -1;
      }
      int image_width, image_height;
      image_width = maskData[0].mask_img.cols;
      image_height = maskData[0].mask_img.rows;

      viewImage = cv::Mat(
          image_height, 
          (image_width + 2) * image_count - 2, 
          CV_8UC3,
          cv::Scalar(255, 255, 255));


      if (viewImage.rows == 0 || viewImage.cols == 0) return -1;

      /* loop through all trackers and update the data //{ */

      cv::Mat tempColor;
      for (int imageIndex = 0; imageIndex < image_count; ++imageIndex) {
        MaskData& data = maskData[imageIndex];

        int differenceX = (image_width + 2) * imageIndex;

        cv::cvtColor(data.mask_img, tempColor, cv::COLOR_GRAY2RGB);
        tempColor.copyTo(viewImage(cv::Rect(differenceX,0,image_width,image_height)));
        }


      return 0;
  }

  //}


  /* attributes //{ */

  std::atomic_bool initialized_;

  std::string              _uav_name_;
  std::string              _mrs_id_;
  bool                     DEBUG;
  bool                     GUI;
  cv::Mat                  viewImage;
  std::thread              show_thread;
  std::mutex               mutex_show;

  using points_seen_callback_t = std::function<void (const uvdar_core::Int32MultiArrayStampedConstPtr&)>;
  std::vector<points_seen_callback_t> pointsSeenCallbacks;
  std::vector<ros::Subscriber> pointsSeenSubscribers;


  struct MaskData {
    std::string              camera_id;
    cv::Mat                  mask_img;

    MaskData(){
      camera_id = "";
      mask_img = cv::Mat(cv::Size(752,480),CV_8UC1,cv::Scalar(255));
    };
    MaskData(std::string i_camera_id){
      camera_id = i_camera_id;
      mask_img = cv::Mat(cv::Size(752,480),CV_8UC1,cv::Scalar(255));
    };
    ~MaskData(){};
  };

  std::vector<MaskData> maskData;

  int _expansion_size_;

  //}
};


} //namsepace uvdar

int main(int argc, char** argv) {
  ros::init(argc, argv, "uvdar_follower");
  ros::NodeHandle nh("~");
  uvdar::MaskGenerator mg(nh);

  ROS_INFO("Mask generator node initiated");

  ros::spin();

  return 0;
}

