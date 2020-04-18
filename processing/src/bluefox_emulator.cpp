#define camera_delay 0.50

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <stdint.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <OCamCalib/ocam_functions.h>
#include <functional>


#define index2d(X, Y) (oc_model.width * (Y) + (X))

namespace enc = sensor_msgs::image_encodings;

namespace uvdar {
class BluefoxEmulator : public nodelet::Nodelet{
public:

  void onInit() {

    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    char calib_path[400];
    std::string _calib_file;
    nh_.param("calib_file", _calib_file, std::string("calib_results_bf_uv_fe.txt"));
    sprintf(calib_path, "%s/include/OCamCalib/config/%s", ros::package::getPath("uvdar").c_str(),_calib_file.c_str());
    get_ocam_model(&oc_model, calib_path);

    std::vector<std::string> _camera_output_topics;
    nh_.param("cameraOutputTopics", _camera_output_topics, _camera_output_topics);
    /* std::vector<std::string> _virtual_points_topics; */
    /* nh_.param("virtualPointsTopics", _virtual_points_topics, _virtual_points_topics); */
    if (_camera_output_topics.empty()) {
      ROS_WARN("[BluefoxEmulator]: No topics of cameraOutputTopics were supplied");
    }
    // Create callbacks for each camera
    virtualPointsCallbacks.resize(_camera_output_topics.size());

    /* imageTransports.resize(_camera_output_topics.size()); */
    imagePublishers.resize(_camera_output_topics.size());
    imageTransport = new image_transport::ImageTransport(nh_);


    for (size_t i = 0; i < _camera_output_topics.size(); ++i) {

        virtual_points_callback_t callback = [imageIndex=i,this] (const sensor_msgs::PointCloudConstPtr& pointsMessage) { 
          drawPoints(pointsMessage, imageIndex);
        };
        virtualPointsCallbacks[i] = callback;
        virtualPointsSubscribers.push_back(nh_.subscribe(_camera_output_topics[i]+"_transfer", 1, &virtual_points_callback_t::operator(), &virtualPointsCallbacks[i]));
        imageData.push_back(ImageData(oc_model));
        imagePublishers[i] = imageTransport->advertise(_camera_output_topics[i], 1);
    }


    ROS_INFO("[Bluefox emulator]: initialized");
  }

  ~BluefoxEmulator() {
  }

private:

  /* drawPoints //{ */
  void drawPoints(const sensor_msgs::PointCloudConstPtr& msg_i, size_t imageIndex){
    sensor_msgs::ImagePtr msg_o;
    /* begin         = std::clock(); */
    //CHECK: Optimize the following
    for (int j = 0; j < imageData[imageIndex].outputImage.image.rows; j++) {
      for (int i = 0; i < imageData[imageIndex].outputImage.image.cols; i++) {
        if (imageData[imageIndex].outputImage.image.data[index2d(i, j)] != imageData[imageIndex].backgroundColour)
          (imageData[imageIndex].outputImage.image.data[index2d(i, j)] = imageData[imageIndex].backgroundColour);
      }
      }
      /* end_p         = std::clock(); */
      /* elapsedTime = double(end_p - begin) / CLOCKS_PER_SEC; */
      /* std::cout << "UV CAM: Wiping took : " << elapsedTime << " s" << std::endl; */
      /* for (std::pair<ignition::math::Pose3d,ignition::math::Pose3d>& i : buffer){ */
      for (auto point : msg_i->points){
        /* ROS_INFO_STREAM("Point: " << point); */
        cv::circle(imageData[imageIndex].outputImage.image, cv::Point2i(point.x, point.y), point.z, cv::Scalar(255), -1);
      }
      msg_o = imageData[imageIndex].outputImage.toImageMsg();
      msg_o->header.stamp = msg_i->header.stamp;
      imagePublishers[imageIndex].publish(msg_o);
      /* end         = std::clock(); */
      /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
      /* std::cout << "UV CAM: Drawing took : " << elapsedTime << " s" << std::endl; */
    }

    //}

private:
  struct ocam_model oc_model;
  using virtual_points_callback_t = std::function<void (const sensor_msgs::PointCloudConstPtr&)>;
  std::vector<virtual_points_callback_t> virtualPointsCallbacks;
  std::vector<ros::Subscriber> virtualPointsSubscribers;
  /* std::vector<image_transport::ImageTransport*> imageTransports; */
  image_transport::ImageTransport* imageTransport;
  std::vector<image_transport::Publisher>       imagePublishers;

  struct ImageData {
    cv_bridge::CvImage          outputImage;
    uchar                       backgroundColour;

    ImageData(struct ocam_model &oc_model_i){
      outputImage = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv::Mat(oc_model_i.height, oc_model_i.width, CV_8UC1, cv::Scalar(0)));
      backgroundColour = std::rand() % 100;
    };
    ~ImageData(){};
  };
  std::vector<ImageData> imageData;
};

/* int main(int argc, char** argv) { */
/*   ros::init(argc, argv, "uv_marker_detector"); */
/*   ros::NodeHandle nodeA; */
/*   UVDetector   uvd(nodeA); */

/*   ROS_INFO("UV LED marker detector node initiated"); */

/*   ros::spin(); */

/*   return 0; */
/* } */

} //namespace uvdar
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uvdar::BluefoxEmulator, nodelet::Nodelet)
