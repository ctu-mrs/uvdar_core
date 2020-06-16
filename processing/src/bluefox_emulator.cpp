#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <cv_bridge/cv_bridge.h>
/* #include <image_transport/image_transport.h> */
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <stdint.h>
#include <opencv2/core/core.hpp>
/* #include <opencv2/highgui/highgui.hpp> */
#include <OCamCalib/ocam_functions.h>
#include <functional>


#define index2d(X, Y) (oc_model.width * (Y) + (X))

namespace enc = sensor_msgs::image_encodings;

namespace uvdar {
class UVDARBluefoxEmulator : public nodelet::Nodelet{
public:

  void onInit() {

    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    mrs_lib::ParamLoader param_loader(nh_, "UVDARBluefoxEmulator");

    /* Load calibration files //{ */
    std::vector<std::string> _calib_files;
    param_loader.loadParam("calib_file", _calib_files, _calib_files);
    if (_calib_fies.empty()) {
      ROS_ERROR("[UVDARBluefoxEmulator]: No camera calibration files were supplied. You can even use \"default\" for the cameras, but some calibration files must be chosen. Returning.");
      return;
    }
    bool _calib_files_mrs_named;
    param_loader.loadParam("calib_files_mrs_named", _calib_files_mrs_named, bool(false));
    std::string file_name;
    oc_models.resize(_calib_files.size());
    int i=0;
    for (auto calib_file : _calib_files){
      if (calib_file == "default"){
        file_name = ros::package::getPath("uvdar")+"/include/OCamCalib/config/calib_results_bf_uv_fe.txt";
      }
      else if (_calib_files_mrs_named){
        file_name = ros::package::getPath("uvdar")+"/include/OCamCalib/config/"+calib_file;
      }
      else {
        file_name = calib_file;
      }

      get_ocam_model(&oc_models[i], file_name);
      ROS_INFO("[UVDARBluefoxEmulator]: Calibration parameters for virtual camera %d came from the file %s.", i, file_name);
      for (int j=0; j<oc_models[i].length_pol; j++){
        if (isnan(oc_models[i].pol[j])){
          ROS_ERROR("[UVDARBluefoxEmulator]: Calibration polynomial containts NaNs! Returning.");
          return;
        }
      }
      i++;
    }
    //}

    /* Load virtual camera image topics //{ */
    std::vector<std::string> _camera_output_topics;
    nh_.param("cameraOutputTopics", _camera_output_topics, _camera_output_topics);
    if (_camera_output_topics.empty()) {
      ROS_ERROR("[UVDARBluefoxEmulator]: No topics of cameraOutputTopics were supplied! Returning.");
      return;
    }
    if (_camera_output_topics.size() != _calib_files.size()){
      ROS_ERROR_STREAM("[UVDARBluefoxEmulator] The number of output topics (" << _camera_output_topics.size()  << ") does not match the number of camera calibration files (" << _calib_files.size() << ")! Returning.");
      return;
    }
    //}


    /* imageTransport = new image_transport::ImageTransport(nh_); */


    /* Create Gazebo metadata callbacks for each camera //{ */
    for (size_t i = 0; i < _camera_output_topics.size(); ++i) {
        virtual_points_callback_t callback = [imageIndex=i,this] (const sensor_msgs::PointCloudConstPtr& pointsMessage) { 
          drawPoints(pointsMessage, imageIndex);
        };
        cals_virtual_points_.push_back(callback);
        sub_virtual_points_.push_back(nh_.subscribe(_camera_output_topics[i]+"_transfer", 1, &virtual_points_callback_t::operator(), &cals_virtual_points_[i]));
        image_data.push_back(ImageData(oc_model));

        /* imageTransport = new image_transport::ImageTransport(nh_); */

        pub_image_.push_back(nh_.advertise<sensor_msgs::Image>(_camera_output_topics[i], 1));
        /* pub_image_.push_back(imageTransport->advertise(_camera_output_topics[i], 1)); */
    }
    //}


    ROS_INFO("[UVDARBluefoxEmulator]: initialized");
  }

  ~UVDARBluefoxEmulator() {
  }

private:

  /* drawPoints //{ */
  void drawPoints(const sensor_msgs::PointCloudConstPtr& msg_i, size_t imageIndex){
    sensor_msgs::ImagePtr msg_o;
    /* begin         = std::clock(); */
    //CHECK: Optimize the following
    for (int j = 0; j < image_data[imageIndex].outputImage.image.rows; j++) {
      for (int i = 0; i < image_data[imageIndex].outputImage.image.cols; i++) {
        if (image_data[imageIndex].outputImage.image.data[index2d(i, j)] != image_data[imageIndex].backgroundColour)
          (image_data[imageIndex].outputImage.image.data[index2d(i, j)] = image_data[imageIndex].backgroundColour);
      }
      }
      /* end_p         = std::clock(); */
      /* elapsedTime = double(end_p - begin) / CLOCKS_PER_SEC; */
      /* std::cout << "UV CAM: Wiping took : " << elapsedTime << " s" << std::endl; */
      /* for (std::pair<ignition::math::Pose3d,ignition::math::Pose3d>& i : buffer){ */
      for (auto point : msg_i->points){
        /* ROS_INFO_STREAM("Point: " << point); */
        cv::circle(image_data[imageIndex].outputImage.image, cv::Point2i(point.x, point.y), point.z, cv::Scalar(255), -1);
      }
      msg_o = image_data[imageIndex].outputImage.toImageMsg();
      msg_o->header.stamp = msg_i->header.stamp;
      pub_image_[imageIndex].publish(msg_o);
      /* end         = std::clock(); */
      /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
      /* std::cout << "UV CAM: Drawing took : " << elapsedTime << " s" << std::endl; */
    }

    //}

private:
  /* struct ocam_model oc_model; */
  std::vector<struct ocam_model> oc_models;
  using virtual_points_callback_t = std::function<void (const sensor_msgs::PointCloudConstPtr&)>;
  std::vector<virtual_points_callback_t> cals_virtual_points_;
  std::vector<ros::Subscriber> sub_virtual_points_;
  image_transport::ImageTransport* imageTransport;
  /* std::vector<image_transport::Publisher> pub_image_; */
  std::vector<ros::Publisher> pub_image_;

  struct ImageData {
    cv_bridge::CvImage          outputImage;
    uchar                       backgroundColour;

    ImageData(struct ocam_model &oc_model_i){
      outputImage = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv::Mat(oc_model_i.height, oc_model_i.width, CV_8UC1, cv::Scalar(0)));
      backgroundColour = std::rand() % 100;
      if ( (outputImage.image.cols < oc_model_i.width) || (outputImage.image.rows < oc_model_i.height) ){
        ROS_ERROR("[UVDARBluefoxEmulator]: Calibration polynomial contains NaNs, exiting.");
      }
    };
    ~ImageData(){};
  };
  std::vector<ImageData> image_data;
};

/* int main(int argc, char** argv) { */
/*   ros::init(argc, argv, "uv_marker_detector"); */
/*   ros::NodeHandle nodeA; */
/*   UVDARDetector   uvd(nodeA); */

/*   ROS_INFO("UV LED marker detector node initiated"); */

/*   ros::spin(); */

/*   return 0; */
/* } */

} //namespace uvdar
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uvdar::UVDARBluefoxEmulator, nodelet::Nodelet)
