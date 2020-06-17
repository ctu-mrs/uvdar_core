#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <stdint.h>
#include <opencv2/core/core.hpp>
#include <OCamCalib/ocam_functions.h>
#include <functional>


#define index2d(X, Y) (_oc_models_.at(image_index).width * (Y) + (X))

namespace enc = sensor_msgs::image_encodings;

namespace uvdar {
class UVDARBluefoxEmulator : public nodelet::Nodelet{
public:


  /**
   * @brief Initialization of the bluefox camera emulator
   */
  /* onInit //{ */
  void onInit() {

    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    mrs_lib::ParamLoader param_loader(nh_, "UVDARBluefoxEmulator");

    /* Load calibration files //{ */
    std::vector<std::string> _calib_files;
    param_loader.loadParam("calib_files", _calib_files, _calib_files);
    if (_calib_files.empty()) {
      ROS_ERROR("[UVDARBluefoxEmulator]: No camera calibration files were supplied. You can even use \"default\" for the cameras, but no calibration is not permissible. Returning.");
      return;
    }
    bool _calib_files_mrs_named;
    param_loader.loadParam("calib_files_mrs_named", _calib_files_mrs_named, bool(false));
    std::string file_name;
    _oc_models_.resize(_calib_files.size());
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

      get_ocam_model(&_oc_models_.at(i), (char*)(file_name.c_str()));
      ROS_INFO_STREAM("[UVDARBluefoxEmulator]: Calibration parameters for virtual camera " << i << " came from the file " <<  file_name);
      for (int j=0; j<_oc_models_.at(i).length_pol; j++){
        if (isnan(_oc_models_.at(i).pol[j])){
          ROS_ERROR("[UVDARBluefoxEmulator]: Calibration polynomial containts NaNs! Returning.");
          return;
        }
      }
      i++;
    }
    //}

    /* Load virtual camera image topics //{ */
    std::vector<std::string> _camera_output_topics;
    nh_.param("camera_output_topics", _camera_output_topics, _camera_output_topics);
    if (_camera_output_topics.empty()) {
      ROS_ERROR("[UVDARBluefoxEmulator]: No topics of camera_output_topics were supplied! Returning.");
      return;
    }
    if (_camera_output_topics.size() != _calib_files.size()){
      ROS_ERROR_STREAM("[UVDARBluefoxEmulator] The number of output topics (" << _camera_output_topics.size()  << ") does not match the number of camera calibration files (" << _calib_files.size() << ")! Returning.");
      return;
    }
    //}




    /* Create Gazebo metadata callbacks for each camera //{ */
    /* cals_virtual_points_.reserve(_camera_output_topics.size()); */
    for (size_t i = 0; i < _camera_output_topics.size(); ++i) {
        virtual_points_callback_t callback = [image_index=i,this] (const sensor_msgs::PointCloudConstPtr& points_message) { 
          drawPoints(points_message, image_index);
        };
        cals_virtual_points_.push_back(callback);
        sub_virtual_points_.push_back(nh_.subscribe(_camera_output_topics.at(i)+"_transfer", 1, cals_virtual_points_.at(i)));
        image_data.push_back(ImageData(_oc_models_.at(i)));


        pub_image_.push_back(nh_.advertise<sensor_msgs::Image>(_camera_output_topics.at(i), 1));
    }
    //}


    initialized_ = true;
    ROS_INFO("[UVDARBluefoxEmulator]: initialized");
  }
  //}

  /* destructor //{ */
  ~UVDARBluefoxEmulator() {
  }
  //}

private:


  /**
   * @brief callback to the metadata from Gazebo plugin - these are used as image points to draw into the virtual camera image
   *
   * @param msg_i - the input message; point cloud where x,y of the points are the image coordinates and z is the size of the blob
   * @param image_index - the index of the current camera image
   */
  /* drawPoints //{ */
  void drawPoints(const sensor_msgs::PointCloudConstPtr& msg_i, size_t image_index){
    if (initialized_) {
      sensor_msgs::ImagePtr msg_o;
      for (int j = 0; j < image_data.at(image_index).outputImage.image.rows; j++) {
        for (int i = 0; i < image_data.at(image_index).outputImage.image.cols; i++) {
          if (image_data.at(image_index).outputImage.image.data[index2d(i, j)] != image_data.at(image_index).backgroundColour)
            (image_data.at(image_index).outputImage.image.data[index2d(i, j)] = image_data.at(image_index).backgroundColour);
        }
      }
      for (auto point : msg_i->points){
        cv::circle(image_data.at(image_index).outputImage.image, cv::Point2i(point.x, point.y), point.z, cv::Scalar(255), -1);
      }
      msg_o = image_data.at(image_index).outputImage.toImageMsg();
      msg_o->header.stamp = msg_i->header.stamp;
      pub_image_.at(image_index).publish(msg_o);
    }
  }

  //}

private:
  bool initialized_ = false;
  std::vector<struct ocam_model> _oc_models_;
  using virtual_points_callback_t = boost::function<void (const sensor_msgs::PointCloudConstPtr&)>;
  std::vector<virtual_points_callback_t> cals_virtual_points_;
  std::vector<ros::Subscriber> sub_virtual_points_;
  std::vector<ros::Publisher> pub_image_;

  /**
   * @brief - A structure used for storing data on a given virtual camera image
   */
  /* ImageData structure //{ */
  struct ImageData {
    cv_bridge::CvImage          outputImage;
    uchar                       backgroundColour;

    /**
     * @brief - constructor
     *
     * @param oc_model_i - calibration parameters of the virtual camera
     */
    ImageData(struct ocam_model &oc_model_i){
      outputImage = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv::Mat(oc_model_i.height, oc_model_i.width, CV_8UC1, cv::Scalar(0)));
      backgroundColour = std::rand() % 100;
      if ( (outputImage.image.cols < oc_model_i.width) || (outputImage.image.rows < oc_model_i.height) ){
        ROS_ERROR("[UVDARBluefoxEmulator]: Calibration polynomial contains NaNs, exiting.");
      }
    };
    ~ImageData(){};
  };
  //}
  std::vector<ImageData> image_data;
};

} //namespace uvdar
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uvdar::UVDARBluefoxEmulator, nodelet::Nodelet)
