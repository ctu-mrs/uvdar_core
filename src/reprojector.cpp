#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <mutex>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/Profiler.h>
#include <OCamCalib/ocam_functions.h>
#include <unscented/unscented.h>

#define filterCount 2
#define freq 3


class Reprojector{
  public:
  Reprojector(ros::NodeHandle nh){
    ros::NodeHandle pnh("~");
    gotImage = false;
    gotOdom = false;
    measSubscriber[0] = nh.subscribe("filteredPose1", 1, &Reprojector::odomCallback, this);
    measSubscriber[1] = nh.subscribe("filteredPose2", 1, &Reprojector::odomCallback, this);
    ImageSubscriber = nh.subscribe("camera", 1, &Reprojector::imageCallback, this);

    char calib_path[100];
    sprintf(calib_path, "%s/include/OCamCalib/config/calib_results.txt", ros::package::getPath("uvdar").c_str());
    get_ocam_model(&oc_model, calib_path);

    profiler = new mrs_lib::Profiler(pnh, "uvdar_reprojector_node", true);
    mrs_lib::ParamLoader param_loader(pnh, "uvdar_reprojector_node");

    param_loader.load_param("frame_camera", frame_camera);
    param_loader.load_param("frame_uvdar", frame_uvdar);
    param_loader.load_param("offline", offline);

    if (offline) {
      image_transport::ImageTransport it(nh);
      imPub = it.advertise("reprojection", 1);
    }

    listener = new tf2_ros::TransformListener(buffer);

    tf_timer       = nh.createTimer(ros::Rate(1), &Reprojector::tfTimer, this);

    im_timer = nh.createTimer(ros::Duration(1.0/fmax(freq,1.0)), &Reprojector::spin, this);
  }

  ~Reprojector(){
  }
  private:
  //methods
  void imageCallback(const sensor_msgs::ImageConstPtr& msg_Image){
    gotImage = true;
    std::scoped_lock lock(mtx_image);
    if (msg_Image == NULL) 
      return;
    cv_bridge::CvImageConstPtr image;
    if (sensor_msgs::image_encodings::isColor(msg_Image->encoding))
      image = cv_bridge::toCvShare(msg_Image, sensor_msgs::image_encodings::BGR8);
    else
      image = cv_bridge::toCvShare(msg_Image, sensor_msgs::image_encodings::MONO8);
    currImage = image->image;
  }

  void odomCallback(nav_msgs::OdometryPtr msg_odom){
    gotOdom = true;
    std::scoped_lock(mtx_odom);
    currOdom = *msg_odom;
  }

  void spin([[ maybe_unused ]] const ros::TimerEvent& unused){
    if (!gotOdom){
      ROS_INFO("Odometry not yet obtained, waiting...");
      return;
    }
    if (!gotImage){
      ROS_INFO("Image not yet obtained, waiting...");
      return;
    }
    if (!gotU2C) {
      ROS_INFO("Transform not yet obtained, waiting...");
      return;
    }

    if (offline)
      drawAndPublish();
    else
      drawAndShow();
  }

  void tfTimer(const ros::TimerEvent& event) {

    mrs_lib::Routine profiler_routine = profiler->createRoutine("tfTimer");

    try {
      {
        std::scoped_lock lock(mtx_tf);
        transformUvdar2Cam = buffer.lookupTransform(frame_camera, frame_uvdar, ros::Time(0), ros::Duration(2));
      }
      ROS_INFO_STREAM("[OpticFlow]: received uvdar2cam tf" << transformUvdar2Cam);


      // calculate the euler angles
      tf2::fromMsg(transformUvdar2Cam, TfU2C);

      gotU2C = true;
    }
    catch (tf2::TransformException ex) {
      ROS_ERROR("TF: %s", ex.what());
      ros::Duration(1.0).sleep();
      return;
    }

    try {
      {
        std::scoped_lock lock(mtx_tf);
        transformCam2Uvdar = buffer.lookupTransform(frame_uvdar, frame_camera, ros::Time(0), ros::Duration(2));
      }

      ROS_INFO_STREAM("[OpticFlow]: received cam2uvdar tf" << transformCam2Uvdar);

      tf2::fromMsg(transformCam2Uvdar, TfC2U);

      // calculate the euler angles
      gotC2U = true;
    }
    catch (tf2::TransformException ex) {
      ROS_ERROR("TF: %s", ex.what());
      ros::Duration(1.0).sleep();
      return;
    }

    // check whether we got everything we need
    // stop the timer if true
    if (gotC2U && gotU2C) {

      ROS_INFO("[OpticFlow]: got TFs, stopping tfTimer");

      delete listener;

      tf_timer.stop();
    }
  }

  void drawAndShow(){
    drawImage();
    cv::imshow("ocv_marked",viewImage);
    cv::waitKey(5);
  }

  void drawAndPublish(){
    sensor_msgs::ImagePtr msg;
    drawImage();
    msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, viewImage).toImageMsg();
    imPub.publish(msg);
  }

  void drawImage(){
    std::scoped_lock lock(mtx_image,mtx_odom);
    if (currImage.channels() == 1){
      cv::cvtColor(currImage,viewImage,cv::COLOR_GRAY2BGR);
    }
    else {
      viewImage = currImage;
    }

    unscented::measurement ms = getProjectedCovariance(currOdom);
    /* cv::ellipse(viewImage, getErrorEllipse(100,ms.x,ms.C), cv::Scalar::all(255), 2); */
    auto proj = getErrorEllipse(2.4477,ms.x,ms.C);
    auto rect = proj.boundingRect();

    int expand = 40;

    rect.height +=expand;
    rect.width +=expand;
    rect.x -=expand/2;
    rect.y -=expand/2;
    cv::ellipse(viewImage, proj, cv::Scalar(255,0,0), 2);
    cv::rectangle(viewImage, rect, cv::Scalar(0,0,255), 2);
    /* cv::circle(viewImage,getImPos(currOdom),getProjSize(currOdom),cv::Scalar(0,0,255)); */
  }

  cv::Point2i getImPos(nav_msgs::Odometry currOdom){
    return projectOmniIm(currOdom.pose.pose);
  }

  cv::Point2i projectOmniIm(geometry_msgs::Pose pose){
    tf2::Vector3 poseTrans = TfU2C*tf2::Vector3(pose.position.x,pose.position.y,pose.position.z);
    double pose3d[3];
    pose3d[0]= poseTrans.y();
    pose3d[1]= poseTrans.x();
    pose3d[2]=-poseTrans.z();
    /* ROS_INFO_STREAM("World: " << pose3d[0] << " : "<< pose3d[1] << " : " << pose3d[2]); */

    double imPos[2];
    world2cam(imPos, pose3d, &oc_model);
    /* ROS_INFO_STREAM("Reprojected: " << imPos[1] << " : "<< imPos[0]); */
    return cv::Point2i(imPos[1],imPos[0]);
  }

  Eigen::Vector2d projectOmniEigen(Eigen::Vector3d x,[[ maybe_unused ]] Eigen::VectorXd dummy){
    tf2::Vector3 poseTrans = TfU2C*tf2::Vector3(x[0],x[1],x[2]);
    double pose3d[3];
    pose3d[0]= poseTrans.y();
    pose3d[1]= poseTrans.x();
    pose3d[2]=-poseTrans.z();
    /* ROS_INFO_STREAM("World: " << pose3d[0] << " : "<< pose3d[1] << " : " << pose3d[2]); */

    double imPos[2];
    world2cam(imPos, pose3d, &oc_model);
    /* ROS_INFO_STREAM("Reprojected: " << imPos[1] << " : "<< imPos[0]); */
    return Eigen::Vector2d(imPos[1],imPos[0]);
  }

  unscented::measurement getProjectedCovariance(nav_msgs::Odometry currOdom){
    Eigen::Matrix3d Px;
    Eigen::Matrix2d Py;
    Eigen::Vector3d x;
    Eigen::Vector2d y;
    for (int i=0; i<3; i++){
      for (int j=0; j<3; j++){
          Px(j,i) = currOdom.pose.covariance[6*j+i] ;
      }
    }
    x = Eigen::Vector3d(
        currOdom.pose.pose.position.x,
        currOdom.pose.pose.position.y,
        currOdom.pose.pose.position.z
        );


    tf2::doTransform (x, x, transformUvdar2Cam);
    Eigen::Quaterniond temp;
    Eigen::fromMsg(transformUvdar2Cam.transform.rotation, temp);
    Px = temp.toRotationMatrix()*Px*temp.toRotationMatrix().transpose();


    boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd)> callback;
    callback=boost::bind(&Reprojector::projectOmniEigen,this,_1,_2);
    auto output =  unscented::unscentedTransform(x,Px, callback,-1.0,-1.0,-1.0);

    output.x = output.x.topRows(2);
    output.C = output.C.topLeftCorner(2, 2);

    return output;
  }
  cv::RotatedRect getErrorEllipse(double chisquare_val, Eigen::Vector2d mean, Eigen::Matrix2Xd C){

    std::cout << "rotated: " << C <<std::endl;

    //Get the eigenvalues and eigenvectors
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(C);
    if (eigensolver.info() != Eigen::Success)
      abort();
    auto eigenvalues  = eigensolver.eigenvalues();
    auto eigenvectors = eigensolver.eigenvectors();
    double angle = atan2(eigenvectors(0,1), eigenvectors(0,0));

    //Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
    if(angle < 0)
      angle += 6.28318530718;

    //Conver to degrees instead of radians
    angle = 180*angle/3.14159265359;

    //Calculate the size of the minor and major axes
    double halfmajoraxissize=chisquare_val*sqrt(eigenvalues(0));
    double halfminoraxissize=chisquare_val*sqrt(eigenvalues(1));

    ROS_INFO_STREAM("axes: " << halfmajoraxissize << " : " << halfminoraxissize);

    //Return the oriented ellipse
    //The -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
    cv::Point2d mean_cv(mean.x(),mean.y());
    return cv::RotatedRect(mean_cv, cv::Size2f(halfmajoraxissize, halfminoraxissize), -angle);

}

  
  //variables
  nav_msgs::Odometry currOdom;
  cv::Mat currImage, viewImage;

  std::mutex mtx_image, mtx_odom, mtx_tf;
  
  ros::Subscriber measSubscriber[filterCount];
  ros::Subscriber ImageSubscriber;

  ros::Timer tf_timer,im_timer;

  bool gotImage,gotOdom, gotU2C, gotC2U;

  bool offline;

  tf2_ros::Buffer                 buffer;
  tf2_ros::TransformListener*     listener;
  geometry_msgs::TransformStamped transformUvdar2Cam;
  geometry_msgs::TransformStamped transformCam2Uvdar;
  tf2::Stamped<tf2::Transform> TfC2U, TfU2C;

  mrs_lib::Profiler* profiler;
  std::string frame_camera, frame_uvdar;

  struct ocam_model oc_model;

  image_transport::Publisher imPub;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "uvdar_reprojector_node");
  ros::NodeHandle nh;
  Reprojector r(nh);


  ros::spin();


  return 0;
}
