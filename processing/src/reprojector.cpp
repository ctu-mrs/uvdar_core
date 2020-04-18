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
#include <mrs_lib/param_loader.h>
#include <mrs_lib/profiler.h>
#include <OCamCalib/ocam_functions.h>
#include <unscented/unscented.h>
#include <uvdar/ROIVector.h>
#include <uvdar/DistRangeVector.h>

#define DEBUG false

#define filterCount 2
#define freq 3.0
#define decayTime 0.8
#define odomBufferLength 2.0
#define armLength 0.2775
#define propRadius 0.1143

#define maxCamAngle 1.61443 //185 deg / 2

namespace uvdar {

class Reprojector{
  public:
    Reprojector(ros::NodeHandle nh){
      ros::NodeHandle pnh("~");
      gotImage = false;
      for (int i=0;i<filterCount;i++){
        gotOdom[i] = false;
      }

      measSubscriber[0] = nh.subscribe("filteredPose1", 1, &Reprojector::odomCallback, this);
      measSubscriber[1] = nh.subscribe("filteredPose2", 1, &Reprojector::odomCallback, this);
      ImageSubscriber = nh.subscribe("camera", 1, &Reprojector::imageCallback, this);


      profiler = new mrs_lib::Profiler(pnh, "uvdar_reprojector_node", true);
      mrs_lib::ParamLoader param_loader(pnh, "uvdar_reprojector_node");

      param_loader.loadParam("frame_camera", _frame_camera);
      param_loader.loadParam("frame_estimate", _frame_estimate);
      param_loader.loadParam("offline", _offline);
      param_loader.loadParam("gui", _gui);
      param_loader.loadParam("calib_file", _calib_file);
      param_loader.loadParam("publish_boxes", _publish_boxes);
      param_loader.loadParam("no_draw", _no_draw_, bool(false));
      /* ROS_INFO_STREAM( "/include/OCamCalib/config/"); */
      /* ROS_INFO_STREAM(ros::package::getPath("uvdar")); */
      /* ROS_INFO_STREAM(_calib_file); */
      char calib_path[600];
      sprintf(calib_path, "%s/include/OCamCalib/config/%s", ros::package::getPath("uvdar").c_str(),_calib_file.c_str());
      get_ocam_model(&oc_model, calib_path);


      if (!_offline) {
        image_transport::ImageTransport it(nh);
        imPub = it.advertise("reprojection", 1000);
      }
      else { 
        cv::namedWindow("ocv_marked");
      }

      if (_publish_boxes){
        sensor_msgs::RegionOfInterest roi;
        roi.x_offset = 0;
        roi.y_offset = 0;
        roi.width = 0;
        roi.height = 0;
        rois_empty_.ROIs.push_back(roi);
        rois_empty_.ROIs.push_back(roi);
        uvdar::DistRange dist;
        dist.distance = 0;
        dist.stddev = 0;
        dists_empty_.distRanges.push_back(dist);
        dists_empty_.distRanges.push_back(dist);
        roiPublisher = nh.advertise<ROIVector>("estimatedROIs", 1);
        distPublisher = nh.advertise<DistRangeVector>("estimatedDistances", 1);

      }


      listener = new tf2_ros::TransformListener(buffer);

      /* tf_timer       = nh.createTimer(ros::Rate(10), &Reprojector::tfTimer, this); */

      im_timer = nh.createTimer(ros::Duration(1.0/fmax(freq,1.0)), &Reprojector::spin, this);
    }

    ~Reprojector(){
    }
  private:
    //methods
    void imageCallback(const sensor_msgs::ImageConstPtr& msg_Image){
      std::scoped_lock lock(mtx_image);

      if (msg_Image == NULL) 
        return;
      cv_bridge::CvImageConstPtr image;
      if (sensor_msgs::image_encodings::isColor(msg_Image->encoding))
        image = cv_bridge::toCvShare(msg_Image, sensor_msgs::image_encodings::BGR8);
      else
        image = cv_bridge::toCvShare(msg_Image, sensor_msgs::image_encodings::MONO8);
      currImage = image->image;
      currImgTime = image->header.stamp;

      gotImage = true;
    }

    void odomCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event){

      ros::M_string mhdr = event.getConnectionHeader();
      std::string topic = mhdr["topic"];
      /* ROS_INFO_STREAM("top: " << topic); */
      int target =(int)(topic.back())-49;
      auto meas = event.getMessage();

      gotOdom[target] = true;
      lastMeasurement[target] = meas->header.stamp;
      std::scoped_lock(mtx_odom);
      odomBuffer[target].push_back(*meas);
      while ((ros::Time::now()-odomBuffer[target].front().header.stamp).toSec()>odomBufferLength){
        odomBuffer[target].erase(odomBuffer[target].begin());
      }
    }

    nav_msgs::Odometry getClosestTime(std::vector<nav_msgs::Odometry> odom_vec,ros::Time compTime, ros::Duration &shortest_diff){
      shortest_diff.fromSec(10);
      ros::Duration curr_diff;
      nav_msgs::Odometry *best_odom;
      /* unsigned int best_index; */
      for (auto& odom : odom_vec){
        curr_diff = (compTime-odom.header.stamp);
        if (abs(curr_diff.toSec()) < abs(shortest_diff.toSec())){
          shortest_diff = curr_diff;
          best_odom =  &odom;
        }
      }
      /* if (DEBUG) */
        ROS_INFO_STREAM("Closest odom out of " <<(int)(odom_vec.size())<< " at " << shortest_diff.toSec() << "s");
      return *best_odom;
    }

    void spin([[ maybe_unused ]] const ros::TimerEvent& unused){
      end         = std::clock();
      end_r         = ros::Time::now();
      elapsedTime = double(end - begin) / CLOCKS_PER_SEC;
      elapsedTime_r = end_r-begin_r;
      if (DEBUG){
        std::cout << "Spin time: " << elapsedTime << " s" << std::endl;
        std::cout << "Ros spin time: " << elapsedTime_r.toSec() << " s" << std::endl;
      }
      begin = std::clock();
      begin_r = ros::Time::now();
      /* ROS_INFO_STREAM("HEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEY"); */
      if (!gotImage){
        ROS_INFO("Image not yet obtained, waiting...");
        return;
      }
      /* if (!gotU2C) { */
      /*   ROS_INFO("Transform not yet obtained, waiting..."); */
      /*   return; */
      /* } */

      if (!_offline)
        drawAndPublish(_no_draw_);
      else if (_gui)
        drawAndShow();
      else
        drawImage(false);

    }

    void getE2C(ros::Time stamp){
      try {
        {
          transformEstim2Cam = buffer.lookupTransform(_frame_camera, _frame_estimate, stamp, ros::Duration(0.05));
          tf2::fromMsg(transformEstim2Cam, TfE2C);
          if (DEBUG)
            ROS_INFO_STREAM("[Reprojector]: received estim2cam tf: " << transformEstim2Cam);
        }
      }
      catch (tf2::TransformException& ex) {
        ROS_ERROR("[Reprojector]: %s", ex.what());
      }
    }

    void tfTimer(const ros::TimerEvent& event) {
      if (DEBUG)
      ROS_INFO_STREAM("[Reprojector]: looping tf timer");

      mrs_lib::Routine profiler_routine = profiler->createRoutine("tfTimer");

      try {
        {
          std::scoped_lock lock(mtx_tf);
          transformEstim2Cam = buffer.lookupTransform(_frame_camera, _frame_estimate, ros::Time(0), ros::Duration(0));
          tf2::fromMsg(transformEstim2Cam, TfE2C);
          if (DEBUG)
            ROS_INFO_STREAM("[Reprojector]: received Estim2cam tf: " << transformEstim2Cam);
        }
        gotU2C = true;
      }
      catch (tf2::TransformException& ex) {
        ROS_ERROR("[Reprojector]: %s", ex.what());
        /* ros::Duration(1.0).sleep(); */
        gotU2C = false;
      }

      try {
        {
          std::scoped_lock lock(mtx_tf);
          transformCam2Estim = buffer.lookupTransform(_frame_estimate, _frame_camera, ros::Time(0), ros::Duration(0));
          tf2::fromMsg(transformCam2Estim, TfC2E);
          if (DEBUG)
            ROS_INFO_STREAM("[Reprojector]: received cam2Estim tf: " << transformCam2Estim);
        }


        gotC2U = true;
      }
      catch (tf2::TransformException& ex) {
        ROS_ERROR("[Reprojector]: %s", ex.what());
        /* ros::Duration(1.0).sleep(); */
        gotC2U = false;

      }

      // check whether we got everything we need
      // stop the timer if true
      /* if (gotC2U && gotU2C) { */
      if (!(ros::ok())) {

        /* ROS_INFO("[Reprojector]: got TFs, stopping tfTimer"); */
        ROS_INFO("[Reprojector]: stopping tfTimer");

        delete listener;

        tf_timer.stop();
      }
    }

    void drawAndShow(){
      drawImage(false);
      cv::imshow("ocv_marked",viewImage);
      cv::waitKey(10);
    }

    void drawAndPublish(bool no_draw){
      sensor_msgs::ImagePtr msg;
      drawImage(no_draw);
      msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, viewImage).toImageMsg();
      imPub.publish(msg);
      if (_publish_boxes){
        roiPublisher.publish(rois);
        distPublisher.publish(dists);
      }
    }

    void drawImage(bool no_draw){
      {
        std::scoped_lock lock(mtx_image);
        if (currImage.channels() == 1){
          cv::cvtColor(currImage,viewImage,cv::COLOR_GRAY2BGR);
        }
        else {
          viewImage = currImage;
        }
      }
      {
        std::scoped_lock lock(mtx_odom);

        if (_publish_boxes){
          rois = rois_empty_;
          dists = dists_empty_;
          rois.stamp = currImgTime;
          dists.stamp = currImgTime;
        }
        ros::Duration shortestDiff;
        bool stale = false;
        for (int target=0;target<filterCount;target++){

          if (DEBUG)
            ROS_INFO_STREAM("Last measurement of target [" << target << "] was " << ros::Duration(ros::Time::now()-lastMeasurement[target]).toSec() << "s ago.");

          if (ros::Duration(ros::Time::now()-lastMeasurement[target]).toSec()>decayTime)
            gotOdom[target] = false;

          if (!(gotOdom[target])){
            ROS_INFO("Estimate %d not yet obtained, waiting...",target+1);
            continue;
          }

          cv::Scalar color;
          if (!stale)
            switch (target) {
            case 0:
              color=cv::Scalar(255,255,0);
              break;
            case 1:
              color=cv::Scalar(255,0,255);
          }
          else
            color=cv::Scalar(0,0,0);
          nav_msgs::Odometry currOdom = getClosestTime(odomBuffer[target],currImgTime, shortestDiff);
          if (shortestDiff.toSec() > 0.1)
            stale = true;

          Eigen::Vector3d x;
          Eigen::Matrix3d Px;
          unscented::measurement ms = getProjectedCovariance(currOdom, Px,x);
          if (ms.x.array().isNaN().any()){
            continue;
          }

          unscented::measurement distrange = getProjectedCovarianceDist(currOdom, Px,x);

          /* cv::ellipse(viewImage, getErrorEllipse(100,ms.x,ms.C), cv::Scalar::all(255), 2); */
          auto proj = getErrorEllipse(1,ms.x,ms.C);

          unscented::measurement ms_expand = getProjectedCovarianceExpand(currOdom, armLength+propRadius);
          if (ms_expand.x.array().isNaN().any()){
            continue;
          }
          auto projExpand = getErrorEllipse(1,ms_expand.x,ms_expand.C);
          /* auto proj = getErrorEllipse(2.4477,ms.x,ms.C); */
          auto rect = projExpand.boundingRect();


          /* int expand = (int)round(100000.0/(oc_model.invpol[0]*getMinDistance(x,Px))); */

          /* ROS_INFO_STREAM("Expansion size: "<< expand << " invpol[0]: " << oc_model.invpol[0] << " distance: " << getMinDistance(x,Px)); */

          /* rect.height +=expand; */
          /* rect.width +=expand; */
          /* rect.x -=expand/2; */
          /* rect.y -=expand/2; */
          if (!no_draw){
            cv::ellipse(viewImage, proj, color, 2);
            ROS_INFO_STREAM("drawing rectangle:" << rect );
            cv::rectangle(viewImage, rect, color, 2);
          }

          if (_publish_boxes){
            sensor_msgs::RegionOfInterest roi;
            roi.x_offset = rect.x;
            roi.y_offset = rect.y;
            roi.height = rect.height;
            roi.width = rect.width;
            rois.ROIs[target] = roi;
            uvdar::DistRange dr;
            dr.distance = distrange.x(0);
            dr.stddev = distrange.C(0);
            dists.distRanges[target] = dr;
          }
          /* cv::circle(viewImage,getImPos(currOdom),getProjSize(currOdom),cv::Scalar(0,0,255)); */
        }
      }
    }

    /* cv::Point2i getImPos(nav_msgs::Odometry currOdom, ros::Time stamp){ */
    /*   return projectOmniIm(currOdom.pose.pose,stamp); */
    /* } */

    /* cv::Point2i projectOmniIm(geometry_msgs::Pose pose, ros::Time stamp){ */
    /*   tf2::Vector3 poseTrans; */
    /*   getE2C(stamp); */
    /*   poseTrans = TfE2C*tf2::Vector3(pose.position.x,pose.position.y,pose.position.z); */
    /*   double pose3d[3]; */
    /*   pose3d[0]= poseTrans.y(); */
    /*   pose3d[1]= poseTrans.x(); */
    /*   pose3d[2]=-poseTrans.z(); */
    /*   /1* ROS_INFO_STREAM("World: " << pose3d[0] << " : "<< pose3d[1] << " : " << pose3d[2]); *1/ */

    /*   double imPos[2]; */
    /*   world2cam(imPos, pose3d, &oc_model); */
    /*   /1* ROS_INFO_STREAM("Reprojected: " << imPos[1] << " : "<< imPos[0]); *1/ */
    /*   return cv::Point2i(imPos[1],imPos[0]); */
    /* } */

    Eigen::Vector2d projectOmniEigen(Eigen::Vector3d x,[[ maybe_unused ]] Eigen::VectorXd dummy, [[ maybe_unused ]] int dummyIndex){
      if (atan2(sqrt(x[0]*x[0]+x[1]*x[1]),x[2])>maxCamAngle)
        return Eigen::Vector2d(std::nan(""),std::nan(""));
      /* if (x[2]<0) return Eigen::Vector2d(std::nan(""),std::nan("")); */
      /* tf2::Vector3 poseTrans; */
      tf2::Vector3 pose;
      pose = tf2::Vector3(x[0],x[1],x[2]);
      ROS_INFO_STREAM("x: " << x);
      double pose3d[3];
      /* pose3d[0]= poseTrans.y(); */
      /* pose3d[1]= poseTrans.x(); */
      /* pose3d[2]=-poseTrans.z(); */
      pose3d[0]= pose.y();
      pose3d[1]= pose.x();
      pose3d[2]=-pose.z();
      /* ROS_INFO_STREAM("World: " << pose3d[0] << " : "<< pose3d[1] << " : " << pose3d[2]); */

      double imPos[2];
      world2cam(imPos, pose3d, &oc_model);
      /* ROS_INFO_STREAM("Reprojected: " << imPos[1] << " : "<< imPos[0]); */
      ROS_INFO_STREAM("y: " << Eigen::Vector2d(imPos[1],imPos[0]));
      return Eigen::Vector2d(imPos[1],imPos[0]);
    }

    Eigen::VectorXd distOmniEigen(Eigen::Vector3d x,[[ maybe_unused ]] Eigen::VectorXd dummy, [[ maybe_unused ]] int dummyIndex){
      if (atan2(sqrt(x[0]*x[0]+x[1]*x[1]),x[2])>maxCamAngle){
        Eigen::VectorXd retvaldummy(1);
        retvaldummy(0) = (std::nan(""));
        return retvaldummy;
      }
      Eigen::Vector3d pose = Eigen::Vector3d(x[0],x[1],x[2]);
      Eigen::VectorXd output(1);
      /* Eigen::VectorXd output; = Eigen::Zero(1); */
      output(0) = pose.norm();
      return output;
    }

    unscented::measurement getProjectedCovariance(nav_msgs::Odometry currOdom,Eigen::Matrix3d &Px,Eigen::Vector3d &x){
      /* Eigen::Matrix3d Px; */
      Eigen::Matrix2d Py;
      /* Eigen::Vector3d x; */
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


      getE2C(currImgTime);
      tf2::doTransform (x, x, transformEstim2Cam);
      Eigen::Quaterniond temp;
      Eigen::fromMsg(transformEstim2Cam.transform.rotation, temp);
      Px = temp.toRotationMatrix()*Px*temp.toRotationMatrix().transpose();


      boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd,int)> callback;
      callback=boost::bind(&Reprojector::projectOmniEigen,this,_1,_2,_3);
      auto output =  unscented::unscentedTransform(x,Px, callback,-1.0,-1.0,-1.0);

      output.x = output.x.topRows(2);
      output.C = output.C.topLeftCorner(2, 2);

      return output;
    }

    unscented::measurement getProjectedCovarianceDist(nav_msgs::Odometry currOdom,Eigen::Matrix3d &Px,Eigen::Vector3d &x){
      /* Eigen::Matrix3d Px; */
      Eigen::Matrix2d Py;
      /* Eigen::Vector3d x; */
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


      getE2C(currImgTime);
      tf2::doTransform (x, x, transformEstim2Cam);
      Eigen::Quaterniond temp;
      Eigen::fromMsg(transformEstim2Cam.transform.rotation, temp);
      Px = temp.toRotationMatrix()*Px*temp.toRotationMatrix().transpose();


      boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd,int)> callback;
      callback=boost::bind(&Reprojector::distOmniEigen,this,_1,_2,_3);
      auto output =  unscented::unscentedTransform(x,Px, callback,-1.0,-1.0,-1.0);

      /* output.x = output.x.topRows(2); */
      /* output.C = output.C.topLeftCorner(2, 2); */

      return output;
    }

    unscented::measurement getProjectedCovarianceExpand(nav_msgs::Odometry currOdom,double expansion){
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


      getE2C(currImgTime);
      tf2::doTransform (x, x, transformEstim2Cam);
      Eigen::Quaterniond temp;
      Eigen::fromMsg(transformEstim2Cam.transform.rotation, temp);
      Px = temp.toRotationMatrix()*Px*temp.toRotationMatrix().transpose();
      Eigen::Matrix3d Pe = Eigen::Matrix3d::Identity()*expansion*expansion*3;
      /* Pe(2, 2) = 0.0; */
      Px += Pe;


      boost::function<Eigen::VectorXd(Eigen::VectorXd,Eigen::VectorXd,int)> callback;
      callback=boost::bind(&Reprojector::projectOmniEigen,this,_1,_2,_3);
      auto output =  unscented::unscentedTransform(x,Px, callback,-1.0,-1.0,-1.0);

      output.x = output.x.topRows(2);
      output.C = output.C.topLeftCorner(2, 2);

      return output;
    }

    //this was bad anyway - it was not transformed to camera!
    /* double getDistance(nav_msgs::Odometry i_odom){ */
    /*   /1* ROS_INFO_STREAM("pose: "<<  x); *1/ */
    /*   Eigen::Vector3d tmp(i_odom.pose.pose.position.x,i_odom.pose.pose.position.y,i_odom.pose.pose.position.z); */
    /*   return tmp.norm(); */
    /* } */

    double getMinDistance(Eigen::VectorXd x, Eigen::MatrixXd C){
      std::vector<Eigen::VectorXd> sigmas = unscented::getSigmaPtsSource(x,C);
      double shortestNorm=999;
      double currNorm;
      for (auto& sigma : sigmas){
        currNorm = sigma.topRows(3).norm();
        if (currNorm < shortestNorm)
          shortestNorm = currNorm;
      }
      return shortestNorm;
    }

    cv::RotatedRect getErrorEllipse(double chisquare_val, Eigen::Vector2d mean, Eigen::Matrix2Xd C){

      /* std::cout << "rotated: " << C <<std::endl; */

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

      /* ROS_INFO_STREAM("axes: " << halfmajoraxissize << " : " << halfminoraxissize); */

      //Return the oriented ellipse
      //The -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
      cv::Point2d mean_cv(mean.x(),mean.y());
      return cv::RotatedRect(mean_cv, cv::Size2f(halfmajoraxissize, halfminoraxissize), -angle);

    }


    //variables
    std::vector<nav_msgs::Odometry> odomBuffer[filterCount];
    cv::Mat currImage, viewImage;
    ros::Time currImgTime;

    std::mutex mtx_image, mtx_odom, mtx_tf;

    ros::Subscriber measSubscriber[filterCount];
    ros::Subscriber ImageSubscriber;

    ros::Timer tf_timer;
    ros::Timer im_timer;

    ros::Time lastMeasurement[filterCount];
    bool gotImage, gotOdom[filterCount], gotU2C, gotC2U;

    bool _gui,_offline, _publish_boxes, _no_draw_;

    tf2_ros::Buffer                 buffer;
    tf2_ros::TransformListener*     listener;
    geometry_msgs::TransformStamped transformEstim2Cam;
    geometry_msgs::TransformStamped transformCam2Estim;
    tf2::Stamped<tf2::Transform> TfC2E, TfE2C;

    mrs_lib::Profiler* profiler;
    std::string _frame_camera, _frame_estimate;

    std::string _calib_file;

    struct ocam_model oc_model;

    image_transport::Publisher imPub;
    ros::Publisher roiPublisher, distPublisher;

    clock_t begin, end;
    ros::Time begin_r, end_r;
    double  elapsedTime;
    ros::Duration elapsedTime_r;

    ROIVector rois, rois_empty_;
    DistRangeVector dists, dists_empty_;
};
}

int main(int argc, char** argv){
  ros::init(argc, argv, "uvdar_reprojector_node");
  ros::NodeHandle nh;
  uvdar::Reprojector r(nh);


  ros::spin();


  return 0;
}
