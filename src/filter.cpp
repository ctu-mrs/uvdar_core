#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mrs_lib/Lkf.h>
#include <Eigen/Geometry>

#define freq 30.0


namespace e = Eigen;

class UvdarKalman {

  public:
  UvdarKalman(ros::NodeHandle nh) {

    dt = 1.0/freq;
    gotMeasurement = false;

    A.resize(9,9);
    B.resize(0,0);
    R.resize(9,9);
    Q.resize(6,6);
    P.resize(6,9);

    filtPublisher = nh.advertise<nav_msgs::Odometry>("filteredPose", 1);
    measSubscriber = nh.subscribe("measuredPose", 1, &UvdarKalman::measurementCallback, this);

    A <<
      1,0,0,dt,0,       0,       0,0,0,
      0,1,0,0,       dt,0,       0,0,0,
      0,0,1,0,       0,       dt,0,0,0,
      0,0,0,1,       0,       0,       0,0,0,
      0,0,0,0,       1,       0,       0,0,0,
      0,0,0,0,       0,       1,       0,0,0,
      0,0,0,0,       0,       0,       1,0,0,
      0,0,0,0,       0,       0,       0,1,0,
      0,0,0,0,       0,       0,       0,0,1;
    R <<
      1,0,0,0,0,0,0,0,0,
      0,1,0,0,0,0,0,0,0,
      0,0,1,0,0,0,0,0,0,
      0,0,0,1,0,0,0,0,0,
      0,0,0,0,1,0,0,0,0,
      0,0,0,0,0,1,0,0,0,
      0,0,0,0,0,0,1,0,0,
      0,0,0,0,0,0,0,1,0,
      0,0,0,0,0,0,0,0,1;

    Q <<
      1,0,0,0,0,0,
      0,1,0,0,0,0,
      0,0,1,0,0,0,
      0,0,0,1,0,0,
      0,0,0,0,1,0,
      0,0,0,0,0,1;

    P <<
      1,0,0,0,0,0,0,0,0,
      0,1,0,0,0,0,0,0,0,
      0,0,1,0,0,0,0,0,0,
      0,0,0,0,0,0,1,0,0,
      0,0,0,0,0,0,0,1,0,
      0,0,0,0,0,0,0,0,1;


    currKalman = new mrs_lib::Lkf(9, 0, 6, A, B, R, Q, P);

    timer = nh.createTimer(ros::Duration(1.0/fmax(freq,1.0)), &UvdarKalman::spin, this);

  }

  ~UvdarKalman(){
    delete currKalman;
  }

  private:

  void measurementCallback(const geometry_msgs::PoseWithCovarianceStampedPtr& meas){
    ROS_INFO_STREAM("Geting message: ");
    ROS_INFO_STREAM("" << meas->pose.pose.position);

    e::Quaternion qtemp(meas->pose.pose.orientation.w, meas->pose.pose.orientation.x, meas->pose.pose.orientation.y, meas->pose.pose.orientation.z );

    e::VectorXd mes(6); 

    mes(1) = meas->pose.pose.position.x;
    mes(2) = meas->pose.pose.position.y;
    mes(3) = meas->pose.pose.position.z;
    mes.bottomRows(3) = qtemp.toRotationMatrix().eulerAngles(0, 1, 2);


    for (int i=0; i<Q.cols(); i++){
      for (int j=0; j<Q.rows(); j++){
        Q(j,i) = meas->pose.covariance[Q.cols()*j+i] ;
      }
    }


    currKalman->setMeasurement(mes,Q);

    gotMeasurement =true;
  }

  void spin(const ros::TimerEvent& e){
    if (gotMeasurement){
      currKalman->iterate();
      gotMeasurement = false;
    }
    else
      currKalman->iterateWithoutCorrection();


    pubPose = boost::make_shared<nav_msgs::Odometry>();

    pubPose->pose.pose.position.x = currKalman->getState(0);
    pubPose->pose.pose.position.y = currKalman->getState(1);
    pubPose->pose.pose.position.z = currKalman->getState(2);
    e::Quaternion qtemp = e::AngleAxisd(currKalman->getState(6), e::Vector3d::UnitX()) * e::AngleAxisd(currKalman->getState(7), e::Vector3d::UnitY()) * e::AngleAxisd(currKalman->getState(8), e::Vector3d::UnitZ());
    /* qtemp.setRPY(currKalman->getState(6), currKalman->getState(7), currKalman->getState(8)); */
    /* qtemp=(transformCam2Base.getRotation().inverse())*qtemp; */
    pubPose->pose.pose.orientation.x = qtemp.x();
    pubPose->pose.pose.orientation.y = qtemp.y();
    pubPose->pose.pose.orientation.z = qtemp.z();
    pubPose->pose.pose.orientation.w = qtemp.w();
    e::MatrixXd C = currKalman->getCovariance();
    for (int i=0; i<3; i++){
      for (int j=0; j<3; j++){
        pubPose->pose.covariance[C.cols()*j+i] =  C(j,i);
      }
    }
    for (int i=6; i<9; i++){
      for (int j=6; j<9; j++){
        pubPose->pose.covariance[C.cols()*(j-6)+(i-6)] =  C(j,i);
      }
    }

    pubPose->twist.twist.linear.x = currKalman->getState(3);
    pubPose->twist.twist.linear.y = currKalman->getState(4);
    pubPose->twist.twist.linear.z = currKalman->getState(5);

    for (int i=3; i<6; i++){
      for (int j=3; j<6; j++){
        pubPose->twist.covariance[C.cols()*(j-3)+(i-3)] =  C(j,i);
      }
    }

    for (int i=6; i<9; i++){
      for (int j=6; j<9; j++){
        if (i == j)
          pubPose->twist.covariance[C.cols()*(j-6)+(i-6)] =  1.0;
        else
          pubPose->twist.covariance[C.cols()*(j-6)+(i-6)] =  0;
      }
    }

    pubPose->header.frame_id ="uvcam";
    pubPose->header.stamp = ros::Time::now();

    filtPublisher.publish(pubPose);


  }


  e::MatrixXd A,B,R,Q,P;
  mrs_lib::Lkf *currKalman;

  ros::Timer timer;

  double dt;

  bool gotMeasurement;
  ros::Subscriber measSubscriber;
  ros::Publisher filtPublisher;

  nav_msgs::OdometryPtr pubPose;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "uvdar_reporter");
  ros::NodeHandle nodeA;
  UvdarKalman        kl(nodeA);

  ROS_INFO("UVDAR Pose reporter node initiated");

  ros::spin();

  return 0;
}
