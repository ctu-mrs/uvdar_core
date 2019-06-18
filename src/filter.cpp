#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStampedPtr.h>
#include <mrs_lib/Lkf.h>

#define freq 30;


class UvdarKalman {
  namespace e = Eigen;

  public:
  UvdarKalman(ros::NodeHandle nh) {

  A.resize(9,9);
  B.resize(0,0);
  R.resize(9,9);
  Q.resize(6,6);
  P.resize(6,9);

  measSubscriber = nh.subscribe("measuredPose", 10, &UvdarKalman::measurementCallback, this);

    A <<
      1,0,0,1.0/freq,0,       0,       0,0,0,
      0,1,0,0,       1.0/freq,0,       0,0,0,
      0,0,1,0,       0,       1.0/freq,0,0,0,
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

    
    currKalman = new Lkf(9, 0, 6, A, B, R, Q, P);

    timer = nh_private.createTimer(ros::Duration(1.0/max(freq,1.0)), &UvdarKalman::spin, this);

  }

  ~UvdarKalman(){
    delete Lkf;
  }

  private:

  measurementCallback(const PoseWithCovarianceStampedPtr& meas){

    e::Quaternion qtemp;
    qtemp.x()= meas->pose.pose.orientation.x ;
    qtemp.y()= meas->pose.pose.orientation.y ;
    qtemp.z()= meas->pose.pose.orientation.z ;
    qtemp.w()= meas->pose.pose.orientation.w ;
    for (int i=0; i<ms.C.cols(); i++){
      for (int j=0; j<ms.C.rows(); j++){
        msgOdom->pose.covariance[ms.C.cols()*j+i] =  ms.C(j,i);
      }
    }

  }

  spin(const ros::TimerEvent& e){
    if (gotMeasurement)
      currKalman.iterate();
    else
      currKalman.iterateWithoutMeasurement();

  }


  e::MatrixXd A,B,R,Q,P;
  Lkf *currKalman;

  ros::Timer timer;


  ros::Subscriber measSubscriber;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "uvdar_reporter");
  ros::NodeHandle nodeA;
  UvdarKalman        kl(nodeA);

  ROS_INFO("UVDAR Pose reporter node initiated");

  ros::spin();

  return 0;
}
