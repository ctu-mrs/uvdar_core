#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mrs_lib/Lkf.h>
#include <Eigen/Geometry>
#include <mutex>

#define freq 30.0
#define filterCount 2
#define decayTime 1.0
#define DEBUG true

#define useVelocity false


namespace e = Eigen;

class UvdarKalman {

  public:
  UvdarKalman(ros::NodeHandle nh) {

    dt = 1.0/freq;

    if (useVelocity){
      
    A.resize(9,9);
    B.resize(0,0);
    R.resize(9,9);
    Q.resize(6,6);
    P.resize(6,9);



    A <<
      1,0,0,dt,0, 0, 0,0,0,
      0,1,0,0, dt,0, 0,0,0,
      0,0,1,0, 0, dt,0,0,0,
      0,0,0,1, 0, 0, 0,0,0,
      0,0,0,0, 1, 0, 0,0,0,
      0,0,0,0, 0, 1, 0,0,0,
      0,0,0,0, 0, 0, 1,0,0,
      0,0,0,0, 0, 0, 0,1,0,
      0,0,0,0, 0, 0, 0,0,1;

    R <<
      dt,0,0,0,0,0,0,0,0,
      0,dt,0,0,0,0,0,0,0,
      0,0,dt,0,0,0,0,0,0,
      0,0,0,dt/2.0,0,0,0,0,0,
      0,0,0,0,dt/2.0,0,0,0,0,
      0,0,0,0,0,dt/2.0,0,0,0,
      0,0,0,0,0,0,dt,0,0,
      0,0,0,0,0,0,0,dt,0,
      0,0,0,0,0,0,0,0,dt;

    P <<
      1,0,0,0,0,0,0,0,0,
      0,1,0,0,0,0,0,0,0,
      0,0,1,0,0,0,0,0,0,
      0,0,0,0,0,0,1,0,0,
      0,0,0,0,0,0,0,1,0,
      0,0,0,0,0,0,0,0,1;
    }
    else {
    A.resize(6,6);
    B.resize(0,0);
    R.resize(6,6);
    Q.resize(6,6);
    P.resize(6,6);


    A <<
      1,0,0, 0,0,0,
      0,1,0, 0,0,0,
      0,0,1, 0,0,0,
      0,0,0, 1,0,0,
      0,0,0, 0,1,0,
      0,0,0, 0,0,1;

    R <<
      dt,0 ,0, 0 ,0 ,0,
      0, dt,0, 0 ,0 ,0,
      0, 0, dt,0 ,0 ,0,
      0, 0, 0, dt,0 ,0,
      0, 0, 0, 0 ,dt,0,
      0, 0, 0, 0 ,0 ,dt;

    P <<
      1,0,0,0,0,0,
      0,1,0,0,0,0,
      0,0,1,0,0,0,
      0,0,0,1,0,0,
      0,0,0,0,1,0,
      0,0,0,0,0,1;
    }

    Q <<
      1,0,0,0,0,0,
      0,1,0,0,0,0,
      0,0,1,0,0,0,
      0,0,0,1,0,0,
      0,0,0,0,1,0,
      0,0,0,0,0,1;



    for (int i=0;i<filterCount;i++){
      gotMeasurement[i] = false;
      gotAnyMeasurement[i] = false;
      if (useVelocity)
        currKalman[i] = new mrs_lib::Lkf(9, 0, 6, A, B, R, Q, P);
      else 
        currKalman[i] = new mrs_lib::Lkf(6, 0, 6, A, B, R, Q, P);
    }

    timer = nh.createTimer(ros::Duration(1.0/fmax(freq,1.0)), &UvdarKalman::spin, this);
    filtPublisher[0] = nh.advertise<nav_msgs::Odometry>("filteredPose1", 1);
    filtPublisher[1] = nh.advertise<nav_msgs::Odometry>("filteredPose2", 1);
    measSubscriber[0] = nh.subscribe("measuredPose1", 1, &UvdarKalman::measurementCallback, this);
    measSubscriber[1] = nh.subscribe("measuredPose2", 1, &UvdarKalman::measurementCallback, this);

  }

  ~UvdarKalman(){
    for (int i =0;i<filterCount;i++)
      delete currKalman[i];
  }

  private:

  /* void measurementCallback(const geometry_msgs::PoseWithCovarianceStampedPtr& meas){ */
  void measurementCallback(const ros::MessageEvent<geometry_msgs::PoseWithCovarianceStamped const>& event){
    std::scoped_lock slck(mtx_kalman);
    /* for(auto elem : event.getConnectionHeader()) */
    /* { */
    /*   ROS_INFO_STREAM("Message header: " << elem.first << " " << elem.second ); */
    /* } */
    ros::M_string mhdr = event.getConnectionHeader();
    std::string topic = mhdr["topic"];
    int target =(int)(topic.back())-49;
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& meas = event.getMessage();

      if (DEBUG){
        ROS_INFO_STREAM("Geting message [" << target <<"]");
        ROS_INFO_STREAM("" << meas->pose.pose.position);
      }


    e::Quaternion qtemp(meas->pose.pose.orientation.w, meas->pose.pose.orientation.x, meas->pose.pose.orientation.y, meas->pose.pose.orientation.z );

    e::VectorXd mes(6); 

    mes(0) = meas->pose.pose.position.x;
    mes(1) = meas->pose.pose.position.y;
    mes(2) = meas->pose.pose.position.z;

    if (mes.array().isNaN().any()){
      if (DEBUG)
        ROS_INFO_STREAM("Message [" << target <<"] contains NaNs, discarding");
      return;
    }

    mes.bottomRows(3) = qtemp.toRotationMatrix().eulerAngles(0, 1, 2);


    for (int i=0; i<Q.cols(); i++){
      for (int j=0; j<Q.rows(); j++){
        Q(j,i) = meas->pose.covariance[Q.cols()*j+i] ;
      }
    }

    if (Q.array().isNaN().any()){
      if (DEBUG)
        ROS_INFO_STREAM("Message [" << target <<"] contains NaNs, discarding");
      return;
    }

      if (DEBUG){
        ROS_INFO_STREAM("" << Q);
      }


    if (!gotAnyMeasurement[target]){

      if (DEBUG)
        ROS_INFO_STREAM("Initiating [" << target <<"]");
      if (useVelocity){
        e::VectorXd initState(9);
        initState << mes.topRows(3),e::Vector3d::Zero(),mes.bottomRows(3);
        currKalman[target]->setStates(initState);
      }
      else{
        e::VectorXd initState(6);
        initState << mes;
        currKalman[target]->setStates(initState);
      }
      /* currKalman[target]->setCovariance(Q); */
      gotAnyMeasurement[target] = true;
    }
    else{
      double md = mahalanobis_distance2(
          mes.topRows(3),
          currKalman[target]->getStates().topRows(3),
          currKalman[target]->getCovariance().topLeftCorner(3, 3)
          );

      if (DEBUG)
        ROS_INFO_STREAM("Mahalanobis dist. of [" << target <<"] is "<< md);

      if (md >16)
        return;

      if (DEBUG)
        ROS_INFO_STREAM("Applying measurement [" << target <<"]");
      currKalman[target]->setMeasurement(mes,Q);
      gotMeasurement[target] =true;
    }

    lastMeasurement[target] = ros::Time::now();

  }

  void spin([[ maybe_unused ]] const ros::TimerEvent& unused){
    std::scoped_lock slck(mtx_kalman);


    for (int target=0;target<filterCount;target++){
      if (!gotAnyMeasurement[target])
        continue;

      if (gotMeasurement[target] ){
          if (DEBUG)
            ROS_INFO_STREAM("Iterating [" << target <<"]");
        currKalman[target]->iterate();
        gotMeasurement[target] = false;
      }
      else
        if (ros::Duration(ros::Time::now()-lastMeasurement[target]).toSec()<decayTime){
          if (DEBUG)
            ROS_INFO_STREAM("Iterating [" << target <<"] without measurement");
          currKalman[target]->iterateWithoutCorrection();
        }
        else{
          if (DEBUG)
            ROS_INFO_STREAM("Tracker [" << target <<"] has decayed");
          gotAnyMeasurement[target] = false;
          return;
        }


      pubPose = boost::make_shared<nav_msgs::Odometry>();

      pubPose->pose.pose.position.x = currKalman[target]->getState(0);
      pubPose->pose.pose.position.y = currKalman[target]->getState(1);
      pubPose->pose.pose.position.z = currKalman[target]->getState(2);

      e::Quaternion<double> qtemp;
      if (useVelocity)
        qtemp = e::AngleAxisd(currKalman[target]->getState(6), e::Vector3d::UnitX()) * e::AngleAxisd(currKalman[target]->getState(7), e::Vector3d::UnitY()) * e::AngleAxisd(currKalman[target]->getState(8), e::Vector3d::UnitZ());
      else
        qtemp = e::AngleAxisd(currKalman[target]->getState(3), e::Vector3d::UnitX()) * e::AngleAxisd(currKalman[target]->getState(4), e::Vector3d::UnitY()) * e::AngleAxisd(currKalman[target]->getState(5), e::Vector3d::UnitZ());
      /* qtemp.setRPY(currKalman[target]->getState(6), currKalman[target]->getState(7), currKalman[target]->getState(8)); */
      /* qtemp=(transformCam2Base.getRotation().inverse())*qtemp; */
      pubPose->pose.pose.orientation.x = qtemp.x();
      pubPose->pose.pose.orientation.y = qtemp.y();
      pubPose->pose.pose.orientation.z = qtemp.z();
      pubPose->pose.pose.orientation.w = qtemp.w();
      e::MatrixXd C = currKalman[target]->getCovariance();
      for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
          pubPose->pose.covariance[6*j+i] =  C(j,i);
        }
      }
      if (useVelocity)
        for (int i=6; i<9; i++){
          for (int j=6; j<9; j++){
            pubPose->pose.covariance[6*(j-3)+(i-3)] =  C(j,i);
          }
        }
      else
        for (int i=3; i<6; i++){
          for (int j=3; j<6; j++){
            pubPose->pose.covariance[6*(j)+(i)] =  C(j,i);
          }
        }

      if (useVelocity){
        pubPose->twist.twist.linear.x = currKalman[target]->getState(3);
        pubPose->twist.twist.linear.y = currKalman[target]->getState(4);
        pubPose->twist.twist.linear.z = currKalman[target]->getState(5);

        for (int i=3; i<6; i++){
          for (int j=3; j<6; j++){
            if (useVelocity)
              pubPose->twist.covariance[6*(j-3)+(i-3)] =  C(j,i);
            else
              pubPose->twist.covariance[6*(j-3)+(i-3)] =  1;
          }
        }

        for (int i=3; i<6; i++){
          for (int j=3; j<6; j++){
            if (i == j)
              pubPose->twist.covariance[6*(j)+(i)] =  1.0;
            else
              pubPose->twist.covariance[6*(j)+(i)] =  0;
          }
        }
      }

      pubPose->header.frame_id ="uvcam";
      pubPose->header.stamp = ros::Time::now();

      filtPublisher[target].publish(pubPose);

      if (DEBUG){
        ROS_INFO_STREAM("State: ");
        ROS_INFO_STREAM("" << currKalman[target]->getStates());
      }

    }

  }


  e::MatrixXd A,B,R,Q,P;
  mrs_lib::Lkf* currKalman[filterCount];

  ros::Timer timer;

  double dt;

  bool gotMeasurement[filterCount], gotAnyMeasurement[filterCount];
  ros::Duration sinceMeasurement[filterCount];
  ros::Time lastMeasurement[filterCount];
  ros::Subscriber measSubscriber[filterCount];
  ros::Subscriber          ImageSubscriber;
  ros::Publisher filtPublisher[filterCount];

  nav_msgs::OdometryPtr pubPose;

  std::mutex mtx_kalman;


    static double mahalanobis_distance2(const Eigen::Vector3d& x, const Eigen::Vector3d& mu1, const Eigen::Matrix3d& sigma1)
    {
      const auto diff = x - mu1;
      const double dist2 = diff.transpose() * sigma1.inverse() * diff;
      return dist2;
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "uvdar_reporter");
  ros::NodeHandle nodeA;
  UvdarKalman        kl(nodeA);

  ROS_INFO("UVDAR Kalman filter node initiated");

  ros::spin();

  return 0;
}
