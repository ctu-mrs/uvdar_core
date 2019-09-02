#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mrs_lib/Lkf.h>
#include <Eigen/Geometry>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mutex>

#define freq 30.0
#define validTime 1.0
#define decayTime 2.0
#define DEBUG true
#define minMeasurementsToValidation 2


namespace e = Eigen;

class UvdarKalman {
  int filterCount = 2;

  public:
  UvdarKalman(ros::NodeHandle nh) {

    ros::NodeHandle pnh("~");
    pnh.param("output_frame", _output_frame, std::string("local_origin"));
    pnh.param("filterCount", filterCount, filterCount);
    pnh.param("useVelocity", _use_velocity_, bool(false));

    ROS_INFO_STREAM("[Param] filterCount: " << filterCount);

    /* Initialize variables //{ */

    currKalman.resize(filterCount);
    trackerValidated.resize(filterCount);
    measurementsAssociated.resize(filterCount);
    gotMeasurement.resize(filterCount);
    gotAnyMeasurement.resize(filterCount);
    lastMeasurement.resize(filterCount);
    measSubscriber.resize(filterCount);
    filtPublisher.resize(filterCount);
    filtPublisherTentative.resize(filterCount);

    //}

    dt = 1.0/freq;

    if (_use_velocity_){
      
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
      2*dt,0,0,0,0,0,0,0,0,
      0,2*dt,0,0,0,0,0,0,0,
      0,0,2*dt,0,0,0,0,0,0,
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


    delay = ros::Duration(0);


    for (int i=0;i<filterCount;i++){
      trackerValidated[i] = false;
      measurementsAssociated[i] = 0;
      gotMeasurement[i] = false;
      gotAnyMeasurement[i] = false;
      if (_use_velocity_)
        currKalman[i] = new mrs_lib::Lkf(9, 0, 6, A, B, R, Q, P);
      else 
        currKalman[i] = new mrs_lib::Lkf(6, 0, 6, A, B, R, Q, P);
    }

    tf_listener = new tf2_ros::TransformListener(tf_buffer);

    timer = nh.createTimer(ros::Duration(1.0 / fmax(freq,1.0)), &UvdarKalman::spin, this);
    for (int i = 0; i < filterCount; ++i) 
    {
      measSubscriber[i] = nh.subscribe("measuredPose" + std::to_string(i+1), 1, &UvdarKalman::measurementCallback, this);
      filtPublisher[i] = nh.advertise<nav_msgs::Odometry>("filteredPose" + std::to_string(i+1), 1);
      filtPublisherTentative[i] = nh.advertise<nav_msgs::Odometry>("filteredPose" + std::to_string(i+1) + "/tentative", 1);

    }
  }

  ~UvdarKalman(){
    for (int i =0;i<filterCount;i++)
      delete currKalman[i];
  }

  private:


  double fixAngle(double origAngle, double newAngle){
    double fixedPre;
    if (origAngle>(M_PI))
      fixedPre = origAngle - (2.0*M_PI);
    if (origAngle < (-M_PI))
      fixedPre = origAngle + (2.0*M_PI);



    if (fabs(newAngle-fixedPre)<M_PI)
      return fixedPre;

    if (fixedPre>newAngle)
      return (fixedPre - (2.0*M_PI));
    else
      return (fixedPre + (2.0*M_PI));
  }

  /* void measurementCallback(const geometry_msgs::PoseWithCovarianceStampedPtr& meas){ */
  void measurementCallback(const ros::MessageEvent<geometry_msgs::PoseWithCovarianceStamped const>& event){
    geometry_msgs::TransformStamped transform;
    try
    {
      transform = tf_buffer.lookupTransform(_output_frame, event.getMessage()->header.frame_id, event.getMessage()->header.stamp, ros::Duration(0.1));
    } catch (tf2::TransformException& ex)
    {
      ROS_WARN("Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", event.getMessage()->header.frame_id.c_str(), _output_frame.c_str(), ex.what());
      return;
    }

    std::scoped_lock slck(mtx_kalman);
    /* for(auto elem : event.getConnectionHeader()) */
    /* { */
    /*   ROS_INFO_STREAM("Message header: " << elem.first << " " << elem.second ); */
    /* } */
    ros::M_string mhdr = event.getConnectionHeader();
    std::string topic = mhdr["topic"];
    int target =(int)(topic.back())-49;
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& meas_local = event.getMessage();
    geometry_msgs::PoseWithCovarianceStamped meas;

    tf2::doTransform(*meas_local, meas, transform);

      if (DEBUG){
        ROS_INFO_STREAM("Geting message [" << target <<"]");
        ROS_INFO_STREAM("" << meas.pose.pose.position);
      }


    e::Quaternion qtemp(meas.pose.pose.orientation.w, meas.pose.pose.orientation.x, meas.pose.pose.orientation.y, meas.pose.pose.orientation.z );

    e::VectorXd mes(6); 

    mes(0) = meas.pose.pose.position.x;
    mes(1) = meas.pose.pose.position.y;
    mes(2) = meas.pose.pose.position.z;

    if (mes.array().isNaN().any()){
      if (DEBUG)
        ROS_INFO_STREAM("Message [" << target <<"] contains NaNs, discarding");
      return;
    }

    mes.bottomRows(3) = qtemp.toRotationMatrix().eulerAngles(0, 1, 2);


    for (int i=0; i<Q.cols(); i++){
      for (int j=0; j<Q.rows(); j++){
        Q(j,i) = meas.pose.covariance[Q.cols()*j+i] ;
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
      if (_use_velocity_){
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
      trackerValidated[target] = false;
      measurementsAssociated[target] = 0;
    }
    else{
      double md = mahalanobis_distance2(
          mes.topRows(3),
          currKalman[target]->getStates().topRows(3),
          /* currKalman[target]->getCovariance().topLeftCorner(3, 3) */
          Q.topLeftCorner(3, 3)
          );

      if (DEBUG)
        ROS_INFO_STREAM("Mahalanobis dist. of [" << target <<"] is "<< md);

      /* if (md >16) */
      /*   return; */

      if (DEBUG)
        ROS_INFO_STREAM("Applying measurement [" << target <<"]");

      //fix angles to account for correction through 0/2pi
      currKalman[target]->setState(
          (_use_velocity_?8:5),
          fixAngle(currKalman[target]->getState((_use_velocity_?8:5)), mes[5])
            );

      currKalman[target]->setMeasurement(mes,Q);
      gotMeasurement[target] =true;
    }

    lastMeasurement[target] = meas_local->header.stamp;
    delay = ros::Duration(0.5*(delay + (ros::Time::now()-lastMeasurement[target])).toSec());
    if (DEBUG)
      ROS_INFO_STREAM("Estimated time delay is: " << delay << "s");
    measurementsAssociated[target]++;
    if (measurementsAssociated[target] >= minMeasurementsToValidation)
      trackerValidated[target] = true;

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
        if (ros::Duration(ros::Time::now()-lastMeasurement[target]).toSec()<validTime){
          if (DEBUG)
            ROS_INFO_STREAM("Iterating [" << target <<"] without measurement");
          currKalman[target]->iterateWithoutCorrection();
        }
        else if (ros::Duration(ros::Time::now()-lastMeasurement[target]).toSec()<decayTime){
          if (DEBUG)
            ROS_INFO_STREAM("Iterating [" << target <<"] without measurement");
          currKalman[target]->iterateWithoutCorrection();
          trackerValidated[target] = false;
        }
        else{
          if (DEBUG)
            ROS_INFO_STREAM("Tracker [" << target <<"] has decayed");
          resetTracker(target);
          return;
        }

      pubPose = boost::make_shared<nav_msgs::Odometry>();

      pubPose->pose.pose.position.x = currKalman[target]->getState(0);
      pubPose->pose.pose.position.y = currKalman[target]->getState(1);
      pubPose->pose.pose.position.z = currKalman[target]->getState(2);

      e::Quaternion<double> qtemp;
      if (_use_velocity_)
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
      if (_use_velocity_)
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

      if (_use_velocity_){
        pubPose->twist.twist.linear.x = currKalman[target]->getState(3);
        pubPose->twist.twist.linear.y = currKalman[target]->getState(4);
        pubPose->twist.twist.linear.z = currKalman[target]->getState(5);

        for (int i=3; i<6; i++){
          for (int j=3; j<6; j++){
            if (_use_velocity_)
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

      pubPose->header.frame_id = _output_frame;
      pubPose->header.stamp = ros::Time::now()-delay;

      if (trackerValidated[target])
        filtPublisher[target].publish(pubPose);
      else
        filtPublisherTentative[target].publish(pubPose);



      if (DEBUG){
        ROS_INFO_STREAM("State: ");
        ROS_INFO_STREAM("\n" << currKalman[target]->getStates());
      }

    }

  }

  void resetTracker(int target){
    gotAnyMeasurement[target] = false;
    trackerValidated[target] = false;
    measurementsAssociated[target] = 0;
  }


  static double mahalanobis_distance2(const Eigen::Vector3d& x, const Eigen::Vector3d& mu1, const Eigen::Matrix3d& sigma1)
  {
    const auto diff = x - mu1;
    const double dist2 = diff.transpose() * sigma1.inverse() * diff;
    return dist2;
  }


  e::MatrixXd A,B,R,Q,P;
  std::vector<mrs_lib::Lkf*> currKalman;
  std::vector<bool> trackerValidated;
  std::vector<int> measurementsAssociated;

  ros::Timer timer;

  double dt;

  std::vector<bool> gotMeasurement, gotAnyMeasurement;
  /* ros::Duration sinceMeasurement[filterCount]; */
  std::vector<ros::Time> lastMeasurement;
  std::vector<ros::Subscriber> measSubscriber;
  std::vector<ros::Subscriber> ImageSubscriber;
  std::vector<ros::Publisher> filtPublisher;
  std::vector<ros::Publisher> filtPublisherTentative;

  nav_msgs::OdometryPtr pubPose;

  ros::Duration delay;

  std::mutex mtx_kalman;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener *tf_listener;

  std::string _output_frame;


  bool _use_velocity_;

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "uvdar_reporter");
  ros::NodeHandle nodeA;
  UvdarKalman        kl(nodeA);

  ROS_INFO("UVDAR Kalman filter node initiated");

  ros::spin();

  return 0;
}
