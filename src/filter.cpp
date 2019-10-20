#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mrs_lib/lkf.h>
#include <Eigen/Geometry>
#include <std_msgs/Int16.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <mutex>

#define freq 30.0
#define validTime 0.5
#define decayTime 3.0
#define DEBUG true
#define minMeasurementsToValidation 2

namespace mrs_lib
{
  const int n_states = -1;
  const int n_inputs = 0;
  const int n_measurements = 6;
  using lkf_t = LKF<n_states, n_inputs, n_measurements>;
}

using A_t = mrs_lib::lkf_t::A_t;
using B_t = mrs_lib::lkf_t::B_t;
using H_t = mrs_lib::lkf_t::H_t;
using Q_t = mrs_lib::lkf_t::Q_t;
using R_t = mrs_lib::lkf_t::R_t;
using u_t = mrs_lib::lkf_t::u_t;
using x_t = mrs_lib::lkf_t::x_t;
using P_t = mrs_lib::lkf_t::P_t;
using statecov_t = mrs_lib::lkf_t::statecov_t;

namespace e = Eigen;


class UvdarKalman {
  int filterCount = 2;

  public:
  UvdarKalman(ros::NodeHandle nh) {

    ros::NodeHandle pnh("~");
    pnh.param("output_frame", _output_frame, std::string("local_origin"));
    pnh.param("filterCount", filterCount, filterCount);
    pnh.param("useVelocity", _use_velocity_, bool(false));
    pnh.param("outputFramerate", _output_framerate_, double(freq));

    pnh.param("indoor", _indoor_, bool(false));
    pnh.param("odometryAvailable", _odometry_available_, bool(true));

    if (_indoor_){
      vl = 1;
      vv = 0.5;
    }
    else {
      vl = 2;
      vv = 1;
    }

    if (_odometry_available_){
      sn = 1;
    }
    else {
      sn = 2;
    }



    ROS_INFO_STREAM("[UVDAR Kalman] filterCount: " << filterCount);

    /* Initialize variables //{ */

    currKalman.resize(filterCount);
    trackerValidated.resize(filterCount);
    measurementsAssociated.resize(filterCount);
    gotAnyMeasurement.resize(filterCount);
    lastMeasurement.resize(filterCount);
    measSubscriber.resize(filterCount);
    filtPublisher.resize(filterCount);
    filtPublisherTentative.resize(filterCount);

    //}

    dt = 1.0/_output_framerate_;

    for (int i=0; i< filterCount; i++){

      td_t tmp;
      td.push_back(tmp);

    if (_use_velocity_){

    td[i].A.resize(9,9);
    td[i].Q.resize(9,9);
    td[i].R.resize(6,6);
    td[i].H.resize(6,9);



    td[i].A <<
      1,0,0,dt,0, 0, 0,0,0,
      0,1,0,0, dt,0, 0,0,0,
      0,0,1,0, 0, dt,0,0,0,
      0,0,0,1, 0, 0, 0,0,0,
      0,0,0,0, 1, 0, 0,0,0,
      0,0,0,0, 0, 1, 0,0,0,
      0,0,0,0, 0, 0, 1,0,0,
      0,0,0,0, 0, 0, 0,1,0,
      0,0,0,0, 0, 0, 0,0,1;

    td[i].Q <<
      vl,0,0,0,0,0,0,0,0,
      0,vl,0,0,0,0,0,0,0,
      0,0,vv,0,0,0,0,0,0,
      0,0,0,0.5,0,0,0,0,0,
      0,0,0,0,0.5,0,0,0,0,
      0,0,0,0,0,0.5,0,0,0,
      0,0,0,0,0,0,1,0,0,
      0,0,0,0,0,0,0,1,0,
      0,0,0,0,0,0,0,0,1;

    td[i].H <<
      1,0,0,0,0,0,0,0,0,
      0,1,0,0,0,0,0,0,0,
      0,0,1,0,0,0,0,0,0,
      0,0,0,0,0,0,1,0,0,
      0,0,0,0,0,0,0,1,0,
      0,0,0,0,0,0,0,0,1;
    }
    else {
    td[i].A.resize(6,6);
    td[i].Q.resize(6,6);
    td[i].R.resize(6,6);
    td[i].H.resize(6,6);


    td[i].A <<
      1,0,0, 0,0,0,
      0,1,0, 0,0,0,
      0,0,1, 0,0,0,
      0,0,0, 1,0,0,
      0,0,0, 0,1,0,
      0,0,0, 0,0,1;

    td[i].Q <<
      vl, 0 ,0, 0, 0 ,0,
      0, vl, 0, 0, 0 ,0,
      0, 0, vv, 0, 0 ,0,
      0, 0, 0, 1, 0 ,0,
      0, 0, 0, 0, 1 ,0,
      0, 0, 0, 0, 0 ,1;

    td[i].H <<
      1,0,0,0,0,0,
      0,1,0,0,0,0,
      0,0,1,0,0,0,
      0,0,0,1,0,0,
      0,0,0,0,1,0,
      0,0,0,0,0,1;
    }

    td[i].B = B_t();

    td[i].R <<
      1,0,0,0,0,0,
      0,1,0,0,0,0,
      0,0,1,0,0,0,
      0,0,0,1,0,0,
      0,0,0,0,1,0,
      0,0,0,0,0,1;


    }

    /* delay = ros::Duration(0); */


    for (int i=0;i<filterCount;i++){
      trackerValidated[i] = false;
      measurementsAssociated[i] = 0;
      gotAnyMeasurement[i] = false;
      /* if (_use_velocity_) */
        currKalman[i] = new mrs_lib::lkf_t(td[i].A, td[i].B, td[i].H);
      /* else */ 
      /*   currKalman[i] = new mrs_lib::lkf_t(td[i].A, td[i].B, td[i].H); */
    }

    tf_listener = new tf2_ros::TransformListener(tf_buffer);

    timer = nh.createTimer(ros::Duration(1.0 / fmax(_output_framerate_,1.0)), &UvdarKalman::spin, this);
    for (int i = 0; i < filterCount; ++i) 
    {
      measSubscriber[i] = nh.subscribe("measuredPose" + std::to_string(i+1), 1, &UvdarKalman::measurementCallback, this);
      filtPublisher[i] = nh.advertise<nav_msgs::Odometry>("filteredPose" + std::to_string(i+1), 1);
      filtPublisherTentative[i] = nh.advertise<nav_msgs::Odometry>("filteredPose" + std::to_string(i+1) + "/tentative", 1);

    }

    targetsSeenCountPublisher = nh.advertise<std_msgs::Int16>("targetsSeenCount",1);

    ROS_INFO_STREAM("[UVDAR Kalman]: initiated");
  }

  ~UvdarKalman(){
    /* for (int i =0;i<filterCount;i++) */
    /*   delete currKalman[i]; */
  }

  private:


  double fixAngle(double origAngle, double newAngle){
    double fixedPre;
    if ( (origAngle>(2*M_PI)) || (origAngle<(-2*M_PI)) )  {
      fixedPre = fmod(origAngle,2*M_PI);
      }
    else {
      fixedPre = origAngle;
    }
      /* fixedPre = origAngle; */

    if (fixedPre>(M_PI))
      fixedPre = fixedPre - (2.0*M_PI);
    if (fixedPre < (-M_PI))
      fixedPre = fixedPre + (2.0*M_PI);



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
        ROS_INFO_STREAM("Getting message [" << target <<"]");
        ROS_INFO_STREAM("" << meas.pose.pose.position);
      }


    e::Quaternion qtemp(meas.pose.pose.orientation.w, meas.pose.pose.orientation.x, meas.pose.pose.orientation.y, meas.pose.pose.orientation.z );

    e::VectorXd mes(6); 

    mes(0) = meas.pose.pose.position.x;
    mes(1) = meas.pose.pose.position.y;
    mes(2) = meas.pose.pose.position.z;

    e::Vector3d tmp  = qtemp.toRotationMatrix().eulerAngles(0, 1, 2);
    if ((fabs(tmp(0))>(M_PI/2)) && (fabs(tmp(1))>(M_PI/2)) ){
      for (int u=0; u<3; u++){
        if ( (tmp(u)>(2*M_PI)) || (tmp(u)<(-2*M_PI)) ){
            ROS_INFO_STREAM("Error in tmp(" << u << "): [" << tmp(u) << "] from qtemp=(" << qtemp.x() << "," <<  qtemp.y() << "," << qtemp.z() << "," << qtemp.w() << ") and tmp = [" << tmp.transpose() << "]");
            }
      }
      tmp(0) = fixAngle(tmp(0)+M_PI,0);
      tmp(1) = fixAngle(tmp(1)+M_PI,0);
      tmp(2) = fixAngle(tmp(2)+M_PI,0);
    }

    mes.bottomRows(3) = tmp;

    if (mes.array().isNaN().any()){
      if (DEBUG)
        ROS_INFO_STREAM("Message [" << target <<"] contains NaNs, discarding");
      return;
    }


    for (int i=0; i<td[target].R.cols(); i++){
      for (int j=0; j<td[target].R.rows(); j++){
        if ( ( (i<3) && (j<3) ) || ( (i>=3) && (j>=3) ) )
          td[target].R(j,i) = meas.pose.covariance[td[target].R.cols()*j+i] ;
        else
          td[target].R(j,i) = 0.0;
      }
    }

    if (td[target].R.array().isNaN().any()){
      if (DEBUG)
        ROS_INFO_STREAM("Message [" << target <<"] contains NaNs, discarding");
      return;
    }

      if (DEBUG){
        ROS_INFO_STREAM("" << td[target].R);
      }


    if (!gotAnyMeasurement[target]){

      if (DEBUG)
        ROS_INFO_STREAM("Initiating [" << target <<"]");
      if (_use_velocity_){
        e::VectorXd initState(9);
        initState << mes.topRows(3),e::Vector3d::Zero(),mes.bottomRows(3);
        e::MatrixXd initCovariance = e::MatrixXd::Identity(9,9)*10;
        initCovariance.topLeftCorner(3, 3) = td[target].R.topLeftCorner(3,3);
        initCovariance.bottomRightCorner(3, 3) = td[target].R.bottomRightCorner(3,3);
        td[target].state_m.x = initState;
        td[target].state_m.P = initCovariance;
      }
      else{
        e::VectorXd initState(6);
        initState << mes;
        td[target].state_m.x = initState;
        td[target].state_m.P = td[target].R;
      }
      /* currKalman[target]->setCovariance(R); */
      gotAnyMeasurement[target] = true;
      trackerValidated[target] = false;
      measurementsAssociated[target] = 0;
    }
    else{
      statecov_t state_tmp;
      double dt = (meas.header.stamp-lastMeasurement[target]).toSec();
      A_dt(target, dt);
      Q_dt(target, dt);
      currKalman[target]->A = td[target].A;
      /* ROS_INFO_STREAM("BEFORE PRED: " << std::endl << td[target].state_m.x ); */
      state_tmp = currKalman[target]->predict(td[target].state_m, u_t(), (td[target].Q), dt);

      double md = mahalanobis_distance2(
          mes.topRows(3),
          state_tmp.x.topRows(3),
          /* currKalman[target]->getCovariance().topLeftCorner(3, 3) */
          td[target].R.topLeftCorner(3, 3)
          );

      if (DEBUG)
        ROS_INFO_STREAM("Mahalanobis dist. of [" << target <<"] is "<< md);

      /* if (md >16) */
      /*   return; */

      if (DEBUG)
        ROS_INFO_STREAM("Applying measurement [" << target <<"]");

      /* ROS_INFO_STREAM("BEFORE CORR: " << std::endl << state_tmp.x ); */
      state_tmp.x[_use_velocity_?8:5] = fixAngle(state_tmp.x((_use_velocity_?8:5)), mes[5]);
      if (DEBUG)
        ROS_INFO_STREAM("State [" << std::endl << state_tmp.x << "]");
        ROS_INFO_STREAM("State [" << std::endl << state_tmp.P << "]");
      td[target].state_m = currKalman[target]->correct(state_tmp, mes, td[target].R);
      if (DEBUG){
        ROS_INFO_STREAM("mes: " <<std::endl << mes );
        ROS_INFO_STREAM("R: " <<std::endl << td[target].R );
        ROS_INFO_STREAM("AFTER: " << std::endl << td[target].state_m.x );
      }
      //fix angles to account for correction through 0/2pi

      /* currKalman[target]->setMeasurement(mes,R); */
    }

    lastMeasurement[target] = meas.header.stamp;
    /* delay = ros::Duration(0.5*(delay + (ros::Time::now()-lastMeasurement[target])).toSec()); */
    /* if (DEBUG) */
    /*   ROS_INFO_STREAM("Estimated time delay is: " << delay << "s"); */
    measurementsAssociated[target]++;
    if (measurementsAssociated[target] >= minMeasurementsToValidation)
      trackerValidated[target] = true;

  }

  void spin([[ maybe_unused ]] const ros::TimerEvent& unused){
    std::scoped_lock slck(mtx_kalman);


    int targetsSeen = 0;
    std_msgs::Int16 pubTargetsSeen;
    for (int target=0;target<filterCount;target++){
      if (!gotAnyMeasurement[target])
        continue;
      double dt = ros::Duration(ros::Time::now()-lastMeasurement[target]).toSec();
      if (dt<decayTime){
        if (DEBUG)
          ROS_INFO_STREAM("Iterating [" << target <<"] without measurement, dt=" << dt);
        A_dt(target, dt);
        Q_dt(target, dt);
        currKalman[target]->A = td[target].A;
        td[target].state_x = currKalman[target]->predict(td[target].state_m, u_t(), (td[target].Q), dt);
        if (dt>validTime){
          trackerValidated[target] = false;
          ROS_INFO_STREAM(" Not validated!");
        }
      }
      else{
        if (DEBUG)
          ROS_INFO_STREAM("Tracker [" << target <<"] has decayed");
        resetTracker(target);
        return;
      }

      pubPose = boost::make_shared<nav_msgs::Odometry>();

      pubPose->pose.pose.position.x = td[target].state_x.x[0];
      pubPose->pose.pose.position.y = td[target].state_x.x[1];
      pubPose->pose.pose.position.z = td[target].state_x.x[2];

      e::Quaternion<double> qtemp;
      if (_use_velocity_)
        qtemp = e::AngleAxisd(td[target].state_x.x[6], e::Vector3d::UnitX()) * e::AngleAxisd(td[target].state_x.x[7], e::Vector3d::UnitY()) * e::AngleAxisd(td[target].state_x.x[8], e::Vector3d::UnitZ());
      else
        qtemp = e::AngleAxisd(td[target].state_x.x[3], e::Vector3d::UnitX()) * e::AngleAxisd(td[target].state_x.x[4], e::Vector3d::UnitY()) * e::AngleAxisd(td[target].state_x.x[5], e::Vector3d::UnitZ());
      /* qtemp.setRPY(currKalman[target]->getState(6), currKalman[target]->getState(7), currKalman[target]->getState(8)); */
      /* qtemp=(transformCam2Base.getRotation().inverse())*qtemp; */
      pubPose->pose.pose.orientation.x = qtemp.x();
      pubPose->pose.pose.orientation.y = qtemp.y();
      pubPose->pose.pose.orientation.z = qtemp.z();
      pubPose->pose.pose.orientation.w = qtemp.w();
      e::MatrixXd C = td[target].state_x.P;
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
        pubPose->twist.twist.linear.x = td[target].state_x.x[3];
        pubPose->twist.twist.linear.y = td[target].state_x.x[4];
        pubPose->twist.twist.linear.z = td[target].state_x.x[5];

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
      pubPose->header.stamp = ros::Time::now();

      if (trackerValidated[target]){
        filtPublisher[target].publish(pubPose);
        targetsSeen++;
      }
      else
        filtPublisherTentative[target].publish(pubPose);



      if (DEBUG){
        ROS_INFO_STREAM("State: ");
        ROS_INFO_STREAM("\n" << td[target].state_x.x);
      }

    }
    pubTargetsSeen.data = targetsSeen;
    targetsSeenCountPublisher.publish(pubTargetsSeen);

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

  void A_dt(int i, double dt){
    if (_use_velocity_)
      td[i].A <<
        1,0,0,dt,0, 0, 0,0,0,
        0,1,0,0, dt,0, 0,0,0,
        0,0,1,0, 0, dt,0,0,0,
        0,0,0,1, 0, 0, 0,0,0,
        0,0,0,0, 1, 0, 0,0,0,
        0,0,0,0, 0, 1, 0,0,0,
        0,0,0,0, 0, 0, 1,0,0,
        0,0,0,0, 0, 0, 0,1,0,
        0,0,0,0, 0, 0, 0,0,1;
    else
      td[i].A <<
        1,0,0, 0,0,0,
        0,1,0, 0,0,0,
        0,0,1, 0,0,0,
        0,0,0, 1,0,0,
        0,0,0, 0,1,0,
        0,0,0, 0,0,1;
  }

  void Q_dt(int i, double dt){
    if (_use_velocity_)
      td[i].Q <<
      sn*sn,0,0,0,0,0,0,0,0,
      0,sn*sn,0,0,0,0,0,0,0,
      0,0,sn*sn,0,0,0,0,0,0,
      0,0,0,2,0,0,0,0,0,
      0,0,0,0,2,0,0,0,0,
      0,0,0,0,0,1,0,0,0,
      0,0,0,0,0,0,1,0,0,
      0,0,0,0,0,0,0,1,0,
      0,0,0,0,0,0,0,0,1;
    else
      td[i].Q <<
        0.5*sn*sn+0.16667*vl*vl*dt,0,0, 0,0,0,
        0,0.5*sn*sn+0.16667*vl*vl*dt,0, 0,0,0,
        0,0,0.5*sn*sn+0.16667*vv*vv*dt, 0,0,0,
        0,0,0, 1,0,0,
        0,0,0, 0,1,0,
        0,0,0, 0,0,1;
  }


  struct td_t{
    A_t A;
    B_t B;
    H_t H;
    Q_t Q;
    R_t R;
    statecov_t state_m;
    statecov_t state_x;
  };
  std::vector<td_t> td;
  std::vector<mrs_lib::lkf_t*> currKalman;
  std::vector<bool> trackerValidated;
  std::vector<int> measurementsAssociated;

  ros::Timer timer;

  double dt;

  std::vector<bool>  gotAnyMeasurement;
  /* std::vector<bool> gotMeasurement, gotAnyMeasurement; */
  /* ros::Duration sinceMeasurement[filterCount]; */
  std::vector<ros::Time> lastMeasurement;
  std::vector<ros::Subscriber> measSubscriber;
  std::vector<ros::Subscriber> ImageSubscriber;
  std::vector<ros::Publisher> filtPublisher;
  std::vector<ros::Publisher> filtPublisherTentative;
  ros::Publisher              targetsSeenCountPublisher;

  nav_msgs::OdometryPtr pubPose;

  /* ros::Duration delay; */

  std::mutex mtx_kalman;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener *tf_listener;

  std::string _output_frame;


  bool _use_velocity_;
  double _output_framerate_;

  bool _odometry_available_, _indoor_;
  double vl, vv, sn;

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "uvdar_reporter");
  ros::NodeHandle nodeA;
  UvdarKalman        kl(nodeA);

  ROS_INFO("[UVDAR Kalman]: filter node initiated");

  ros::spin();

  return 0;
}
