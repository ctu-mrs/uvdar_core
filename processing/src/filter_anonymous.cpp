#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/transformer.h>

#include <mrs_lib/lkf.h>

#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/Vec4.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Odometry.h>

/* #include <sensor_msgs/PointCloud2.h> */
/* #include <sensor_msgs/point_cloud2_iterator.h> */

#include <boost/range/adaptor/indexed.hpp> 

#include <mutex>

#include <Eigen/Dense>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define DEBUG true
#define freq 20.0
#define DECAY_AGE_NORMAL 10.0

using namespace boost::adaptors;

namespace mrs_lib
{
  const int n_states = 6;
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


struct td_t{
  A_t A;
  B_t B;
  H_t H;
  Q_t Q;
  R_t R;
  /* statecov_t state_m; */
  statecov_t state_x;
};

struct filter_data{
  td_t td;
  int update_count;
  ros::Time latest_update;
};

class UvdarKalmanAnonymous {

    private:

      bool is_initialized = false;
      std::string _uav_name_;

      std::string _output_frame_;
      double _output_framerate_;
      bool _odometry_available_, _indoor_;
      double vl, vv, sn;

      std::mutex meas_mutex;
      std::mutex filter_mutex;

      mrs_lib::lkf_t *filter;
      td_t td_template;
      std::vector<filter_data> fd;
      // | ----------------------- subscribers ---------------------- |

      ros::Subscriber sub_measurement_;

      // | ----------------------- publishers ---------------------- |
      ros::Publisher pub_filter_;
      ros::Publisher pub_filter_tent_;

      // | ------------------------ services ------------------------ |
      ros::Timer timer;
      ros::Duration filter_update_period;
      double dt;



      mrs_lib::Transformer transformer_;

  UvdarKalmanAnonymous(ros::NodeHandle nh) {
    ros::NodeHandle pnh("~");
    ros::Time::waitForValid();


    mrs_lib::ParamLoader param_loader(pnh, "UvdarKalmanAnonymous");
    param_loader.load_param("uav_name", _uav_name_);
    param_loader.load_param("output_frame", _output_frame_, std::string("local_origin"));
    param_loader.load_param("outputFramerate", _output_framerate_, double(freq));

    param_loader.load_param("indoor", _indoor_, bool(false));
    param_loader.load_param("odometryAvailable", _odometry_available_, bool(true));

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

    filter_update_period = ros::Duration(1.0 / fmax(_output_framerate_,1.0));
    dt = filter_update_period.toSec();
    timer = nh.createTimer(filter_update_period, &UvdarKalmanAnonymous::spin, this);

    transformer_ = mrs_lib::Transformer("UvdarKalmanAnonymous", _uav_name_);

    sub_measurement_  = nh.subscribe("measuredPoses", 3, &UvdarKalmanAnonymous::callbackMeasurement, this, ros::TransportHints().tcpNoDelay()); 
    pub_filter_ = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("filteredPoses", 1);
    pub_filter_tent_ = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("filteredPoses/tentative", 1);

    td_template.A <<
      1,0,0, 0,0,0,
      0,1,0, 0,0,0,
      0,0,1, 0,0,0,
      0,0,0, 1,0,0,
      0,0,0, 0,1,0,
      0,0,0, 0,0,1;

    td_template.Q <<
      vl, 0 ,0, 0, 0 ,0,
      0, vl, 0, 0, 0 ,0,
      0, 0, vv, 0, 0 ,0,
      0, 0, 0,  1, 0 ,0,
      0, 0, 0,  0, 1 ,0,
      0, 0, 0,  0, 0 ,1;

    td_template.H <<
      1,0,0,0,0,0,
      0,1,0,0,0,0,
      0,0,1,0,0,0,
      0,0,0,1,0,0,
      0,0,0,0,1,0,
      0,0,0,0,0,1;

    td_template.B = B_t();

    td_template.R <<
      1,0,0,0,  0,  0,
      0,1,0,0,  0,  0,
      0,0,1,0,  0,  0,
      0,0,0,0.3,0,  0,
      0,0,0,0,  0.3,0,
      0,0,0,0,  0,  0.3;

     filter = new mrs_lib::lkf_t(td_template.A, td_template.B, td_template.H);


    if (param_loader.loaded_successfully()) {
      is_initialized = true;
      ROS_INFO_STREAM("[UvdarKalmanAnonymous]: initiated");
    } else {
      ROS_ERROR_ONCE("PARAMS not loaded correctly, shutdown");
      nh.shutdown();
    }


  }

  void callbackMeasurement(const mrs_msgs::PoseWithCovarianceArrayStamped& msg){
    if ((int)(msg.poses.size()) < 1)
      return;
    if (DEBUG)
      ROS_INFO_STREAM("[UVDAR Kalman]: Getting " << (int)(msg.poses.size()) << " measurements...");


    mrs_msgs::PoseWithCovarianceArrayStamped msg_local;
    {
      std::scoped_lock lock(meas_mutex);
      msg_local = msg;
    }

    for (auto &meas : msg_local.poses){


    e::VectorXd poseVec(6);
    e::MatrixXd poseCov(6,6);

    poseVec(0) = meas.pose.position.x;
    poseVec(1) = meas.pose.position.y;
    poseVec(2) = meas.pose.position.z;
    if (poseVec.array().isNaN().any()){
      ROS_INFO("[UvdarKalmanAnonymous]: Discarding input, it includes Nans.");
      return;
    }

    e::Quaterniond qtemp(
        meas.pose.orientation.w,
        meas.pose.orientation.x,
        meas.pose.orientation.y,
        meas.pose.orientation.z
        );
    /* e::Vector3d tmp  = qtemp.toRotationMatrix().eulerAngles(0, 1, 2); */
    poseVec(3) = fixAngle(quatToRoll(qtemp), 0);
    poseVec(4) = fixAngle(quatToPitch(qtemp), 0);
    poseVec(5) = fixAngle(quatToYaw(qtemp), 0);

    for (int i=0; i<6; i++){
      for (int j=0; j<6; j++){
        poseCov(j,i) =  meas.covariance[6*j+i];
      }
    }

    int index;
    index = getClosestMatch(poseVec);
    if (index>=0){
      update(index, poseVec, poseCov, msg_local.header.stamp);
    } else {
      initiateNew(poseVec, poseCov, msg_local.header.stamp);
    }


  }
  }
//}


/* deleteFilter //{ */

void deleteFilter(int &index){
  if (index < 0) return;
    fd.erase(fd.begin()+index);
    index--;
}

//}

/* spin //{ */

void spin([[ maybe_unused ]] const ros::TimerEvent& unused){
  std::scoped_lock lock(filter_mutex);
  removeNANs();
  /* timeTransform(); */
  publishStates();
  for (int target=0; target<(int)(fd.size());target++){
  int targetsSeen = 0;
    /* update_counts[target]--; */
    double age = (ros::Time::now() - fd[target].latest_update).toSec();
    double decay_age;
    /* if (localizaton_mode == "ground_fire"){ */
    /*   decay_age = 2.5; */
    /* }else{ */
      decay_age = DECAY_AGE_NORMAL;
    /* } */
    if (age>decay_age){
      ROS_INFO_STREAM("[UvdarKalmanAnonymous]: Removing state " << target << ": " << fd[target].td.state_x.x.transpose() << " due to old age of " << age << " s." );
      fd.erase(fd.begin()+target);
      target--;
      continue;
    }
    fd[target].td.state_x = filter->predict(fd[target].td.state_x, u_t(), (fd[target].td.Q), filter_update_period.toSec());

  std::scoped_lock slck(filter_mutex);


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
    qtemp = e::AngleAxisd(td[target].state_x.x[3], e::Vector3d::UnitX()) * e::AngleAxisd(td[target].state_x.x[4], e::Vector3d::UnitY()) * e::AngleAxisd(td[target].state_x.x[5], e::Vector3d::UnitZ());
    /* qtemp.setRPY(currKalman[target]->getState(6), currKalman[target]->getState(7), currKalman[target]->getState(8)); */
    /* qtemp=(transformCam2Base.getRotation().inverse())*qtemp; */
    pubPose->pose.pose.orientation.x = qtemp.x();
    pubPose->pose.pose.orientation.y = qtemp.y();
    pubPose->pose.pose.orientation.z = qtemp.z();
    pubPose->pose.pose.orientation.w = qtemp.w();

    e::MatrixXd C = td[target].state_x.P;
    for (int i=0; i<6; i++){
      for (int j=0; j<6; j++){
        pubPose->pose.covariance[6*j+i] =  C(j,i);
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

}

//}

/* getClosestMatch //{ */

int getClosestMatch(e::Vector4d input){
  {
    std::scoped_lock lock(filter_mutex);
    for (auto const& td_curr : td | indexed(0)){
      /* if (td_curr.index() == (0)) */
      /*     break; */
      /* if (td_curr.index() == ((int)(td.size())-1)) */
      /*     break; */
      e::Vector3d position_i(input.x(), input.y(), input.z());
      e::Vector3d position_s(td_curr.value().state_x.x[0],td_curr.value().state_x.x[1],td_curr.value().state_x.x[2]);
      double diff_pos = (position_i-position_s).norm();
      double diff_yaw = abs(input.w()-(fixAngle(td_curr.value().state_x.x[3], input.w())));

      /* ROS_INFO_STREAM("meas: " << input << "; state: " << td_curr.value().state_x.x); */
      /* ROS_INFO_STREAM("diff_pos: " << diff_pos << "; diff_yaw: " << diff_yaw); */
      /* if ((diff_pos < POS_THRESH) && (diff_yaw < YAW_THRESH)){ */
      if (diff_pos < POS_THRESH){
        /* ROS_INFO("passed pos thresh"); */
        if  ((diff_yaw < YAW_THRESH) || (ground_mode || indoor_mode)){
          /* ROS_INFO("passed yaw thresh - index is %d", td_curr.index()); */
          return td_curr.index();
          break; //TODO if more match, select e.g. the closest
        }
        /* else */
        /* ROS_INFO("failed yaw thresh"); */
      }
      /* else */
      /* ROS_INFO("failed pos thresh"); */
    }
    }

    return -1;
  }

//}

/* initiateNew //{ */

void initiateNew(e::VectorXd x, e::MatrixXd C, ros::Time stamp){
  std::scoped_lock lock(filter_mutex);

  if (fd.size() > 20)
    return;
  int index = (int)(td.size());
  ROS_INFO_STREAM("[UvdarKalmanAnonymous]: Initiating state " << index << "  with: " << x.transpose());

  fd.push_back({td = td_template, update_count = 0, latest_update = stamp});


  fd[index].td.state_x.x[0]=x(0);
  fd[index].td.state_x.x[1]=x(1);
  fd[index].td.state_x.x[2]=x(2);
  fd[index].td.state_x.x[3]=x(3);
  fd[index].td.state_x.x[4]=x(4);
  fd[index].td.state_x.x[5]=x(5);

  fd[index].td.state_x.P = C;
  /* ROS_INFO_STREAM("[UvdarKalmanAnonymous]: Initiated state " << index << " with: " << td[index].state_x.x.transpose()); */

}

//}

/* update //{ */

void update(int index, e::VectorXd x, e::MatrixXd C, ros::Time stamp){
  std::scoped_lock lock(filter_mutex);

  /* ROS_INFO_STREAM("[UvdarKalmanAnonymous]: PRE: Upding state: " << td[index].state_x.x[3] << " with " << x.w()); */
  if (ground_mode){
    x.w() = td[index].state_x.x[3] + fmod(fixAngle(x.w(),td[index].state_x.x[3]) - td[index].state_x.x[3], M_PI_2);
  }
  /* ROS_INFO_STREAM("[UvdarKalmanAnonymous]: POST: Upding state: " << td[index].state_x.x[3] << " with " << x.w()); */
  td[index].state_x.x[3] = fixAngle(td[index].state_x.x[3], x.w());
  /* ROS_INFO_STREAM("C: " << C << " D: " << td[index].state_x.P); */
  td[index].state_x = filter->correct(td[index].state_x, x, C);
  /* ROS_INFO_STREAM("[UvdarKalmanAnonymous]: Updated state: " << td[index].state_x.x.transpose()); */
  /* td[index].state_x.P = td_template.R;; */
  /* state_tmp = currKalman[target]->predict(td[target].state_m, u_t(), (td[target].Q), dt); */
  /* td[target].state_m = currKalman[target]->correct(state_tmp, mes, td[target].R); */


  update_counts[index]++;
  latest_updates[index]=stamp;
}

//}

/* publishStates //{ */

    void publishStates(){
      /* std::scoped_lock lock(filter_mutex); */
      mrs_msgs::PoseWithCovarianceArrayStamped msg;
      mrs_msgs::PoseWithCovarianceArrayStamped msg_selected;
      mrs_msgs::PoseWithCovarianceArrayStamped msg_tent;
      mrs_msgs::ReferenceStamped  msg_ref;
      mrs_msgs::ReferenceStamped  msg_ref_final;
      msg.header.frame_id = _uav_name_+"/stable_origin";
      msg.header.stamp = ros::Time::now();
      msg_tent.header = msg.header;
      msg_selected.header = msg.header;

      auto tf = transformer_.getTransform("fcu_untilted", "stable_origin", msg.header.stamp);
      if (!tf) { 
        ROS_ERROR("[FireLocalize]: Could not obtain transform to fcu_untilted");
        return;
      }
      double curr_x = tf.value().getTransform().transform.translation.x;
      double curr_y = tf.value().getTransform().transform.translation.y;
      double curr_z = tf.value().getTransform().transform.translation.z;
      /* tf::Quaternion quat; */
      /* { */
      /*   std::scoped_lock lock(odom_msg_mutex); */
      /*   curr_x = odom_cmd.pose.position.x; */
      /*   curr_y = odom_cmd.pose.position.y; */
      /*   curr_z = odom_cmd.pose.position.z; */
      /*   quaternionMsgToTF(odom_cmd.pose.orientation, quat); */
      /* } */
      tf::Quaternion quat;
      quaternionMsgToTF(tf.value().getTransform().transform.rotation, quat);
      tf::Matrix3x3 m(quat);
      double dummy, curr_yaw;
      m.getRPY(dummy, dummy, curr_yaw);
      curr_yaw = fixAngle(curr_yaw, 0);

      geometry_msgs::PoseWithCovariance temp;
      e::Quaterniond qtemp;
      for (auto const& td_curr : td | indexed(0)){
        /* ROS_INFO_STREAM("Adding to publisher state " << td_curr.index() << " with value " << td_curr.value().state_x.x); */
        temp.pose.position.x = td_curr.value().state_x.x[0];
        temp.pose.position.y = td_curr.value().state_x.x[1];
        temp.pose.position.z = td_curr.value().state_x.x[2];

        qtemp = e::AngleAxisd(td_curr.value().state_x.x[3], e::Vector3d::UnitZ());

        // AngleAxisf(roll, Vector3f::UnitX())
        //j* AngleAxisf(pitch, Vector3f::UnitY())

        temp.pose.orientation.x = qtemp.x();
        temp.pose.orientation.y = qtemp.y();
        temp.pose.orientation.z = qtemp.z();
        temp.pose.orientation.w = qtemp.w();

        for (int m=0; m<3; m++){
          for (int n=0; n<3; n++){
            temp.covariance[6*n+m] =  td_curr.value().state_x.P(n,m);
          }
        }
        for (int m=3; m<6; m++){
          for (int n=3; n<6; n++){
            temp.covariance[6*n+m] =  (m==n?0.1:0.0);
          }
        }
        if (ground_mode)
          temp.covariance[35] = td_curr.value().state_x.P(5,5);
        if (update_counts[td_curr.index()] < 10)
          msg_tent.poses.push_back(temp);
        else{
          msg.poses.push_back(temp);
          e::Vector3d diff_vec(
              td_curr.value().state_x.x[0]-curr_x,
              td_curr.value().state_x.x[1]-curr_y,
              td_curr.value().state_x.x[2]-curr_z
              );
          if (abs(fixAngle(abs(td_curr.value().state_x.x[3] - (fixAngle(atan2(diff_vec.y(), diff_vec.x()),td_curr.value().state_x.x[3]))), 0)) > M_PI_2){
            msg_selected.poses.push_back(temp);
          }
        }


      }


      double min_dist = std::numeric_limits<double>::max();
      int min_index = -1;
      e::Vector3d diff_vec_best;
      for (auto const& td_curr : td | indexed(0)){
        if (update_counts[td_curr.index()] < 10)
          continue;
        e::Vector3d diff_vec(
            td_curr.value().state_x.x[0]-curr_x,
            td_curr.value().state_x.x[1]-curr_y,
            td_curr.value().state_x.x[2]-curr_z
            );

/*           ROS_INFO_STREAM("diff_vec: " << diff_vec.transpose()); */
/*           ROS_INFO_STREAM("td_curr" << td_curr.value().state_x.x.transpose()); */
/*           ROS_INFO_STREAM("tf " << tf.value().getTransform().transform.translation.x << ", " << tf.value().getTransform().transform.translation.y << ", " << tf.value().getTransform().transform.translation.z); */
/*           ROS_INFO_STREAM("yaw 1: " << fixAngle(atan2(diff_vec.y(), diff_vec.x()),0)); */
/*           ROS_INFO_STREAM("yaw 2: " << td_curr.value().state_x.x[3]); */
/*           ROS_INFO_STREAM("yaw diff: " << td_curr.value().state_x.x[3] - (fixAngle(atan2(diff_vec.y(), diff_vec.x()),0))); */

/*           ROS_INFO_STREAM("yaw diff from fcu: " << abs(td_curr.value().state_x.x[3] - (fixAngle(atan2(diff_vec.y(), diff_vec.x()),0)))); */
/*           ROS_INFO_STREAM("norm of dist: " << diff_vec.norm()); */
/*           ROS_INFO_STREAM("min_dist: " << min_dist); */

        if ((abs(fixAngle(abs(td_curr.value().state_x.x[3] - (fixAngle(atan2(diff_vec.y(), diff_vec.x()),td_curr.value().state_x.x[3]))), 0)) > M_PI_2) || (ground_mode)){
          if (min_dist > diff_vec.norm()){
            min_dist = diff_vec.norm();
            min_index = td_curr.index();
            diff_vec_best = diff_vec;
          }
        }

      }


        if (min_index >= 0){
      /* ROS_INFO("[UvdarKalmanAnonymous]: Got target %d", min_index); */
          got_target = true;
        }
        else{
        got_target = false;
        }

        if (!ground_mode)
        {
        /* if (!got_odom){ */
        /*   ROS_ERROR("[UvdarKalmanAnonymous]: Missing odometry command! returning"); */
        /*   return; */
        /* } */
        double water_spraying_time_current = water_spraying_time.toSec() + (water_state?(ros::Time::now() - water_last_started).toSec():0);
        /* ROS_INFO_STREAM("[UvdarKalmanAnonymous]: Water has been spraying for " << water_spraying_time_current << " s."); */

      if (server_->isActive()) {
        action_server_feedback_.water_percent = 100 * (1.0-(water_spraying_time_current/water_capacity_time));
        server_->publishFeedback(action_server_feedback_);
      }
      if (water_spraying_time_current > water_capacity_time){
        if (server_->isActive()) {
          succeedAction(0, "Water depleted");
          }
      }

        if (td.size() < 1){
          if (server_->isActive()) {
            abortAction(1, "All targets lost");
          }
        }
        if (min_index >= 0){
          if ((ros::Time::now() -latest_ray_time).toSec() < 3.0)
            hover_distance = extinguishing_distance; 
          else
            hover_distance = 2.0;

          msg_ref.header = msg.header;
          e::Vector3d norm_vec;
          if (indoor_mode){
            norm_vec.x() = curr_x - td[min_index].state_x.x[0];
            norm_vec.y() = curr_y - td[min_index].state_x.x[1];
            norm_vec.z() = 0;
            norm_vec.normalize();
            norm_vec *= hover_distance;
          }
          else{
            norm_vec.x() = cos(td[min_index].state_x.x[3]);
            norm_vec.y() = sin(td[min_index].state_x.x[3]);
            norm_vec.z() = 0;
            norm_vec *= hover_distance;
          }

          e::Vector3d ref_pos(
              td[min_index].state_x.x[0] + norm_vec.x(),
              td[min_index].state_x.x[1] + norm_vec.y(),
              td[min_index].state_x.x[2] + z_offset
              );
          geometry_msgs::PoseStamped pose;
          pose.pose.position.x = ref_pos.x();
          pose.pose.position.y = ref_pos.y();
          pose.pose.position.z = ref_pos.z();
          pose.header = msg.header;
          pub_ref_point.publish(pose);
          msg_ref_final.header = msg.header;
          msg_ref_final.reference.position = pose.pose.position;
          msg_ref_final.reference.yaw = fixAngle(td[min_index].state_x.x[3]-M_PI, 0);
          e::Vector2d ref_diff(
              ref_pos.x() - curr_x, 
              ref_pos.y() - curr_y
              );
          e::Vector2d ref_vec_centering;
          /* e::Vector3d target_diff( */
          /*     td[min_index].state_x.x[0] - curr_x, */
          /*     td[min_index].state_x.x[1] - curr_y, */
          /*     td[min_index].state_x.x[2] - curr_z); */

          double z_diff  = ref_pos.z() - curr_z;

          if (test_mode)
            ROS_INFO_STREAM_THROTTLE(2,"[UvdarKalmanAnonymous]: The reference is in " << ref_diff.transpose() << " from me");
          /* reached_position = true; */
          /* hysteresis = 10.0; */

          double angle_from_normal = acos(norm_vec.normalized().dot((-diff_vec_best).normalized()));
          //TODO fix parallax effect. Maybe.

          double desired_max_angle_offset = (2*atan(HOLE_RADIUS_SMALL/diff_vec_best.norm()));
          double camera_ray_closeness = rayCloseness(e::Vector3d(extinguishing_distance, -y_offset, -z_offset));
          double camera_ray_closeness_horizontal = rayClosenessHorizontal(e::Vector3d(extinguishing_distance, -y_offset, -z_offset));
          double camera_ray_closeness_vertical = rayClosenessVertical(e::Vector3d(extinguishing_distance, -y_offset, -z_offset));
          double latest_ray_time_diff = (ros::Time::now() -latest_ray_time).toSec();
          if (
              (
               ((abs(diff_vec_best.norm() - hover_distance) > ( hover_distance*hysteresis_distance )) ||
                (angle_from_normal > (wall_angle_range))) ||
               (!reached_position)
              ) && (!test_mode)
              ){ //if we are close, it is not worth moving - tilting fucks up our aim
            stopWater();
            e::Vector2d ref_diff_norm = ref_diff.normalized();
            ref_vec_centering = e::Vector2d(
                curr_x + (ref_diff_norm.x())*(filter_update_period.toSec()*max_centering_speed),
                curr_y + (ref_diff_norm.y())*(filter_update_period.toSec()*max_centering_speed)
                );
            ROS_INFO_STREAM("SHIT: " << ref_vec_centering.transpose() << " : "<< curr_x << ":" << curr_y << ":" << ref_diff_norm.transpose()<<":"<<filter_update_period.toSec()<<":"<<max_centering_speed );
            msg_ref.reference.position.x = ref_vec_centering.x();
            msg_ref.reference.position.y = ref_vec_centering.y();
            /* msg_ref.reference.position.z = ref_pos.z(); */
            if ((abs(diff_vec_best.norm() - hover_distance) < ( hover_distance*near_distance )) &&
                (angle_from_normal < (near_angle))){ 
              reached_position = true;
            } else {
              reached_position = false;
            }
          } else {
            /* ROS_INFO_STREAM("[UvdarKalmanAnonymous]: Time diff: " << latest_ray_time_diff); */
            if (
                /* ( ((ros::Time::now() - latest_updates[min_index]).toSec()) < 0.3 ) && */ 
                /* (abs(z_diff) < HOLE_RADIUS_SMALL*2) && //TODO this condition may make us unreasonably frugal */
                /* (abs(fixAngle(fixAngle(atan2(diff_vec_best.y(),diff_vec_best.x()), curr_yaw) - curr_yaw,0) ) < 2*desired_max_angle_offset ) && //rad */
                (camera_ray_closeness < desired_max_angle_offset) && (latest_ray_time_diff < 0.3)
               ){
              /* ROS_INFO_STREAM("[UvdarKalmanAnonymous]: Shooting with conditions: z_diff " << z_diff << " time_diff: " << (ros::Time::now() - latest_updates[min_index]).toSec() << " curr_yaw: " << curr_yaw << " target_yaw_calc: " << fixAngle(atan2(diff_vec_best.y(),diff_vec_best.x()), curr_yaw) << " yaw diff: " <<fixAngle(fixAngle(atan2(diff_vec_best.y(),diff_vec_best.x()), curr_yaw) - curr_yaw,0) << " vs.: " <<  desired_max_angle_offset << " diff_vec_best: " <<diff_vec_best.transpose() << " camera_ray_difference_angle: " << camera_ray_closeness << " latest_ray_time_diff " << latest_ray_time_diff); */
              startWater();
            }
            else{
              stopWater();
            }
            msg_ref.reference.position.x = curr_x;
            msg_ref.reference.position.y = curr_y;
            /* msg_ref.reference.position.z = tf.value().getTransform().transform.translation.z; */
          }
          /* msg_ref.reference.yaw = fixAngle(td[min_index].state_x.x[3] + M_PI, 0); */
          //z and yaw do not tilt uav on their own 
          ROS_INFO_STREAM("curr_yaw: " << curr_yaw);
          if ((latest_ray_time_diff < 0.3) && ((int)(camera_rays.size()) > 0)){
            msg_ref.reference.yaw = curr_yaw + camera_ray_closeness_horizontal;
            ROS_INFO_STREAM("[UvdarKalmanAnonymous]: Controlling yaw by " << (color_state?"realsense":"thermo") << " camera: "<< camera_ray_closeness_horizontal);
          } else if ( (ros::Time::now() - latest_updates[min_index]).toSec() < 3.0 ) {
            if (reached_position){
              msg_ref.reference.yaw = fixAngle(atan2(diff_vec_best.y(),diff_vec_best.x())-atan2(-y_offset, extinguishing_distance), 0);
              ROS_INFO_STREAM("[UvdarKalmanAnonymous]: Controlling yaw by filter: "<< diff_vec_best.transpose() << " : " << diff_vec_best.norm() << " : " << y_offset);
            /* last_yaw = msg_ref.reference.yaw; */
            } else {
              e::Vector2d diff_vec_next(
                  td[min_index].state_x.x[0] - ref_vec_centering.x(),
                  td[min_index].state_x.x[1] - ref_vec_centering.y()
                  );
              msg_ref.reference.yaw = fixAngle(atan2(diff_vec_next.y(),diff_vec_next.x())-atan2(-y_offset, extinguishing_distance), 0);
              ROS_INFO_STREAM("[UvdarKalmanAnonymous]: Controlling yaw by reference: "<<ref_vec_centering.transpose() << " : " << atan2(ref_vec_centering.y(),ref_vec_centering.x()) << " : " << atan2(-y_offset, extinguishing_distance) << " : " << y_offset);
            }
          } else {
            msg_ref.reference.yaw = curr_yaw;
          }
          ROS_INFO_STREAM("reference yaw: " << msg_ref.reference.yaw);

          if ( ((abs(diff_vec_best.norm() - hover_distance) < ( hover_distance*hysteresis_distance )) &&
                (angle_from_normal < (wall_angle_range)) )  &&  (latest_ray_time_diff > 0.3)
             ){ 
              raiseThreshold();
          } else {
            lowerThreshold();
          }
          /* if (latest_ray_time_diff < 0.3){ */
          /*   msg_ref.reference.position.z = curr_z + tan(camera_ray_closeness_vertical); */
          /* } else { */
          if (!camera_z_control){
            msg_ref.reference.position.z = ref_pos.z(); 
          } else {
            msg_ref.reference.position.z = curr_z-tan(camera_ray_closeness_vertical)*extinguishing_distance; 
          }
          /* } */
          if (enable_control)
            pub_reference.publish(msg_ref);
          else
            pub_desire.publish(msg_ref_final);
        }
        else{
          stopWater();
        }

      if (!enable_control) {
        stopWater();
      }
      } else { //Ground mode
        if (got_target) {
          if (enable_control){


            double cmd_yaw, cmd_x, cmd_y,cmd_z;
            double planarDist;
            {
              std::scoped_lock lock(odom_msg_mutex);
              e::Quaterniond qtemp(odom_cmd.pose.orientation.w, odom_cmd.pose.orientation.x, odom_cmd.pose.orientation.y, odom_cmd.pose.orientation.z );
              cmd_yaw = fixAngle(quatToYaw(qtemp), 0);
              cmd_x = odom_cmd.pose.position.x;
              cmd_y = odom_cmd.pose.position.y;
              cmd_z = odom_cmd.pose.position.z;

              planarDist = e::Vector2d(
                  td[min_index].state_x.x[0] - cmd_x,
                  td[min_index].state_x.x[1] - cmd_y
                  ).norm();
            }

            if ((ros::Time::now() - latest_updates[min_index]).toSec() < 3.0){
              if ((planarDist > 0.1) && (!reached_position)){
                ROS_INFO_STREAM_THROTTLE(1.0,"[UvdarKalmanAnonymous]: Approaching ground target.");
                msg_ref.header = msg.header;
                msg_ref.reference.position.x = td[min_index].state_x.x[0];
                msg_ref.reference.position.y = td[min_index].state_x.x[1];
                msg_ref.reference.position.z = GROUND_ESTIM_HEIGHT;
                msg_ref.reference.yaw = cmd_yaw;
              } else {
                if (!covarianceSmall(min_index)){
                  ROS_INFO_STREAM_THROTTLE(1.0,"[UvdarKalmanAnonymous]: Doing estimation pirouette.");
                  reached_position = true;
                  /* msg_ref.reference.position.x = cmd_x; */
                  /* msg_ref.reference.position.y = cmd_y; */
                  msg_ref.reference.position.x = td[min_index].state_x.x[0];
                  msg_ref.reference.position.y = td[min_index].state_x.x[1];
                  msg_ref.reference.position.z = GROUND_ESTIM_HEIGHT;
                  /* msg_ref.reference.position.z = cmd_z; */
                  msg_ref.reference.yaw = cmd_yaw + filter_update_period.toSec()*GROUND_ESTIM_ROT_RATE;
                }
                else {
                  ROS_INFO_STREAM_THROTTLE(1.0,"[UvdarKalmanAnonymous]: I want to drop blanket.");
                  if ((ros::Time::now() - latest_updates[min_index]).toSec() < 1.0){
                    ROS_INFO_STREAM_THROTTLE(1.0,"[UvdarKalmanAnonymous]: Triggering blanket dropping.");
                    triggerBlanketDropping(
                        td[min_index].state_x.x[0],
                        td[min_index].state_x.x[1],
                        td[min_index].state_x.x[3]);
                    /* enable_control = false; */
                  }
                }
              }
            } else {
              ROS_INFO_STREAM_THROTTLE(1.0,"[UvdarKalmanAnonymous]: Triggering return to sweeping.");
              triggerReturn();
              deleteFilter(min_index);
            }

              if (enable_control){
                ROS_INFO_STREAM_THROTTLE(1.0,"[UvdarKalmanAnonymous]: Controlling UAV.");
                pub_reference.publish(msg_ref);
              }
              else{
                ROS_INFO_STREAM_THROTTLE(1.0,"[UvdarKalmanAnonymous]: Asking for control.");
                pub_desire.publish(msg_ref);
              }
            }
        }
      }


      pub_filter.publish(msg);
      if (min_index >= 0){
        pub_targets.publish(msg);
      }
      pub_filter_tent.publish(msg_tent);

    }

//}

/* removeNANs //{ */

void removeNANs(){
  for (int i=0; i<(int)(td.size());i++){
    if ( (td[i].state_x.x.array().isNaN().any() ) ||
        (td[i].state_x.P.array().isNaN().any() )){
      td.erase(td.begin()+i);
      update_counts.erase(update_counts.begin()+i);
      latest_updates.erase(latest_updates.begin()+i);
      i--;
    }
  }
}

//}

/* quatRotateRoll //{ */

double quatToRoll(e::Quaterniond q){
  e::Matrix33d m = q.matrix();
  return atan2(m(2,1),m(2,2));
}

//}

/* quatRotatePitch //{ */

double quatToPitch(e::Quaterniond q){
  e::Matrix33d m = q.matrix();
  return atan2( -m(2,0), sqrt( m(2,1)*m(2,1) +m(2,2)*m(2,2) )  );
}

//}

/* quatRotateYaw //{ */

double quatToYaw(e::Quaterniond q){
  e::Matrix33d m = q.matrix();
  return atan2(m(1,0),m(0,0));
}

//}


/* fixAngle //{ */

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

//}
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "uvdar_kalman_anonymous");
  ros::NodeHandle nodeA;
  UvdarKalmanAnonymous        kl(nodeA);

  ROS_INFO("[UvdarKalmanAnonymous]: filter node initiated");

  ros::spin();

  return 0;
}
