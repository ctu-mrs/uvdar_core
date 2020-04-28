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
#define minMeasurementsToValidation 2
#define POS_THRESH 2.0
#define YAW_THRESH 1.5

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






    if (DEBUG){
      ROS_INFO_STREAM("State: ");
      ROS_INFO_STREAM("\n" << fd[target].td.state_x.x);
    }

  }

}

//}

/* getClosestMatch //{ */

int getClosestMatch(e::Vector4d input){
  {
    std::scoped_lock lock(filter_mutex);
    for (auto const& fd_curr : fd | indexed(0)){
      /* if (fd_curr.td.index() == (0)) */
      /*     break; */
      /* if (fd_curr.td.index() == ((int)(td.size())-1)) */
      /*     break; */
      e::Vector3d position_i(input.x(), input.y(), input.z());
      e::Vector3d position_s(fd_curr.value().td.state_x.x[0],fd_curr.value().td.state_x.x[1],fd_curr.value().td.state_x.x[2]);
      double diff_pos = (position_i-position_s).norm();
      double diff_yaw = abs(input.w()-(fixAngle(fd_curr.value().td.state_x.x[3], input.w())));

      /* ROS_INFO_STREAM("meas: " << input << "; state: " << fd_curr.value().td.state_x.x); */
      /* ROS_INFO_STREAM("diff_pos: " << diff_pos << "; diff_yaw: " << diff_yaw); */
      /* if ((diff_pos < POS_THRESH) && (diff_yaw < YAW_THRESH)){ */
      if (diff_pos < POS_THRESH){
        /* ROS_INFO("passed pos thresh"); */
        /* if  ((diff_yaw < YAW_THRESH) || (ground_mode || indoor_mode)){ */
        if  (true){ // we will not match based on orientation
          /* ROS_INFO("passed yaw thresh - index is %d", fd_curr.value().td); */
          return fd_curr.index();
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
  int index = (int)(fd.size());
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
      mrs_msgs::PoseWithCovarianceArrayStamped msg_tent;
      msg.header.frame_id = _output_frame_;
      msg.header.stamp = ros::Time::now() - questionable;
      msg_tent.header = msg.header;

      /* auto tfr = transformer_.getTransform("fcu_untilted", "stable_origin", msg.header.stamp); */
      /* if (!tfr) { */ 
      /*   ROS_ERROR("[UvdarKalmanAnonymous]: Could not obtain transform to fcu_untilted"); */
      /*   return; */
      /* } */
      tf::Quaternion quat;
      quaternionMsgToTF(tf.value().getTransform().transform.rotation, quat);
      tf::Matrix3x3 m(quat);
      double dummy, curr_yaw;
      m.getRPY(dummy, dummy, curr_yaw);
      curr_yaw = fixAngle(curr_yaw, 0);

      geometry_msgs::PoseWithCovariance temp;
      e::Quaterniond qtemp;
      for (auto const& fd_curr : fd | indexed(0)){
        temp.pose.position.x = fd_curr.td.value().state_x.x[0];
        temp.pose.position.y = fd_curr.td.value().state_x.x[1];
        temp.pose.position.z = fd_curr.td.value().state_x.x[2];

        qtemp = e::AngleAxisd(fd_curr.td.state_x.x[3], e::Vector3d::UnitX()) * e::AngleAxisd(fd_curr.td.state_x.x[4], e::Vector3d::UnitY()) * e::AngleAxisd(fd_curr.td.state_x.x[5], e::Vector3d::UnitZ());

        temp.pose.orientation.x = qtemp.x();
        temp.pose.orientation.y = qtemp.y();
        temp.pose.orientation.z = qtemp.z();
        temp.pose.orientation.w = qtemp.w();

        for (int m=0; m<6; m++){
          for (int n=0; n<6; n++){
            temp.covariance[6*n+m] =  fd_curr.td.value().state_x.P(n,m);
          }
        }
        if (update_counts[fd_curr.td.index()] < minMeasurementsToValidation)
          msg_tent.poses.push_back(temp);
        else{
          msg.poses.push_back(temp);
        }


      }

      pub_filter_.publish(msg);
      pub_filter_tent_.publish(msg_tent);

    }

//}

/* removeNANs //{ */

void removeNANs(){
  for (int i=0; i<(int)(fd.size());i++){
    if ( (fd[i].td.state_x.x.array().isNaN().any() ) ||
        (fd[i].td.state_x.P.array().isNaN().any() )){
      fd.erase(fd.begin()+i);
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
