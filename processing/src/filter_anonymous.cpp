#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
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
#define DECAY_AGE_NORMAL 5.0
#define DECAY_AGE_UNVALIDATED 1.0
#define minMeasurementsToValidation 10
#define POS_THRESH 2.0
#define MAH_THRESH 2.0
#define YAW_THRESH 1.5

#define MATCH_LEVEL_THRESHOLD_ASSOCIATE 0.3
#define MATCH_LEVEL_THRESHOLD_REMOVE 0.5

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
  ros::Time latest_measurement;
  unsigned long long int id;
};

class UvdarKalmanAnonymous {

    private:

      bool is_initialized = false;
      std::string _uav_name_;

      std::string _output_frame_;
      int _input_count_;
      double _output_framerate_;
      bool _odometry_available_, _indoor_;
      double vl, vv, sn;

      std::mutex meas_mutex;
      std::mutex filter_mutex;
      std::mutex transformer_mutex;

      mrs_lib::lkf_t *filter;
      td_t td_template;
      std::vector<filter_data> fd;
      // | ----------------------- subscribers ---------------------- |

      std::vector<ros::Subscriber> sub_measurements_;

      // | ----------------------- publishers ---------------------- |
      ros::Publisher pub_filter_;
      ros::Publisher pub_filter_tent_;

      // | ------------------------ services ------------------------ |
      ros::Timer timer;
      ros::Duration filter_update_period;
      double dt;
      unsigned long long int latest_id = 0;



      mrs_lib::Transformer transformer_;

    public:

  UvdarKalmanAnonymous(ros::NodeHandle nh) {
    ros::NodeHandle pnh("~");
    ros::Time::waitForValid();


    mrs_lib::ParamLoader param_loader(pnh, "UvdarKalmanAnonymous");
    param_loader.loadParam("uav_name", _uav_name_);
    param_loader.loadParam("output_frame", _output_frame_, std::string("local_origin"));
    param_loader.loadParam("output_framerate", _output_framerate_, double(freq));

    param_loader.loadParam("indoor", _indoor_, bool(false));
    param_loader.loadParam("odometryAvailable", _odometry_available_, bool(true));

    param_loader.loadParam("input_count", _input_count_);

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

    for (int i = 0; i < _input_count_; ++i) {
      ROS_INFO_STREAM("[PoseReporter]: Subscribing to " << "measuredPoses"+std::to_string(i+1));
      sub_measurements_.push_back(pnh.subscribe("measuredPoses"+std::to_string(i+1), 3, &UvdarKalmanAnonymous::callbackMeasurement, this, ros::TransportHints().tcpNoDelay())); 
    }
    pub_filter_ = pnh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("filteredPoses", 1);
    pub_filter_tent_ = pnh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("filteredPoses/tentative", 1);

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


    if (param_loader.loadedSuccessfully()) {
      is_initialized = true;
      ROS_INFO_STREAM("[UvdarKalmanAnonymous]: initiated");
    } else {
      ROS_ERROR_ONCE("PARAMS not loaded correctly, shutdown");
      nh.shutdown();
    }


  }

    private:

  void callbackMeasurement(const mrs_msgs::PoseWithCovarianceArrayStamped& msg){
    if ((int)(msg.poses.size()) < 1)
      return;
    if (DEBUG)
      ROS_INFO_STREAM("[UvdarKalmanAnonymous]: Getting " << (int)(msg.poses.size()) << " measurements...");


    mrs_msgs::PoseWithCovarianceArrayStamped msg_local;
    {
      std::scoped_lock lock(meas_mutex);
      msg_local = msg;
    }

    std::optional<mrs_lib::TransformStamped> tf;
    {
      std::scoped_lock lock(transformer_mutex);
      tf = transformer_.getTransform(msg_local.header.frame_id, _output_frame_, msg.header.stamp);
    }
    if (!tf) { 
      ROS_ERROR("[UvdarKalmanAnonymous]: Could not obtain transform from %s to %s",msg_local.header.frame_id.c_str(), _output_frame_.c_str());
      return;
    }
    std::vector<std::pair<e::VectorXd,e::MatrixXd>> meas_converted;
    for (auto &meas : msg_local.poses){
      e::VectorXd poseVec(6);
      e::MatrixXd poseCov(6,6);

      std::optional<geometry_msgs::PoseWithCovarianceStamped> meas_t;
      geometry_msgs::PoseWithCovarianceStamped meas_s;
      meas_s.pose.pose = meas.pose;
      meas_s.pose.covariance = meas.covariance;
      meas_s.header = msg_local.header;
      {
        std::scoped_lock lock(transformer_mutex);
        meas_t = transformer_.transform(tf.value(), meas_s);
      }
      if (!meas_t){
        ROS_INFO_STREAM("[UvdarKalmanAnonymous]: Failed to get transformation for measurement, returning.");
        return;
      }

      poseVec(0) = meas_t.value().pose.pose.position.x;
      poseVec(1) = meas_t.value().pose.pose.position.y;
      poseVec(2) = meas_t.value().pose.pose.position.z;
      e::Quaterniond qtemp(
          meas_t.value().pose.pose.orientation.w,
          meas_t.value().pose.pose.orientation.x,
          meas_t.value().pose.pose.orientation.y,
          meas_t.value().pose.pose.orientation.z
          );
      /* e::Vector3d tmp  = qtemp.toRotationMatrix().eulerAngles(0, 1, 2); */
      poseVec(3) = fixAngle(quatToRoll(qtemp), 0);
      poseVec(4) = fixAngle(quatToPitch(qtemp), 0);
      poseVec(5) = fixAngle(quatToYaw(qtemp), 0);

      if (poseVec.array().isNaN().any()){
        ROS_INFO("[UvdarKalmanAnonymous]: Discarding input, it includes Nans.");
        return;
      }

    /* e::Vector3d tmp  = qtemp.toRotationMatrix().eulerAngles(0, 1, 2); */
        ROS_INFO_STREAM("[UvdarKalmanAnonymous]: eulerAngles give: " << qtemp.toRotationMatrix().eulerAngles(0, 1, 2).transpose() << " while my angles are: " << poseVec.bottomRows(3).transpose());

      for (int i=0; i<6; i++){
        for (int j=0; j<6; j++){
          poseCov(j,i) =  meas_t.value().pose.covariance[6*j+i];
        }
      }

      meas_converted.push_back({poseVec,poseCov});
      /* int index; */
      /* double mah_distance; */
      /* {indexmah_distance} = getClosestMatch(poseVec); */
      /* if (index>=0){ */
      /*   update(index, poseVec, poseCov, msg_local.header.stamp); */
      /* } else { */
      /*   initiateNew(poseVec, poseCov, msg_local.header.stamp); */
      /* } */

    }

    applyMeasurements(meas_converted, msg_local.header);
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
  removeOverlaps();
  /* timeTransform(); */
  for (int target=0; target<(int)(fd.size());target++){
  int targetsSeen = 0;
    /* update_counts[target]--; */
    double age = (ros::Time::now() - fd[target].latest_measurement).toSec();
    ROS_INFO("[UvdarKalmanAnonymous]: Age of %d is %f", target, age);
    double decay_age;
    /*   decay_age = 2.5; */
    /* }else{ */
    if (fd[target].update_count > minMeasurementsToValidation){
      decay_age = DECAY_AGE_NORMAL;
    }
    else {
      decay_age = DECAY_AGE_UNVALIDATED;
    }
    /* } */
    if (age>decay_age){
      ROS_INFO_STREAM("[UvdarKalmanAnonymous]: Removing state " << target << ": " << fd[target].td.state_x.x.transpose() << " due to old age of " << age << " s." );
      fd.erase(fd.begin()+target);
      target--;
      continue;
    }
    fd[target].td.state_x = filter->predict(fd[target].td.state_x, u_t(), (fd[target].td.Q), std::fmin((ros::Time::now()-fd[target].latest_measurement).toSec(), dt));
    fd[target].latest_update = ros::Time::now();



    if (DEBUG){
      ROS_INFO_STREAM("State: ");
      ROS_INFO_STREAM("\n" << fd[target].td.state_x.x);
    }

  }
  publishStates();

}

//}

/* getClosestMatch //{ */

/* std::pair<int, double> getClosestMatch(e::VectorXd input){ */
/*   { */
/*     std::scoped_lock lock(filter_mutex); */
/*     int best_match_index = -1; */
/*     double best_match_distance = std::numeric_limits<double>::max();; */
/*     for (auto const& fd_curr : fd | indexed(0)){ */
/*       /1* if (fd_curr.td.index() == (0)) *1/ */
/*       /1*     break; *1/ */
/*       /1* if (fd_curr.td.index() == ((int)(td.size())-1)) *1/ */
/*       /1*     break; *1/ */
/*       e::Vector3d position_i(input.x(), input.y(), input.z()); */
/*       e::Vector3d position_s(fd_curr.value().td.state_x.x[0],fd_curr.value().td.state_x.x[1],fd_curr.value().td.state_x.x[2]); */
/*       double diff_pos = (position_i-position_s).norm(); */
/*       double md = mahalanobis_distance2( */
/*           input.topRows(3), */
/*           fd_curr.value().td.state_x.x.topRows(3), */
/*           /1* currKalman[target]->getCovariance().topLeftCorner(3, 3) *1/ */
/*           fd_curr.value().td.state_x.x.topLeftCorner(3, 3) */
/*           ); */
/*       /1* double diff_yaw = abs(input.w()-(fixAngle(fd_curr.value().td.state_x.x[3], input.w()))); *1/ */

/*       /1* ROS_INFO_STREAM("meas: " << input << "; state: " << fd_curr.value().td.state_x.x); *1/ */
/*       /1* ROS_INFO_STREAM("diff_pos: " << diff_pos << "; diff_yaw: " << diff_yaw); *1/ */
/*       /1* if ((diff_pos < POS_THRESH) && (diff_yaw < YAW_THRESH)){ *1/ */
/*       /1* if (diff_pos < POS_THRESH){ *1/ */
/*     ROS_INFO_STREAM("[UvdarKalmanAnonymous]: Mah. distance of " << input.topRows(3).transpose() << " from " << fd_curr.value().td.state_x.x.topRows(3).transpose() << " is " << md); */
/*       if (md < POS_THRESH){ */
/*         if (md<best_match_distance) */
/*           best_match_distance = md; */
/*       } */
/*       /1* else *1/ */
/*       /1* ROS_INFO("failed pos thresh"); *1/ */
/*     } */
/*     } */

/*     return {best_match_index, best_match_distance} */
/*   } */

//}

/* initiateNew //{ */

void initiateNew(e::VectorXd x, e::MatrixXd C, ros::Time stamp){
  /* std::scoped_lock lock(filter_mutex); */

  /* ROS_INFO("[%s]: HERE G", ros::this_node::getName().c_str()); */
  if (fd.size() > 20)
    return;
  int index = (int)(fd.size());
  ROS_INFO_STREAM("[UvdarKalmanAnonymous]: Initiating state " << index << "  with: " << x.transpose());

  fd.push_back({.td = td_template, .update_count = 0, .latest_update = stamp, .latest_measurement = stamp, .id = (latest_id++)});


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
  /* ROS_INFO_STREAM("[UvdarKalmanAnonymous]: POST: Upding state: " << td[index].state_x.x[3] << " with " << x.w()); */
  fd[index].td.state_x = filter->predict(fd[index].td.state_x, u_t(), (fd[index].td.Q), std::fmin((stamp-fd[index].latest_update).toSec(),(stamp-fd[index].latest_measurement).toSec()));
  fd[index].td.state_x.x[3] = fixAngle(fd[index].td.state_x.x[3], x[3]);
  fd[index].td.state_x.x[4] = fixAngle(fd[index].td.state_x.x[4], x[4]);
  fd[index].td.state_x.x[5] = fixAngle(fd[index].td.state_x.x[5], x[5]);
  /* ROS_INFO_STREAM("C: " << C << " D: " << td[index].state_x.P); */
  fd[index].td.state_x = filter->correct(fd[index].td.state_x, x, C);
  /* ROS_INFO_STREAM("[UvdarKalmanAnonymous]: Updated state: " << td[index].state_x.x.transpose()); */
  /* td[index].state_x.P = td_template.R;; */
  /* state_tmp = currKalman[target]->predict(td[target].state_m, u_t(), (td[target].Q), dt); */
  /* td[target].state_m = currKalman[target]->correct(state_tmp, mes, td[target].R); */


  fd[index].update_count++;
  fd[index].latest_measurement=stamp;
}


/* gaussJointMaxVal //{ */

double gaussJointMaxVal(e::MatrixXd si0,e::MatrixXd si1,e::VectorXd mu0,e::VectorXd mu1){
  bool scaled = true;
  int k=mu0.size();
  auto K=si0*(si0+si1).inverse();
  // mu0
  // mu1
  // mu1-mu0
  auto d0=K*(mu1-mu0);
  auto d1=(K-e::MatrixXd::Identity(K.rows(),K.rows()))*(mu1-mu0);
  /* ROS_INFO_STREAM("[UvdarKalmanAnonymous]: K: " << K << " d0: " << d0.transpose() << " d1: " << d1.transpose()); */
  // exp((-1/2)*( (d0'*inv(si0)*d0) ));
  // exp((-1/2)*(  (d1'*inv(si1)*d1) ));
  double N;
  if (scaled){
    auto N_v = ((-0.5)*( (d0.transpose()*(si0).inverse()*d0) + (d1.transpose()*(si1).inverse()*d1) ));
    /* ROS_INFO_STREAM("[UvdarKalmanAnonymous]: N_v: " << N_v); */
    N=exp(N_v(0));
  }
  else{
    auto N_v = ((-0.5)*( (d0.transpose()*(si0).inverse()*d0) + (d1.transpose()*(si1).inverse()*d1) ));
    N=(1.0/pow((2*M_PI),k)*sqrt((si0).determinant()*(si1).determinant()))*exp(N_v(0));
  }
  if (isnan(N))
    ROS_INFO("[UvdarKalmanAnonymous]: Joint Gaussian value came out NaN");

  return N;

}
//}


//}

/* applyMeasurements //{ */

void applyMeasurements(std::vector<std::pair<e::VectorXd,e::MatrixXd>> &measurements, std_msgs::Header header){
    /* ROS_INFO("[%s]: HERE A", ros::this_node::getName().c_str()); */
  std::scoped_lock lock(filter_mutex);
  if (fd.size() == 0){
    for (auto const& measurement_curr : measurements | indexed(0)){
      /* ROS_INFO("[%s]: HERE F", ros::this_node::getName().c_str()); */
      initiateNew(measurement_curr.value().first, measurement_curr.value().second, header.stamp);
    }
    return;
  }

    /* ROS_INFO("[%s]: HERE B", ros::this_node::getName().c_str()); */
  /* e::MatrixXd reduction_matrix(measurements.size(),fd.size()); */
  e::MatrixXd match_matrix(measurements.size(),fd.size());
  std::vector<std::vector<statecov_t>> tentative_states;

  for (auto const& measurement_curr : measurements | indexed(0)){
    tentative_states.push_back(std::vector<statecov_t>());
    /* auto eigens = measurement_curr.value().second.topLeftCorner(3,3).eigenvalues(); */
    /* double position_cov_size_init = (eigens.topLeftCorner(3, 1)).norm(); */
    for (auto const& state_curr : fd | indexed(0)){

      tentative_states.back().push_back(filter->predict(state_curr.value().td.state_x, u_t(), (state_curr.value().td.Q), std::fmin((header.stamp-state_curr.value().latest_update).toSec(),(header.stamp-state_curr.value().latest_measurement).toSec())));
      tentative_states.back().back().x[3] = fixAngle(state_curr.value().td.state_x.x[3], measurement_curr.value().first[3]);
      tentative_states.back().back().x[4] = fixAngle(state_curr.value().td.state_x.x[4], measurement_curr.value().first[4]);
      tentative_states.back().back().x[5] = fixAngle(state_curr.value().td.state_x.x[5], measurement_curr.value().first[5]);


      /* eigens = tentative_states.back().back().P.topLeftCorner(3,3).eigenvalues(); */
      /* double position_cov_size_finit = (eigens.topLeftCorner(3, 1)).norm(); */

      /* reduction_matrix(measurement_curr.index(),state_curr.index()) = ((position_cov_size_init-position_cov_size_finit)/position_cov_size_init); */

      match_matrix(measurement_curr.index(),state_curr.index()) = gaussJointMaxVal(
          measurement_curr.value().second.topLeftCorner(3,3),
          tentative_states.back().back().P.topLeftCorner(3,3),
          measurement_curr.value().first.topRows(3),
          tentative_states.back().back().x.topRows(3)
          );
      tentative_states.back().back() = filter->correct(tentative_states.back().back(), measurement_curr.value().first, measurement_curr.value().second);
      double dt_t = 0.1;
      if ((ros::Time::now() - state_curr.value().latest_measurement).toSec() < dt_t) // just in case - in simulation the camera outputs follow one another immediately, so no inflation happens in between
        tentative_states.back().back() = filter->predict(tentative_states.back().back(), u_t(), (state_curr.value().td.Q), dt_t); // just to fatten it for next matching
    }
  }

  ROS_INFO_STREAM("[UvdarKalmanAnonymous]: match_matrix: " << std::endl <<match_matrix);

  std::vector<std::pair<int,int>> matches;

  int best_index;
  /* double best_reduction; */
  double best_match_level;
  double best_update_count;
  /* double curr_reduction; */
  double curr_match_level;
  for (int f_i = 0; f_i < (int)(fd.size()); f_i++) {
    best_update_count = -1;
    best_index = -1;
    best_match_level = -1;
    for (int m_i = 0; m_i < (int)(measurements.size()); m_i++) {
      curr_match_level = match_matrix(m_i,f_i);  
      if ((curr_match_level > MATCH_LEVEL_THRESHOLD_ASSOCIATE) && ( (fd[f_i].update_count > best_update_count) || ((fd[f_i].update_count == best_update_count) && (curr_match_level > best_match_level)) ) ){
        best_match_level = curr_match_level;
        best_index = m_i;
        best_update_count = fd[f_i].update_count;
      }
    }

    if (best_index >= 0){
      /* ROS_INFO_STREAM("[UvdarKalmanAnonymous]: selecting m.: " <<best_index << " for f.: " << f_i); */
      matches.push_back({best_index,f_i});
      /* for (int f_j = f_i+1; f_j < (int)(fd.size()); f_j++) { */
      for (int f_j = 0; f_j < (int)(fd.size()); f_j++) {
        /* if (match_matrix(best_index,f_j) > MATCH_LEVEL_THRESHOLD_ASSOCIATE){ */
          match_matrix(best_index,f_j) = std::nan("");
        /* } */
      }
    }
    for (int m_j = 0; m_j < (int)(measurements.size()); m_j++) {
      if (match_matrix(m_j,f_i) > MATCH_LEVEL_THRESHOLD_ASSOCIATE){
        match_matrix(m_j,f_i) = std::nan("");
      }
    }
  }

    /* ROS_INFO("[%s]: HERE C", ros::this_node::getName().c_str()); */
  for( auto& match_curr : matches){
  ROS_INFO_STREAM("[UvdarKalmanAnonymous]: Updating state: " << match_curr.second << " with " << measurements[match_curr.first].first.transpose());
  ROS_INFO_STREAM("[UvdarKalmanAnonymous]: This yelds state: " << tentative_states[match_curr.first][match_curr.second].x.topRows(6).transpose());
    fd[match_curr.second].td.state_x = tentative_states[match_curr.first][match_curr.second];
    fd[match_curr.second].update_count++;
    fd[match_curr.second].latest_measurement= header.stamp;
  }

  ROS_INFO_STREAM("[UvdarKalmanAnonymous]: match_matrix post: " << std::endl <<match_matrix);
  int fd_size_orig = (int)(fd.size());
  for (int f_i = 0; f_i < fd_size_orig; f_i++) {
    bool found_unused_measurement = false;
    for (int m_i = 0; m_i < (int)(measurements.size()); m_i++) {
      if (!found_unused_measurement){
      ROS_INFO_STREAM("[UvdarKalmanAnonymous]: match_matrix at: [" << m_i << ":" << f_i << "] is: " << match_matrix(m_i,f_i));
      if (!isnan(match_matrix(m_i,f_i))){
          initiateNew(measurements[m_i].first, measurements[m_i].second, header.stamp);
          found_unused_measurement = true;
      }
      else{
        match_matrix(m_i,f_i) = std::nan("");
      }
      }
     }
   }

    /* ROS_INFO("[%s]: HERE E", ros::this_node::getName().c_str()); */
}

//}
/* publishStates //{ */

    void publishStates(){

      
      /* std::scoped_lock lock(filter_mutex); */
      mrs_msgs::PoseWithCovarianceArrayStamped msg;
      mrs_msgs::PoseWithCovarianceArrayStamped msg_tent;
      msg.header.frame_id = _output_frame_;
      msg.header.stamp = ros::Time::now();
      msg_tent.header = msg.header;

      /* auto tfr = transformer_.getTransform("fcu_untilted", "stable_origin", msg.header.stamp); */
      /* if (!tfr) { */ 
      /*   ROS_ERROR("[UvdarKalmanAnonymous]: Could not obtain transform to fcu_untilted"); */
      /*   return; */
      /* } */
      mrs_msgs::PoseWithCovarianceIdentified temp;
      e::Quaterniond qtemp;
      for (auto const& fd_curr : fd | indexed(0)){

        temp.id = fd_curr.value().id;


        temp.pose.position.x = fd_curr.value().td.state_x.x[0];
        temp.pose.position.y = fd_curr.value().td.state_x.x[1];
        temp.pose.position.z = fd_curr.value().td.state_x.x[2];

        qtemp = e::AngleAxisd(fd_curr.value().td.state_x.x[3], e::Vector3d::UnitX()) * e::AngleAxisd(fd_curr.value().td.state_x.x[4], e::Vector3d::UnitY()) * e::AngleAxisd(fd_curr.value().td.state_x.x[5], e::Vector3d::UnitZ());

        temp.pose.orientation.x = qtemp.x();
        temp.pose.orientation.y = qtemp.y();
        temp.pose.orientation.z = qtemp.z();
        temp.pose.orientation.w = qtemp.w();

        for (int m=0; m<6; m++){
          for (int n=0; n<6; n++){
            temp.covariance[6*n+m] =  fd_curr.value().td.state_x.P(n,m);
          }
        }
        if (fd_curr.value().update_count < minMeasurementsToValidation)
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
//
/* removeOverlaps //{ */

void removeOverlaps(){
  double curr_match_level;
  for (int i=0; i<((int)(fd.size())-1);i++){
    bool remove_first = false;
    for (int j=i+1; j<(int)(fd.size()); j++){
       curr_match_level = gaussJointMaxVal(
          fd[i].td.state_x.P.topLeftCorner(3,3),
          fd[j].td.state_x.P.topLeftCorner(3,3),
          fd[i].td.state_x.x.topRows(3),
          fd[j].td.state_x.x.topRows(3)
          );
      if (curr_match_level > MATCH_LEVEL_THRESHOLD_REMOVE){
          auto eigens = fd[i].td.state_x.P.topLeftCorner(3,3).eigenvalues();
          double size_i = (eigens.topLeftCorner(3, 1)).norm();
          eigens = fd[j].td.state_x.P.topLeftCorner(3,3).eigenvalues();
          double size_j = (eigens.topLeftCorner(3, 1)).norm();
          int n = -1;
          int m = -1;
          if ( (fd[i].update_count < minMeasurementsToValidation) == (fd[j].update_count < minMeasurementsToValidation) ){
              if ( size_j > size_i ){
              n = j;
              m = i;
              }
              else {
              n = i;
              m = j;
              }
            }
          else {
            if (fd[i].update_count < minMeasurementsToValidation){
              n = i;
              m = j;
            }
            else {
              n = j;
              m = i;
            }

          }
            fd.erase(fd.begin()+n);
            ROS_INFO_STREAM("[UvdarKalmanAnonymous]: Removing state " << n << ": " << fd[n].td.state_x.x.transpose() << " due to large overlap with state " << m << ": " << fd[m].td.state_x.x.transpose());
            if (n<m){
              remove_first=true;
              break;
            }
            else {
              j--;
            }
      }
    }
    if (remove_first){
      i--;
    }

  }
}

//}

/* quatRotateRoll //{ */

double quatToRoll(e::Quaterniond q){
  e::Matrix3d m = q.matrix();
  return atan2(m(2,1),m(2,2));
}

//}

/* quatRotatePitch //{ */

double quatToPitch(e::Quaterniond q){
  e::Matrix3d m = q.matrix();
  return atan2( -m(2,0), sqrt( m(2,1)*m(2,1) +m(2,2)*m(2,2) )  );
}

//}

/* quatRotateYaw //{ */

double quatToYaw(e::Quaterniond q){
  e::Matrix3d m = q.matrix();
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

/* mahalanobis_distance2 //{ */

static double mahalanobis_distance2(const Eigen::Vector3d& x, const Eigen::Vector3d& mu1, const Eigen::Matrix3d& sigma1)
{
  const auto diff = x - mu1;
  const double dist2 = diff.transpose() * sigma1.inverse() * diff;
  return fabs(dist2);
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
