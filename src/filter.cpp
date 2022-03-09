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
#include <std_msgs/String.h>

#include <boost/range/adaptor/indexed.hpp> 

#include <mutex>

#include <Eigen/Dense>

#define sqr(X) ((X) * (X))

#define DEFAULT_OUTPUT_FRAMERATE 20.0
#define DECAY_AGE_NORMAL 3.0
#define DECAY_AGE_UNVALIDATED 1.0
#define MIN_MEASUREMENTS_TO_VALIDATION 10
#define POS_THRESH 2.0
#define MAH_THRESH 2.0
#define YAW_THRESH 1.5
#define MATCH_LEVEL_THRESHOLD_ASSOCIATE 0.3
#define MATCH_LEVEL_THRESHOLD_REMOVE 0.5

using namespace boost::adaptors;

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
using u_t = mrs_lib::lkf_t::u_t;
using statecov_t = mrs_lib::lkf_t::statecov_t;

namespace e = Eigen;

namespace uvdar {


  /**
   * @brief A processing lass for filtering measurements from UVDAR-based relative UAV pose estimator. This uses a vector of Kalman filter states for tracking multiple targets either if they have known identity or if they are anonymous.
   */
  class UVDARKalman {

    private:
      /* attributes //{ */

      bool _debug_ = false;

      bool initialized_ = false;
      std::string _uav_name_;

      std::string _output_frame_;
      int _input_count_;
      double _output_framerate_;
      bool _odometry_available_, _indoor_;
      double vl, vv, sn;
      bool _anonymous_measurements_;
      bool _use_velocity_;

      std::mutex meas_mutex;
      std::mutex filter_mutex;
      std::mutex transformer_mutex;

      std::shared_ptr<mrs_lib::lkf_t> filter;

      struct td_t{
        A_t A;
        B_t B;
        H_t H;
        Q_t Q;
      };
      td_t filter_matrices;

      struct FilterData{
        statecov_t filter_state;
        int update_count;
        ros::Time latest_update;
        ros::Time latest_measurement;
        unsigned long long int id;
      };

      std::vector<FilterData> fd;
      // | ----------------------- subscribers ---------------------- |

      std::vector<ros::Subscriber> sub_measurements_;

      // | ----------------------- publishers ---------------------- |
      ros::Publisher pub_filter_;
      ros::Publisher pub_filter_tent_;

      ros::Publisher pub_status_;

      ros::Timer timer;
      ros::Duration filter_update_period;
      double dt;
      unsigned long long int latest_id = 0;

      std::unique_ptr<mrs_lib::Transformer> transformer_;

      //}

    public:
      /**
       * @brief Constructor - loads parameters and initializes necessary structures
       *
       * @param nh Private NodeHandle of this ROS node
       */
      /* Constructor //{ */
      UVDARKalman(ros::NodeHandle nh) {
        ros::Time::waitForValid();

        mrs_lib::ParamLoader param_loader(nh, "UVDARKalman");

        param_loader.loadParam("debug", _debug_);

        param_loader.loadParam("uav_name", _uav_name_);
        param_loader.loadParam("output_frame", _output_frame_, std::string("local_origin"));
        param_loader.loadParam("output_framerate", _output_framerate_, double(DEFAULT_OUTPUT_FRAMERATE));

        param_loader.loadParam("anonymous_measurements", _anonymous_measurements_, bool(false));

        param_loader.loadParam("indoor", _indoor_, bool(false));
        param_loader.loadParam("odometry_available", _odometry_available_, bool(true));
        param_loader.loadParam("use_velocity", _use_velocity_, bool(false));

        if (_indoor_){ //lateral and vertical velocities are more limited in indoor conditions
          vl = 1;
          vv = 0.5;
        }
        else {
          vl = 2;
          vv = 1;
        }

        if (_odometry_available_){ //process noise is greater, since without odometry we can't correct for the ego-motion of the observer
          sn = 2;
        }
        else {
          sn = 4;
        }

        filter_update_period = ros::Duration(1.0 / fmax(_output_framerate_,1.0));
        dt = filter_update_period.toSec();
        timer = nh.createTimer(filter_update_period, &UVDARKalman::spin, this);

        transformer_ = std::make_unique<mrs_lib::Transformer>("UVDARKalman");
        transformer_->setDefaultPrefix(_uav_name_);

        std::vector<std::string> _measured_poses_topics;
        param_loader.loadParam("measured_poses_topics", _measured_poses_topics, _measured_poses_topics);
        if (_measured_poses_topics.empty()) {
          ROS_WARN("[UVDARKalman]: No topics of measured_poses_topics were supplied. Returning.");
          return;
        }

        for (auto& topic : _measured_poses_topics) {
          ROS_INFO_STREAM("[UVDARKalman]: Subscribing to " << topic);
          sub_measurements_.push_back(nh.subscribe(topic, 3, &UVDARKalman::callbackMeasurement, this, ros::TransportHints().tcpNoDelay())); 
        }

        pub_filter_ = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("filtered_poses", 1);
        pub_filter_tent_ = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("filtered_poses/tentative", 1);

        pub_status_ = nh.advertise<std_msgs::String>("/"+_uav_name_+"/mrs_uav_status/display_string", 1);

        if (_anonymous_measurements_ && _use_velocity_){
          ROS_WARN("[UVDARKalman]: Velocity estimation for anonymous measurements is not implemented. Returning.");
          return;
        }

        if (_anonymous_measurements_){
          filter_matrices.A.resize(6,6);
          filter_matrices.Q.resize(6,6);
          filter_matrices.H.resize(6,6);

          filter_matrices.H <<
            1,0,0,0,0,0,
            0,1,0,0,0,0,
            0,0,1,0,0,0,
            0,0,0,1,0,0,
            0,0,0,0,1,0,
            0,0,0,0,0,1;

        }
        else {
          if (_use_velocity_){
            filter_matrices.A.resize(9,9);
            filter_matrices.Q.resize(9,9);
            filter_matrices.H.resize(6,9);

            filter_matrices.H <<
              1,0,0,0,0,0,0,0,0,
              0,1,0,0,0,0,0,0,0,
              0,0,1,0,0,0,0,0,0,
              0,0,0,0,0,0,1,0,0,
              0,0,0,0,0,0,0,1,0,
              0,0,0,0,0,0,0,0,1;
          }
          else {
            filter_matrices.A.resize(6,6);
            filter_matrices.Q.resize(6,6);
            filter_matrices.H.resize(6,6);

            filter_matrices.H <<
              1,0,0,0,0,0,
              0,1,0,0,0,0,
              0,0,1,0,0,0,
              0,0,0,1,0,0,
              0,0,0,0,1,0,
              0,0,0,0,0,1;
          }
        }

        filter_matrices.B = B_t();


        filter = std::make_unique<mrs_lib::lkf_t>(filter_matrices.A, filter_matrices.B, filter_matrices.H);


        if (param_loader.loadedSuccessfully()) {
          initialized_ = true;
          ROS_INFO_STREAM("[UVDARKalman]: initiated");
        } else {
          ROS_ERROR_ONCE("[UVDARKalman]: PARAMS not loaded correctly, shutting down.");
          nh.shutdown();
        }


      }
      //}

    private:

      /**
       * @brief Callback to process input messages - array of pose estimates with covariance and identity
       *
       * @param msg
       */
      /* calllbackMeasurement //{ */
      void callbackMeasurement(const mrs_msgs::PoseWithCovarianceArrayStamped& msg){
        if ((int)(msg.poses.size()) < 1)
          return;
        if (_debug_)
          ROS_INFO_STREAM("[UVDARKalman]: Getting " << (int)(msg.poses.size()) << " measurements...");


        mrs_msgs::PoseWithCovarianceArrayStamped msg_local;
        {
          std::scoped_lock lock(meas_mutex);
          msg_local = msg;
        }

        std::optional<geometry_msgs::TransformStamped> tf;
        {
          std::scoped_lock lock(transformer_mutex);
          tf = transformer_->getTransform(msg_local.header.frame_id, _output_frame_, msg_local.header.stamp);
        }
        if (!tf) { 
          ROS_ERROR("[UVDARKalman]: Could not obtain transform from %s to %s",msg_local.header.frame_id.c_str(), _output_frame_.c_str());
          return;
        }
        std::vector<statecov_t> meas_converted;
        std::vector<int> ids;
        for (auto &meas : msg_local.poses){
          e::VectorXd poseVec(6);
          e::MatrixXd poseCov(6,6);

          std::optional<geometry_msgs::PoseWithCovarianceStamped> meas_t;
          geometry_msgs::PoseWithCovarianceStamped meas_s;
          meas_s.pose.pose = meas.pose;
          meas_s.pose.covariance = meas.covariance;
          if (_debug_)
            ROS_INFO_STREAM("[UVDARKalman]: Meas. cov. input: " << std::endl << rosCovarianceToEigen(meas.covariance));
          meas_s.header = msg_local.header;
          {
            std::scoped_lock lock(transformer_mutex);
            meas_t = transformer_->transform(meas_s, tf.value());
          }
          if (!meas_t){
            ROS_INFO_STREAM("[UVDARKalman]: Failed to get transformation for measurement, returning.");
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

          poseVec(3) = fixAngle(quatToRoll(qtemp), 0);
          poseVec(4) = fixAngle(quatToPitch(qtemp), 0);
          poseVec(5) = fixAngle(quatToYaw(qtemp), 0);


          if (poseVec.array().isNaN().any()){
            ROS_INFO_STREAM("[UVDARKalman]: Discarding input with ID" << meas.id << ", value includes Nans.");
            return;
          }

          /*           for (int i=0; i<6; i++){ */
          /*             for (int j=0; j<6; j++){ */
          /*               poseCov(j,i) =  [6*j+i]; */
          /*             } */
          /*           } */

          poseCov = rosCovarianceToEigen(meas_t.value().pose.covariance);

          if (poseCov.array().isNaN().any()){
            ROS_INFO("[UVDARKalman]: Discarding input, covariance includes Nans.");
            return;
          }

          bool changed = false;

          e::EigenSolver<e::Matrix3d> es(poseCov.bottomRightCorner(3,3));
          auto eigvals = es.eigenvalues();
          for (int i=0; i<(int)(eigvals.size()); i++){
            /* ROS_INFO_STREAM("[UVDARKalman]: Eigenvalues: "<< eigvals(i).real() << " im: " <<  eigvals(i).imag()); */
            if (eigvals(i).real() > sqr(M_PI/3.0)){ //angle std.dev representing no knowledge (3*sigma contains 99.7 percent)
              eigvals(i) = sqr(666.0); //arbitrarily large number, s.t. the measurement will not affect the state
              changed = true;
            }
          }

          if (changed){
            auto eigvecs = es.eigenvectors();
            /* ROS_INFO_STREAM("[UVDARKalman]: Eigenvectors: "<< std::endl << eigvecs.real() << std::endl << "im: " << std::endl <<  eigvecs.imag()); */
            poseCov.bottomRightCorner(3,3) = eigvecs.real()*eigvals.real().asDiagonal()*eigvecs.real().transpose(); //reform the covariance with new expanded eigenvalues
          }

          if (_debug_){
            ROS_INFO_STREAM("[UVDARKalman]: Transformed measurement input is: [" << poseVec.transpose() << "]");
            ROS_INFO_STREAM("[UVDARKalman]: Meas. cov. transformed: " << std::endl << poseCov);
          }

          /* poseCov.topLeftCorner(3,3) += e::Matrix3d::Identity()*0.5; //fattening the measurements before applying them - they don't represent the distribution too well and the elliptic shape of the gaussians unduly shortents the filter covariances, as if we had more information on distance than we really do. */

          meas_converted.push_back({.x=poseVec,.P=poseCov});
          ids.push_back(meas.id);
        }


        if (_anonymous_measurements_){
          applyMeasurementsAnonymous(meas_converted, msg_local.header.stamp, msg_local.header.frame_id);
        }
        else {
          applyMeasurementsWithIdentity(meas_converted, ids, msg_local.header.stamp, msg_local.header.frame_id);
        }
      }
      //}

      /**
       * @brief Thread for processing, predicting and outputting filter states at a regular rate
       *
       * @param te TimerEvent for the timer spinning this thread
       */
      /* spin //{ */
      void spin([[ maybe_unused ]] const ros::TimerEvent& te){
        std::scoped_lock lock(filter_mutex);
        removeNANs();
        if (_anonymous_measurements_){
          removeOverlaps();
        }
        for (int target=0; target<(int)(fd.size());target++){
          int targetsSeen = 0;
          double age = (ros::Time::now() - fd[target].latest_measurement).toSec();
          if (_debug_)
            ROS_INFO("[UVDARKalman]: Age of %d is %f", target, age);
          double decay_age;
          if (fd[target].update_count > MIN_MEASUREMENTS_TO_VALIDATION){
            decay_age = DECAY_AGE_NORMAL;
          }
          else {
            decay_age = DECAY_AGE_UNVALIDATED;
          }
          if (age>decay_age){
            ROS_INFO_STREAM("[UVDARKalman]: Removing state " << target << " (ID:" << fd[target].id << ") : " << fd[target].filter_state.x.transpose() << " due to old age of " << age << " s." );
            fd.erase(fd.begin()+target);
            target--;
            continue;
          }
          predictTillTime(fd.at(target), ros::Time::now(), true);

          if (_debug_){
            ROS_INFO_STREAM("[UVDARKalman]: State: ");
            ROS_INFO_STREAM("[UVDARKalman]: \n" << fd[target].filter_state.x);
          }


            /* if ( */
            /*     ((fd[target].filter_state.P.diagonal().array() < 0).any()) */
            /*    ){ */
            /*   ROS_INFO_STREAM("[UVDARKalman]: SPIN: NEGATIVE NUMBERS ON MAIN DIAGONAL!"); */
            /*   ROS_INFO_STREAM("[UVDARKalman]: State. cov: " << std::endl << fd[target].filter_state.P); */
            /* } */
        }

        publishStates();

      }
      //}

      /**
       * @brief Initiates a new filter based on a measurement
       *
       * @param x Measurement vector
       * @param C Error covariance associated with x
       * @param stamp Time of the measurement
       * @param id Identity of the target (-1 means that it is unknown or irrelevant)
       */
      /* initiateNew //{ */
      void initiateNew(e::VectorXd x, e::MatrixXd C, ros::Time stamp, int id = -1){
        if (fd.size() > 20)
          return;

        bool changed = false;
        auto eigens = C.topLeftCorner(3,3).eigenvalues();
        for (int i=0; i<3; i++){
          if (eigens(i).real() > (x.topLeftCorner(3,1).norm())){
            eigens(i) = 5.0;
            changed = true;
          }
        }

        if (changed){
          if (id == -1){
            e::EigenSolver<e::Matrix3d> es(C.topLeftCorner(3,3));
            C.topLeftCorner(3,3) = es.eigenvectors().real()*eigens.real().asDiagonal()*es.eigenvectors().real().transpose();
            x.topLeftCorner(3,1) = x.topLeftCorner(3,1).normalized()*15;
            //so that we don't initialize with the long covariances intersecting in the origin
          }
        }

          auto C_local = C; // the correlation between angle and position only exists in the measurement - the filter may have many measurements where this relation does not exist anymore
        if (!_use_velocity_){
          int index = (int)(fd.size());
          ROS_INFO_STREAM("[UVDARKalman]: Initiating state " << index << "(ID:" << ((id<0)?(latest_id++):(id)) << ")  with: " << x.transpose());
          ROS_INFO_STREAM("[UVDARKalman]: The source input of the state is " << (ros::Time::now() - stamp).toSec() << "s old.");

          C_local.topRightCorner(3,3).setZero();  // check the case of _use_velocity_ == true
          C_local.bottomLeftCorner(3,3).setZero();  // check the case of _use_velocity_ == true

          fd.push_back({.filter_state = {.x = x, .P = C_local}, .update_count = 0, .latest_update = stamp, .latest_measurement = stamp, .id = ((id<0)?(latest_id++):(id))});
        }
        else{
          ROS_WARN("[UVDARKalman]: Initialization of states with velocity is not implemented. Returning.");
          return;
        }

        /* if ( */
        /*     ((fd.back().filter_state.P.diagonal().array() < 0).any()) */
        /*    ){ */
        /*   ROS_INFO_STREAM("[UVDARKalman]: INIT: NEGATIVE NUMBERS ON MAIN DIAGONAL!"); */
        /*   ROS_INFO_STREAM("[UVDARKalman]: Generating measurement: " << std::endl << C); */
        /*   ROS_INFO_STREAM("[UVDARKalman]: Generating fixed measurement: " << std::endl << C_local); */
        /*   ROS_INFO_STREAM("[UVDARKalman]: State. cov: " << std::endl << fd.back().filter_state.P); */
        /* } */
      }
      //}

      /**
       * @brief Predicts the filter state at target_time, given that target_time is newer than the last update or measurement
       *
       * @param fd_curr The filter data structure to start from (if apply_update is true, this structure will be updated with the result)
       * @param target_time The time to which the state is to be predicted
       * @param apply_update If true, the output will be applied to the structure fd_curr. This provides more data than the return value, such as the update time
       *
       * @return The resulting state and its output covariance
       */
      /* predictTillTime //{ */
      statecov_t predictTillTime(struct FilterData &fd_curr, ros::Time target_time, bool apply_update = false){
        auto orig_state = fd_curr;
        double dt_from_last = std::fmax(std::fmin((target_time-fd_curr.latest_update).toSec(),(target_time-fd_curr.latest_measurement).toSec()),0.0);
        filter->A = A_dt(dt_from_last);
        /* ROS_INFO_STREAM("[UVDARKalman]: Filter pred. orig: " << std::endl << fd_curr.filter_state.P); */
        auto new_state =  filter->predict(fd_curr.filter_state, u_t(), Q_dt(dt_from_last), dt_from_last);
        /* ROS_INFO_STREAM("[UVDARKalman]: Filter predicted: " << std::endl << fd_curr.filter_state.P); */
        if (apply_update){
          fd_curr.filter_state = new_state;
          fd_curr.latest_update = target_time;
        }

            if (
                ((orig_state.filter_state.P.diagonal().array() < 0).any()) ||
                ((fd_curr.filter_state.P.diagonal().array() < 0).any())
               ){
              ROS_INFO_STREAM("[UVDARKalman]: NEGATIVE NUMBERS ON MAIN DIAGONAL!");
              ROS_INFO_STREAM("[UVDARKalman]: Orig. state. cov: " << std::endl << orig_state.filter_state.P);
              /* ROS_INFO_STREAM("[UVDARKalman]: dt: " << dt_from_last); */
              /* ROS_INFO_STREAM("[UVDARKalman]: Q_dt: " << Q_dt(dt_from_last)); */
              ROS_INFO_STREAM("[UVDARKalman]: New state. cov: " << std::endl << fd_curr.filter_state.P);
            }
        return new_state;
      }
      //}

      /**
       * @brief Predicts the change of the state and error covariance till meas_time, and then corrects the state with measurement
       *
       * @param fd_curr The filter data structure to start from (if apply_update is true, this structure will be updated with the result)
       * @param measurement The input measurement to correct the state with
       * @param match_level The output level of overlap between the original state fd_curr and the measurement in terms of position. This is based on the maximum of multivariate Gaussian multiplication between the measurement and the state, and represents information that is normally lost in the Kalman filter process
       * @param meas_time The time of the input measurement
       * @param prior_predict if true, the state is first brought to meas_time by predicting its change from the last update or measurement
       * @param apply_update If true, the output will be applied to the structure fd_curr. This provides more data than the return value, such as the update time
       *
       * @return The resulting state and its output covariance
       */
      /* correctWithMeasurement //{ */
      statecov_t correctWithMeasurement(struct FilterData &fd_curr, statecov_t measurement, double &match_level_pos, ros::Time meas_time, bool prior_predict, bool apply_update = false, std::string camera_frame = ""){
        auto filter_local = fd_curr;

        auto orig_state = filter_local.filter_state;
        /* ROS_INFO_STREAM("[UVDARKalman]: Original state: " << fd_curr.id << " was " << filter_local.filter_state.x.transpose()); */
        /* ROS_INFO_STREAM("[UVDARKalman]: Original cov: " << std::endl << filter_local.filter_state.P); */

        if (prior_predict){
          predictTillTime(filter_local, meas_time, true);
        }


        /* ROS_INFO_STREAM("[UVDARKalman]: Orig. angles: " << filter_local.filter_state.x.bottomRows(3)); */
        filter_local.filter_state.x[3] = fixAngle(filter_local.filter_state.x[3], measurement.x[3]);
        filter_local.filter_state.x[4] = fixAngle(filter_local.filter_state.x[4], measurement.x[4]);
        filter_local.filter_state.x[5] = fixAngle(filter_local.filter_state.x[5], measurement.x[5]);
        /* ROS_INFO_STREAM("[UVDARKalman]: Fixed. angles: " << filter_local.filter_state.x.bottomRows(3)); */

        match_level_pos = gaussJointMaxVal(
            measurement.P.topLeftCorner(3,3),
            filter_local.filter_state.P.topLeftCorner(3,3),
            measurement.x.topRows(3),
            filter_local.filter_state.x.topRows(3)
            );
        double match_level_rot = gaussJointMaxVal(
            measurement.P.bottomRightCorner(3,3),
            filter_local.filter_state.P.bottomRightCorner(3,3),
            measurement.x.bottomRows(3),
            filter_local.filter_state.x.bottomRows(3)
            );

        auto P_local = measurement.P;
        /* ROS_INFO_STREAM("[UVDARKalman]: Meas. cov: " << std::endl << P_local); */
        P_local.topRightCorner(3,3).setZero();  // check the case of _use_velocity_ == true
        P_local.bottomLeftCorner(3,3).setZero();  // check the case of _use_velocity_ == true

        /* double dt_from_last = std::fmax(std::fmin((filter_local.latest_update-meas_time).toSec(),(filter_local.latest_measurement-meas_time).toSec()),0.0); */
        /* P_local += Q_dt(dt_from_last)*dt_from_last; */

        /* ROS_INFO_STREAM("[UVDARKalman]: Scaling by: " << 1.0/match_level << " due to match level of " << match_level); */

        double volume_ratio_pos = std::min(1.0,filter_local.filter_state.P.topLeftCorner(3,3).determinant()/P_local.topLeftCorner(3,3).determinant());

        P_local.topLeftCorner(3,3) *= (1.0/(match_level_pos*volume_ratio_pos));
        /* P_local.bottomRightCorner(3,3) *= (1.0/match_level_rot); */
        /* ROS_INFO_STREAM("[UVDARKalman]: With ML: " << std::endl << P_local); */
        /* P_local *= std::max(0.1,(1.0/match_level)); */


        /* ROS_INFO_STREAM("[UVDARKalman]: Filter orig: " << std::endl << filter_local.filter_state.P); */
        try {
          filter_local.filter_state = filter->correct(filter_local.filter_state, measurement.x, P_local);
        }
        catch ([[maybe_unused]] std::exception e) {
          ROS_ERROR_STREAM("[UVDARKalman]: Attempted to correct with bad covariance (match_level_pos = " << match_level_pos << "). Will replace with big, but manabeable one.");
          /* P_local = e::MatrixXd(6,6); */
          P_local.setIdentity();
          P_local *= 10000;
          filter_local.filter_state = filter->correct(filter_local.filter_state, measurement.x, P_local);
        }
        /* ROS_INFO_STREAM("[UVDARKalman]: Filter corr: " << std::endl << filter_local.filter_state.P); */
        /* ROS_INFO_STREAM("[UVDARKalman]: Fixed to: " << std::endl << P_local); */

        /* filter_local.filter_state.P *= (1+match_level);//this makes the filter not increase certainty in case of multiple identical measurements - the mean in the covariances is more probable than the rest of its x<1*sigma space */

        auto eigens_pos = filter_local.filter_state.P.topLeftCorner(3,3).eigenvalues();
        double min_eig_pos = eigens_pos.real().minCoeff();
        auto eigens_rot = filter_local.filter_state.P.bottomRightCorner(3,3).eigenvalues();
        double min_eig_rot = eigens_rot.real().minCoeff();
        filter_local.filter_state.P.topLeftCorner(3,3) += e::MatrixXd::Identity(3,3)*(min_eig_pos*(match_level_pos));//this makes the filter not increase certainty in case of multiple identical measurements - the mean in the covariances is more probable than the rest of its x<1*sigma space
        /* filter_local.filter_state.P.bottomRightCorner(3,3) += e::MatrixXd::Identity(3,3)*(min_eig_rot*(match_level_rot));//this makes the filter not increase certainty in case of multiple identical measurements - the mean in the covariances is more probable than the rest of its x<1*sigma space */
        /* ROS_INFO_STREAM("[UVDARKalman]: Filter exp.: " << std::endl << filter_local.filter_state.P); */

        filter_local.filter_state.x[3] = fixAngle(filter_local.filter_state.x[3], 0);
        filter_local.filter_state.x[4] = fixAngle(filter_local.filter_state.x[4], 0);
        filter_local.filter_state.x[5] = fixAngle(filter_local.filter_state.x[5], 0);

        if (isInFrontOfCamera(filter_local.filter_state.x.topRows(3), camera_frame, meas_time)){
          if (apply_update){
            /* if (_debug_){ */
            /* if ( */
            /*     ((orig_state.x.topRows(3) - filter_local.filter_state.x.topRows(3)).norm() > 2.0) || */
            /*     ((orig_state.x.bottomRows(3) - filter_local.filter_state.x.bottomRows(3)).norm() > 0.2) */
            /*    ){ */
            /*   ROS_INFO_STREAM("[UVDARKalman]: Updating state: " << fd_curr.id << " with " << measurement.x.transpose()); */
            /*   ROS_INFO_STREAM("[UVDARKalman]: Meas. cov: " << std::endl << measurement.P); */
            /*   ROS_INFO_STREAM("[UVDARKalman]: Fixed to: " << std::endl << P_local); */
            /*   ROS_INFO_STREAM("[UVDARKalman]: State. cov: " << std::endl << orig_state.P); */
            /*   ROS_INFO_STREAM("[UVDARKalman]: This yelds state: " << filter_local.filter_state.x.topRows(6).transpose()); */
            /* } */
            /* if ( */
            /*     ((orig_state.P.diagonal().array() < 0).any()) || */
            /*     ((measurement.P.diagonal().array() < 0).any()) || */ 
            /*     ((filter_local.filter_state.P.diagonal().array() < 0).any()) */
            /*    ){ */
            /*   ROS_INFO_STREAM("[UVDARKalman]: NEGATIVE NUMBERS ON MAIN DIAGONAL!"); */
            /*   ROS_INFO_STREAM("[UVDARKalman]: Meas. cov: " << std::endl << measurement.P); */
            /*   ROS_INFO_STREAM("[UVDARKalman]: Fixed to: " << std::endl << P_local); */
            /*   ROS_INFO_STREAM("[UVDARKalman]: Orig. state. cov: " << std::endl << orig_state.P); */
            /*   ROS_INFO_STREAM("[UVDARKalman]: New state. cov: " << std::endl << filter_local.filter_state.P); */
            /* } */
            /* } */
            fd_curr.filter_state = filter_local.filter_state;
            fd_curr.latest_measurement = meas_time;
            fd_curr.update_count++;
          }
        }
        return filter_local.filter_state;
      }
      //}

      /**
       * @brief Deletes on of the active filter states
       *
       * @param index The index of the filter state to remove
       */
      /* deleteFilter //{ */
      void deleteFilter(int &index){
        if (index < 0) return;
        fd.erase(fd.begin()+index);
        index--;
      }
      //}

      /**
       * @brief Returns a value between 0 and 1 representing the level of overlap between two probability distributions in the form of multivariate Gaussians. Multiplying two Gaussians produces another Gaussian, with the value of its peak roughly corresponding to the level of the overlap between the two inputs. The ouptut of this function is the value of such peak, given that the input Gaussians have been scaled s.t. their peaks have the value of 1. Therefore, the output is 1 if the means of both inputs are identical.
       *
       * @param si0 The covariance of the first distribution
       * @param si1 The covariance of the second distribution
       * @param mu0 The mean of the first distribution
       * @param mu1 The mean of the second distribution
       *
       * @return The level of overlap between the two distribution
       */
      /* gaussJointMaxVal //{ */
      double gaussJointMaxVal(e::MatrixXd si0,e::MatrixXd si1,e::VectorXd mu0,e::VectorXd mu1){
        bool scaled = true;
        int k=mu0.size();
        auto K=si0*(si0+si1).inverse();
        auto d0=K*(mu1-mu0);
        auto d1=(K-e::MatrixXd::Identity(K.rows(),K.rows()))*(mu1-mu0);
        double N;
        if (scaled){
          auto N_v = ((-0.5)*( (d0.transpose()*(si0).inverse()*d0) + (d1.transpose()*(si1).inverse()*d1) ));
          N=exp(N_v(0));
        }
        else{
          auto N_v = ((-0.5)*( (d0.transpose()*(si0).inverse()*d0) + (d1.transpose()*(si1).inverse()*d1) ));
          N=(1.0/pow((2*M_PI),k)*sqrt((si0).determinant()*(si1).determinant()))*exp(N_v(0));
        }
        if (isnan(N))
          ROS_INFO("[UVDARKalman]: Joint Gaussian value came out NaN");

        return N;
      }
      //}

      bool isInFrontOfCamera(e::Vector3d mean, std::string camera_frame, ros::Time stamp){
        geometry_msgs::PoseStamped target_cam_view, target_filter;
        target_filter.header.frame_id = _output_frame_;
        target_filter.header.stamp = stamp;
        target_filter.pose.position.x = mean.x();
        target_filter.pose.position.y = mean.y();
        target_filter.pose.position.z = mean.z();

        auto ret = transformer_->transformSingle(target_filter, camera_frame);
        if (ret) {
          target_cam_view = ret.value();
        }
        else{
          ROS_ERROR_STREAM("[UVDARKalman]: Could not transform filter state from "<< _output_frame_ << " to camera frame " << camera_frame);
          return false;
        }

        e::Vector3d target_cam_view_vector(target_cam_view.pose.position.x, target_cam_view.pose.position.y, target_cam_view.pose.position.z);
        double norm = target_cam_view_vector.norm();
        double cos_angle = target_cam_view_vector.normalized().dot(e::Vector3d(0,0,1));

        return ((norm > 1.5) && (cos_angle > -0.173648));//cos(100 deg) 

      }

      /**
       * @brief Applies relative position measurements to states that are the closest to their position component
       *
       * @param measurements A vector of measurements with error covariances
       * @param meas_time The time of the input measurements
       */
      /* applyMeasurementsAnonymous //{ */
      void applyMeasurementsAnonymous(std::vector<statecov_t> measurements, ros::Time meas_time, std::string camera_frame){
        std::scoped_lock lock(filter_mutex);
        if (fd.size() == 0){
          for (auto const& measurement_curr : measurements | indexed(0)){
            initiateNew(measurement_curr.value().x, measurement_curr.value().P, meas_time);
          }
          return;
        }

        e::MatrixXd match_matrix(measurements.size(),fd.size());
        std::vector<std::vector<FilterData>> tentative_states;

        for (auto const& measurement_curr : measurements | indexed(0)){
          tentative_states.push_back(std::vector<FilterData>());
          for (auto const& state_curr : fd | indexed(0)){

            double dt_from_last = std::fmin((meas_time-state_curr.value().latest_update).toSec(),(meas_time-state_curr.value().latest_measurement).toSec());
            

            tentative_states.back().push_back(
                state_curr.value()
                );
            double match_level;
            correctWithMeasurement(tentative_states.back().back(),measurement_curr.value(), match_level, meas_time, true, true, camera_frame);
            match_matrix(measurement_curr.index(),state_curr.index()) = match_level;
            double dt_s = 0.1;
            if ((ros::Time::now() - state_curr.value().latest_measurement).toSec() < dt_s){ // just in case - in simulation the camera outputs follow one another immediately, so no inflation happens in between
              tentative_states.back().back().filter_state = predictTillTime(tentative_states.back().back(), ros::Time::now()+ros::Duration(dt_s),false);
            }
          }
        }

        if (_debug_){
          ROS_INFO_STREAM("[UVDARKalman]: match_matrix: " << std::endl <<match_matrix);
        }

        std::vector<std::pair<int,int>> matches;

        int best_index;
        double best_match_level;
        double best_update_count;
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
            matches.push_back({best_index,f_i});
            for (int f_j = 0; f_j < (int)(fd.size()); f_j++) {
              match_matrix(best_index,f_j) = std::nan("");
            }
          }
          for (int m_j = 0; m_j < (int)(measurements.size()); m_j++) {
            if (match_matrix(m_j,f_i) > MATCH_LEVEL_THRESHOLD_ASSOCIATE){
              match_matrix(m_j,f_i) = std::nan("");
            }
          }
        }

        for( auto& match_curr : matches){
          if (_debug_){
            ROS_INFO_STREAM("[UVDARKalman]: Updating state: " << match_curr.second << " with " << measurements[match_curr.first].x.transpose());
            ROS_INFO_STREAM("[UVDARKalman]: This yelds state: " << tentative_states[match_curr.first][match_curr.second].filter_state.x.topRows(6).transpose());
          }
          fd[match_curr.second] = tentative_states[match_curr.first][match_curr.second];

        }

        if (_debug_){
          ROS_INFO_STREAM("[UVDARKalman]: match_matrix post: " << std::endl <<match_matrix);
        }
        int fd_size_orig = (int)(fd.size());
        for (int f_i = 0; f_i < fd_size_orig; f_i++) {
          for (int m_i = 0; m_i < (int)(measurements.size()); m_i++) {
            if (_debug_){
              ROS_INFO_STREAM("[UVDARKalman]: match_matrix at: [" << m_i << ":" << f_i << "] is: " << match_matrix(m_i,f_i));
            }
            if (!isnan(match_matrix(m_i,f_i))){
              initiateNew(measurements[m_i].x, measurements[m_i].P, meas_time);
              for (int f_j = 0; f_j < fd_size_orig; f_j++) {
                match_matrix(m_i,f_j) = std::nan("");
              }
              for (int m_j = 0; m_j < (int)(measurements.size()); m_j++) {
                match_matrix(m_j,f_i) = std::nan("");
              }
            }
          }
        }

      }
      //}

      /**
       * @brief Applies relative position measurements to states that match their IDs
       *
       * @param measurements A vector of measurements with error covariances
       * @param ids A vector of IDs associated with the measurements - deanonymize measurements of multiple targets
       * @param meas_time
       */
      /* applyMeasurementsWithIdentity //{ */
      void applyMeasurementsWithIdentity(std::vector<statecov_t> measurements, std::vector<int> ids, ros::Time meas_time, std::string camera_frame){
        std::scoped_lock lock(filter_mutex);

        if (measurements.size() != ids.size()){
            ROS_ERROR("[UVDARKalman]: the sizes of the input measurement vector and of the vector of their identities do not match! Returning.");
            return;
            }

        for(auto const& measurement_curr : measurements | indexed(0)){
          int id_local = ids[measurement_curr.index()] % 1000;
          int target = -1;
          for (auto const& filter_curr : fd | indexed(0)){
            if (id_local == (int)filter_curr.value().id){
              target = filter_curr.index();
            }
          }
          if (target < 0){
            initiateNew(measurement_curr.value().x, measurement_curr.value().P, meas_time, id_local);
          }
          else {
            [[ maybe_unused ]] double match_level; //for future use with multiple measurements with the same ID
            correctWithMeasurement(fd.at(target),measurement_curr.value(), match_level,meas_time,true, true, camera_frame);
          }

        }


      }
      //}

      /**
       * @brief Publishes the current set of filter states to an output topic
       */
      /* publishStates //{ */
      void publishStates(){
        mrs_msgs::PoseWithCovarianceArrayStamped msg;
        mrs_msgs::PoseWithCovarianceArrayStamped msg_tent;
        msg.header.frame_id = _output_frame_;
        msg.header.stamp = ros::Time::now();
        msg_tent.header = msg.header;

        mrs_msgs::PoseWithCovarianceIdentified temp;
        e::Quaterniond qtemp;
        for (auto const& fd_curr : fd | indexed(0)){
          temp.id = fd_curr.value().id;

          temp.pose.position.x = fd_curr.value().filter_state.x[0];
          temp.pose.position.y = fd_curr.value().filter_state.x[1];
          temp.pose.position.z = fd_curr.value().filter_state.x[2];

          qtemp = e::AngleAxisd(fd_curr.value().filter_state.x[3], e::Vector3d::UnitX()) * e::AngleAxisd(fd_curr.value().filter_state.x[4], e::Vector3d::UnitY()) * e::AngleAxisd(fd_curr.value().filter_state.x[5], e::Vector3d::UnitZ());

          temp.pose.orientation.x = qtemp.x();
          temp.pose.orientation.y = qtemp.y();
          temp.pose.orientation.z = qtemp.z();
          temp.pose.orientation.w = qtemp.w();

          /* for (int m=0; m<6; m++){ */
          /*   for (int n=0; n<6; n++){ */
          /*     temp.covariance[6*n+m] =  fd_curr.value().filter_state.P(n,m); */
          /*   } */
          /* } */
          temp.covariance = eigenCovarianceToRos(fd_curr.value().filter_state.P);

          if (fd_curr.value().update_count < MIN_MEASUREMENTS_TO_VALIDATION){
            msg_tent.poses.push_back(temp);
          }
          else{
            msg.poses.push_back(temp);
          }
        }

        std_msgs::String msg_status;
        msg_status.data = std::string("UVDAR sees "+std::to_string(msg.poses.size())+" targets").c_str();
        pub_status_.publish(msg_status);

        pub_filter_.publish(msg);
        pub_filter_tent_.publish(msg_tent);
      }
      //}

      /**
       * @brief Removes any filter state that contains NaNs.
       */
      /* removeNANs //{ */
      void removeNANs(){
        for (int i=0; i<(int)(fd.size());i++){
          if ( (fd[i].filter_state.x.array().isNaN().any() ) ||
              (fd[i].filter_state.P.array().isNaN().any() )){
            fd.erase(fd.begin()+i);
            i--;
          }
        }
      }
      //}


      /**
       * @brief Remove filter states that overlap too much with others in terms of their state vectors and associated covariances. Of each overlapping pair, the one removed is either the one that was not yet validated with sufficient number of measurements, or the one with larger covariance (less precise knowledge).
       */
      /* removeOverlaps //{ */
      void removeOverlaps(){
        double curr_match_level;
        for (int i=0; i<((int)(fd.size())-1);i++){
          bool remove_first = false;
          for (int j=i+1; j<(int)(fd.size()); j++){
            curr_match_level = gaussJointMaxVal(
                fd[i].filter_state.P.topLeftCorner(3,3),
                fd[j].filter_state.P.topLeftCorner(3,3),
                fd[i].filter_state.x.topRows(3),
                fd[j].filter_state.x.topRows(3)
                );
            if (curr_match_level > MATCH_LEVEL_THRESHOLD_REMOVE){
              auto eigens = fd[i].filter_state.P.topLeftCorner(3,3).eigenvalues();
              double size_i = (eigens.topLeftCorner(3, 1)).norm();
              eigens = fd[j].filter_state.P.topLeftCorner(3,3).eigenvalues();
              double size_j = (eigens.topLeftCorner(3, 1)).norm();
              int n = -1;
              int m = -1;
              if ( (fd[i].update_count < MIN_MEASUREMENTS_TO_VALIDATION) == (fd[j].update_count < MIN_MEASUREMENTS_TO_VALIDATION) ){
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
                if (fd[i].update_count < MIN_MEASUREMENTS_TO_VALIDATION){
                  n = i;
                  m = j;
                }
                else {
                  n = j;
                  m = i;
                }

              }
              fd.erase(fd.begin()+n);
              ROS_INFO_STREAM("[UVDARKalman]: Removing state " << n << " (ID:" << fd[n].id << "): " << fd[n].filter_state.x.transpose() << " due to large overlap with state " << m << ": " << fd[m].filter_state.x.transpose());
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

      /**
       * @brief Retrieves aviation Roll angle from a quaternion
       *
       * @param q The input quaternion
       *
       * @return The ouput angle
       */
      /* quatRotateRoll //{ */
      double quatToRoll(e::Quaterniond q){
        e::Matrix3d m = q.matrix();
        return atan2(m(2,1),m(2,2));
      }
      //}



      e::MatrixXd rosCovarianceToEigen(const boost::array<double,36> input){
        e::MatrixXd output(6,6);
        /* if ((int)(input.size()) != 36 ){ */
        /*   ROS_ERROR_STREAM("[UVDARKalman]: Covariance to be converted to Eigen matrix has " << input.size() << " elements instead of the expected 36! Returning"); */
        /*   return e::MatrixXd(); */
        /* } */

        for (int i=0; i<6; i++){
          for (int j=0; j<6; j++){
            output(j,i) =  input[6*j+i];
          }
        }

        return output;
      }

      boost::array<double,36> eigenCovarianceToRos(const e::MatrixXd input){
        boost::array<double, 36> output;
        if (((int)(input.rows()) != 6 ) || ((int)(input.cols()) != 6 )){
          ROS_ERROR_STREAM("[UVDARKalman]: Covariance to be converted to Ros has size of  " << input.rows() << "x" << input.cols() << " as opposed to the expected size of 6x6! Returning");
          /* return boost::array<double, 36>(std::nan("")); */
          double n = std::nan("");
          return {n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n};
        }

          for (int m=0; m<6; m++){
            for (int n=0; n<6; n++){
              output[6*n+m] = input(n,m);
            }
          }

        return output;
      }


      /**
       * @brief Retrieves aviation Pitch angle from a quaternion
       *
       * @param q The input quaternion
       *
       * @return The ouput angle
       */
      /* quatRotatePitch //{ */
      double quatToPitch(e::Quaterniond q){
        e::Matrix3d m = q.matrix();
        return atan2( -m(2,0), sqrt( m(2,1)*m(2,1) +m(2,2)*m(2,2) )  );
      }
      //}

      /**
       * @brief Retrieves aviation Yaw angle from a quaternion
       *
       * @param q The input quaternion
       *
       * @return The ouput angle
       */
      /* quatRotateYaw //{ */
      double quatToYaw(e::Quaterniond q){
        e::Matrix3d m = q.matrix();
        return atan2(m(1,0),m(0,0));
      }
      //}

      /**
       * @brief Changes the expression of the origAngle, such that if it is updated in a filtering process with newAngle, circularity issues will be avoided
       *
       * @param origAngle The angle, representing prior state, to be updated
       * @param newAngle The new angle, representing new measurement, that affects origAngle in filtering
       *
       * @return The new expression of origAngle
       */
      /* fixAngle //{ */
      double fixAngle(double origAngle, double newAngle){
        double fixedPre;
        if ( (origAngle>(2*M_PI)) || (origAngle<(-2*M_PI)) )  {
          fixedPre = fmod(origAngle,2*M_PI);
        }
        else {
          fixedPre = origAngle;
        }

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


      /**
       * @brief Updates the shared system matrix in case that it is time step-dependent. This is useful for non-uniform time-steps
       *
       * @param dt - The current time step of the kalman filter process
       *
       * @return The new system matrix
       */
      /* A_dt //{ */
      A_t A_dt(double dt){
        if (_anonymous_measurements_){
          filter_matrices.A <<
            1,0,0, 0,0,0,
            0,1,0, 0,0,0,
            0,0,1, 0,0,0,
            0,0,0, 1,0,0,
            0,0,0, 0,1,0,
            0,0,0, 0,0,1;
        } else {
          if (_use_velocity_){
            filter_matrices.A <<
              1,0,0,dt,0, 0, 0,0,0,
              0,1,0,0, dt,0, 0,0,0,
              0,0,1,0, 0, dt,0,0,0,
              0,0,0,1, 0, 0, 0,0,0,
              0,0,0,0, 1, 0, 0,0,0,
              0,0,0,0, 0, 1, 0,0,0,
              0,0,0,0, 0, 0, 1,0,0,
              0,0,0,0, 0, 0, 0,1,0,
              0,0,0,0, 0, 0, 0,0,1;
          }
          else{
            filter_matrices.A <<
              1,0,0, 0,0,0,
              0,1,0, 0,0,0,
              0,0,1, 0,0,0,
              0,0,0, 1,0,0,
              0,0,0, 0,1,0,
              0,0,0, 0,0,1;
          }
        }
        return filter_matrices.A;
      }
      //}

      /**
       * @brief Updates the shared process noise covariance matrix in case that it is time step-dependent. This is useful for non-uniform time-steps
       *
       * @param dt - The current time step of the kalman filter process
       *
       * @return The new process noise matrix
       */
      // Q_dt //{ */
      Q_t Q_dt(double dt){
        if (_anonymous_measurements_){
          //simplified to the process described in detail below. These two approaches will be compared
          filter_matrices.Q <<
            vl, 0 ,0, 0, 0 ,0,
            0, vl, 0, 0, 0 ,0,
            0, 0, vv, 0, 0 ,0,
            0, 0, 0,  1, 0 ,0,
            0, 0, 0,  0, 1 ,0,
            0, 0, 0,  0, 0 ,1;
        } else {
          if (_use_velocity_){
            filter_matrices.Q <<
              sn*sn, 0,     0,     0,  0,  0,  0, 0, 0,
              0,     sn*sn, 0,     0,  0,  0,  0, 0, 0,
              0,     0,     sn*sn, 0,  0,  0,  0, 0, 0,
              0,     0,     0,     vl, 0,  0,  0, 0, 0,
              0,     0,     0,     0,  vl, 0,  0, 0, 0,
              0,     0,     0,     0,  0,  vv, 0, 0, 0,
              0,     0,     0,     0,  0,  0,  1, 0, 0,
              0,     0,     0,     0,  0,  0,  0, 1, 0,
              0,     0,     0,     0,  0,  0,  0, 0, 1;
          }
          else{
            // This is an unorthodox approach.
            // I wanted for the process noise covariance Q (expressing a multivariate gaussian) to cover not only the noise in static position estimate, but to also take into account the mean of expected relative velocity, which is not a state variable in this case.
            // The noise should inflate the error covariance of the state approximately linearly. This can be demonstrated in the following example:
            //  In this filter, we can receive measurement with time-stamp after a previous step of prediction. We therefore first need to predict the effects of process noise up to the measurement time, we then correct the state with the measurement and in the next prediction step we need to expand the error covariance from the measurement time to the next step in the regular process.
            //  If the expansion was not to be linear (or at least close to linear), the two prediction steps, splitting a normal time step into two parts, would not add up to the same result as a single prediction with time step equal to the sum of the two.
            // To approach linearity without drastically changing the Kalman filter process (it must be based on multivariate Gaussians) while including an unknown velocity of uniform distribution, we need a Q that represents the sum of the influences of position estimate noise and of the drift due to unknown velocity.
            // The function used here was obtained thusly:
            // We want a normal distribution to approximate an original normal distribution (accounting only for static position noise sn) expanded by uniform (since we don't have any information on the velocity) distribution f_u(x) limited by maximum velocity v_max*dt=s_max (maximum position shift caused by unknown velocity)
            // This uniform distribution has center at mu, and its values are {f_u(x) = 1/(2*s_max) for abs(x-mu) < s_max} and {f_u(x) = 0 otherwise}, so that the integral of the distribudion is 1.
            // A normal distribution with the same standard deviation as such a sum of a normal and uniform distributions resolves (after some calculations) to one with a new standard deviation:
            //  sn_new^2 = (1/2)*sn^2 + (1/6)*v_max^2*dt^2
            // This is not linear. We may be overcomplicating the issue here, and this is a subject to testing in the future.
            filter_matrices.Q <<
              0.5*sn*sn+0.16667*vl*vl*dt*dt,0,0, 0,0,0,
              0,0.5*sn*sn+0.16667*vl*vl*dt*dt,0, 0,0,0,
              0,0,0.5*sn*sn+0.16667*vv*vv*dt*dt, 0,0,0,
              0,0,0, 0.5,0,0,
              0,0,0, 0,0.5,0,
              0,0,0, 0,0,0.5;
          }
        }
        return filter_matrices.Q;
      }
      //}

  };

} //uvdar

int main(int argc, char** argv) {
  ros::init(argc, argv, "uvdar_kalman_anonymous");
  ros::NodeHandle nh("~");
  uvdar::UVDARKalman        kl(nh);

  ROS_INFO("[UVDARKalman]: filter node initiated");

  ros::spin();

  return 0;
}
