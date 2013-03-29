/**
 * @file
 * @brief Pose twist error state extended Kalman filter
 * with multiplicative orientation error node class (implementation).
 */


#include "pose_twist_meskf_node_base.h"
#include "nominal_state_vector.h"
#include "error_state_vector.h"
#include "input_vector.h"
#include "visual_measurement_vector.h"
#include "visual_measurement_error_vector.h"


/**
 * @brief Default constructor.
 * @param node node handle to node's namespace.
 * @param priv node handle to node's private namespace.
 * @return
 */
pose_twist_meskf::PoseTwistMESKFNodeBase::
PoseTwistMESKFNodeBase(const ros::NodeHandle& node,
                       const ros::NodeHandle& priv)
: node_(node), priv_(priv)
{}


/**
 * @brief Initialize parameters from parameter server or default values.
 */
void pose_twist_meskf::PoseTwistMESKFNodeBase::initializeParameters()
{
  priv_.param<std::string>("frame_id", frame_id_, "map");
  priv_.param<std::string>("child_frame_id", child_frame_id_, "base_link");
  priv_.param<std::string>("visual_odometry_frame_id", visual_odom_frame_id_,
                           "visual_odom");
  priv_.param("update_rate", update_rate_, 10.0);
  priv_.param("gravity_x", G_VEC_(0), 0.0);
  priv_.param("gravity_y", G_VEC_(1), 0.0);
  priv_.param("gravity_z", G_VEC_(2), 9.80665);
  ROS_DEBUG_STREAM("Reference frame: " << frame_id_ << ".");
  ROS_DEBUG_STREAM("Body-fixed frame: " << child_frame_id_ << ".");
  ROS_DEBUG_STREAM("Visual odometry frame: " << visual_odom_frame_id_ << ".");
  ROS_DEBUG_STREAM("Update rate: " << visual_odom_frame_id_ << " Hz.");
}


/**
 * @brief Advertise combined odometry and IMU bias topics.
 */
void pose_twist_meskf::PoseTwistMESKFNodeBase::advertiseTopics()
{
  publ_odom_ = node_.advertise<nav_msgs::Odometry>("odometry", 10);
  publ_odom_pre_ = node_.advertise<nav_msgs::Odometry>("odometry_pre", 10);
  publ_gyro_drift_ = node_.advertise<geometry_msgs::Vector3Stamped>("gyroscope_bias", 10);
  publ_accel_bias_ = node_.advertise<geometry_msgs::Vector3Stamped>("accelerometer_bias", 10);
}


/**
 * @brief Advertise start and stop services.
 */
void pose_twist_meskf::PoseTwistMESKFNodeBase::advertiseServices()
{
  serv_start_ = node_.advertiseService("start",
                                       &PoseTwistMESKFNodeBase::startCallback,
                                       this);
  serv_stop_ = node_.advertiseService("stop",
                                      &PoseTwistMESKFNodeBase::stopCallback,
                                      this);
}


/**
 * @brief Service callback to launch or reset the filter.
 * @param req service request with initial estimates and covariances.
 * @param res service response with old filter status information.
 */
bool pose_twist_meskf::PoseTwistMESKFNodeBase::
startCallback(pose_twist_meskf::Start::Request& req,
              pose_twist_meskf::Start::Response& res)
{
  PoseTwistMESKF::Vector state(NominalStateVector::DIMENSION);
  state(NominalStateVector::POSITION_X) = req.pose.position.x;
  state(NominalStateVector::POSITION_Y) = req.pose.position.y;
  state(NominalStateVector::POSITION_Z) = req.pose.position.z;
  state(NominalStateVector::ORIENTATION_W) = req.pose.orientation.w;
  state(NominalStateVector::ORIENTATION_X) = req.pose.orientation.x;
  state(NominalStateVector::ORIENTATION_Y) = req.pose.orientation.y;
  state(NominalStateVector::ORIENTATION_Z) = req.pose.orientation.z;
  state(NominalStateVector::LIN_VEL_X) = req.twist.linear.x;
  state(NominalStateVector::LIN_VEL_Y) = req.twist.linear.y;
  state(NominalStateVector::LIN_VEL_Z) = req.twist.linear.z;
  state(NominalStateVector::ANG_VEL_X) = req.twist.angular.x;
  state(NominalStateVector::ANG_VEL_Y) = req.twist.angular.y;
  state(NominalStateVector::ANG_VEL_Z) = req.twist.angular.z;
  state(NominalStateVector::GYRO_DRIFT_X) = req.bias_gyro.x;
  state(NominalStateVector::GYRO_DRIFT_Y) = req.bias_gyro.y;
  state(NominalStateVector::GYRO_DRIFT_Z) = req.bias_gyro.z;
  state(NominalStateVector::ACC_BIAS_X) = req.bias_accl.x;
  state(NominalStateVector::ACC_BIAS_Y) = req.bias_accl.y;
  state(NominalStateVector::ACC_BIAS_Z) = req.bias_accl.z;

  PoseTwistMESKF::SymmetricMatrix covariance(ErrorStateVector::DIMENSION);
  covariance = 0.0;
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
    {
      covariance(ErrorStateVector::D_POSITION_X + i,
                 ErrorStateVector::D_POSITION_X + j) = req.covariance_pose[6*i + j];
      covariance(ErrorStateVector::D_POSITION_X + i,
                 ErrorStateVector::D_ORIENTATION_X + j) = req.covariance_pose[6*i + j + 3];
      covariance(ErrorStateVector::D_ORIENTATION_X + i,
                 ErrorStateVector::D_POSITION_X + j) = req.covariance_pose[6*i + j + 18];
      covariance(ErrorStateVector::D_ORIENTATION_X + i,
                 ErrorStateVector::D_ORIENTATION_X + j) = req.covariance_pose[6*i + j + 21];
      covariance(ErrorStateVector::D_LIN_VEL_X + i,
                 ErrorStateVector::D_LIN_VEL_X + j) = req.covariance_twist[6*i + j];
      covariance(ErrorStateVector::D_GYRO_DRIFT_X + i,
                 ErrorStateVector::D_GYRO_DRIFT_X + j) = req.covariance_bias[6*i + j];
      covariance(ErrorStateVector::D_GYRO_DRIFT_X + i,
                 ErrorStateVector::D_ACC_BIAS_X + j) = req.covariance_bias[6*i + j + 3];
      covariance(ErrorStateVector::D_ACC_BIAS_X + i,
                 ErrorStateVector::D_GYRO_DRIFT_X + j) = req.covariance_bias[6*i + j + 18];
      covariance(ErrorStateVector::D_ACC_BIAS_X + i,
                 ErrorStateVector::D_ACC_BIAS_X + j) = req.covariance_bias[6*i + j + 21];
    }

  double timestamp = req.stamp.toSec();

  filter_.initialize(state, covariance, timestamp);

  if (update_timer_.isValid() && update_timer_.hasPending())
    res.status_message = "Filter reset.";
  else
  {
    subscribeTopics();
    if (update_timer_.isValid())
    {
      update_timer_.start();
    }
    else
    {
      update_timer_ = node_.createTimer(ros::Duration(update_rate_),
                                        boost::bind(&PoseTwistMESKFNodeBase::updateCallback, this));
    }
    res.status_message = "Filter started.";
  }

  return true;
}


/**
 * @brief Service callback to stop filtering.
 * @param req service request empty (anything needed).
 * @param res service response with old filter status information.
 */
bool pose_twist_meskf::PoseTwistMESKFNodeBase::
stopCallback(pose_twist_meskf::Stop::Request& req,
             pose_twist_meskf::Stop::Response& res)
{
  if (update_timer_.isValid() && update_timer_.hasPending())
  {
      update_timer_.stop();
      unsubscribeTopics();
      res.status_message = "Filter stopped.";
  }
  else
  {
    res.status_message = "Filter already stopped.";
  }
  return true;
}


/**
 * @brief Subscribe to IMU and visual odometry topics.
 */
void pose_twist_meskf::PoseTwistMESKFNodeBase::subscribeTopics()
{
  subs_imu_ = node_.subscribe("imu", 10,
                              &PoseTwistMESKFNodeBase::IMUCallback,
                              this);
  subs_visual_odom_ = node_.subscribe("visual_odometry", 10,
                                      &PoseTwistMESKFNodeBase::visualCallback,
                                      this);
}


/**
 * @brief Shutdown IMU and visual odometry subscribers.
 */
void pose_twist_meskf::PoseTwistMESKFNodeBase::unsubscribeTopics()
{
  subs_imu_.shutdown();
  subs_visual_odom_.shutdown();
}


/**
 * @brief IMU reading callback.
 * @param msg new IMU reading to be enqueued.
 */
void pose_twist_meskf::PoseTwistMESKFNodeBase::
IMUCallback(const sensor_msgs::ImuConstPtr& msg)
{
  InputVector input;
  input.time_ = msg->header.stamp.toSec();
  input.ang_vel_ = Eigen::Vector3d(msg->angular_velocity.x,
                                   msg->angular_velocity.y,
                                   msg->angular_velocity.z);
  input.lin_acc_ = Eigen::Vector3d(msg->linear_acceleration.x,
                                   msg->linear_acceleration.y,
                                   msg->linear_acceleration.z);
  filter_.addInput(input.time_, input.toVector());
}


/**
 * @brief Visual measurement callback.
 * @param msg new visual measurement to be enqueued.
 */
void
pose_twist_meskf::PoseTwistMESKFNodeBase::
visualCallback(const nav_msgs::OdometryConstPtr& msg)
{
  double timestamp = msg->header.stamp.toSec();

  PoseTwistMESKF::Vector measurement(VisualMeasurementVector::DIMENSION);
  measurement(VisualMeasurementVector::ANG_VEL_X) = msg->twist.twist.angular.x;
  measurement(VisualMeasurementVector::ANG_VEL_Y) = msg->twist.twist.angular.y;
  measurement(VisualMeasurementVector::ANG_VEL_Z) = msg->twist.twist.angular.z;
  measurement(VisualMeasurementVector::LIN_VEL_X) = msg->twist.twist.linear.x;
  measurement(VisualMeasurementVector::LIN_VEL_Y) = msg->twist.twist.linear.y;
  measurement(VisualMeasurementVector::LIN_VEL_Z) = msg->twist.twist.linear.z;

  PoseTwistMESKF::SymmetricMatrix covariance(VisualMeasurementErrorVector::DIMENSION);
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
    {
      covariance(VisualMeasurementErrorVector::D_LIN_VEL_X + i,
                 VisualMeasurementErrorVector::D_LIN_VEL_X + j) = msg->twist.covariance[6*i + j];
      covariance(VisualMeasurementErrorVector::D_LIN_VEL_X + i,
                 VisualMeasurementErrorVector::D_GYRO_DRIFT_X + j) = msg->twist.covariance[6*i + j + 3];
      covariance(VisualMeasurementErrorVector::D_GYRO_DRIFT_X + i,
                 VisualMeasurementErrorVector::D_LIN_VEL_X + j) = msg->twist.covariance[6*i + j + 18];
      covariance(VisualMeasurementErrorVector::D_GYRO_DRIFT_X + i,
                 VisualMeasurementErrorVector::D_GYRO_DRIFT_X + j) = msg->twist.covariance[6*i + j + 21];
    }
  filter_.addMeasurement(filter_.MEAS_VISUAL, measurement, covariance, timestamp);
}


/**
 * @brief Update callback.
 *
 * Filter is updated processing inputs and measurements in all queues until
 * is processed. Final estimates are published on respective topics.
 */
void pose_twist_meskf::PoseTwistMESKFNodeBase::updateCallback()
{
  bool success = filter_.updateAll();

  if (!success)
  {
    ROS_ERROR_STREAM("Could not update filter.");
    return;
  }

  const double timestamp = filter_.getTime();
  const PoseTwistMESKF::Vector state = filter_.getEstimate();
  const PoseTwistMESKF::SymmetricMatrix covariance = filter_.getCovariance();

  geometry_msgs::Vector3StampedPtr accel_bias_msg;
  geometry_msgs::Vector3StampedPtr gyro_drift_msg;
  nav_msgs::OdometryPtr msg_odom(new nav_msgs::Odometry);
  msg_odom->header.stamp.fromSec(timestamp);
  msg_odom->header.frame_id = frame_id_;
  msg_odom->child_frame_id = child_frame_id_;
  msg_odom->pose.pose.position.x = state(NominalStateVector::POSITION_X);
  msg_odom->pose.pose.position.y = state(NominalStateVector::POSITION_Y);
  msg_odom->pose.pose.position.z = state(NominalStateVector::POSITION_Z);
  msg_odom->pose.pose.orientation.w = state(NominalStateVector::ORIENTATION_W);
  msg_odom->pose.pose.orientation.x = state(NominalStateVector::ORIENTATION_X);
  msg_odom->pose.pose.orientation.y = state(NominalStateVector::ORIENTATION_Y);
  msg_odom->pose.pose.orientation.z = state(NominalStateVector::ORIENTATION_Z);
  msg_odom->twist.twist.linear.x = state(NominalStateVector::LIN_VEL_X);
  msg_odom->twist.twist.linear.y = state(NominalStateVector::LIN_VEL_Y);
  msg_odom->twist.twist.linear.z = state(NominalStateVector::LIN_VEL_Z);
  msg_odom->twist.twist.angular.x = state(NominalStateVector::ANG_VEL_X);
  msg_odom->twist.twist.angular.y = state(NominalStateVector::ANG_VEL_Y);
  msg_odom->twist.twist.angular.z = state(NominalStateVector::ANG_VEL_Z);
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
    {
      msg_odom->pose.covariance[6*i + j] =
          covariance(ErrorStateVector::D_POSITION_X + i, ErrorStateVector::D_POSITION_X + j);
      msg_odom->pose.covariance[6*i + j + 3] =
          covariance(ErrorStateVector::D_POSITION_X + i, ErrorStateVector::D_ORIENTATION_X + j);
      msg_odom->pose.covariance[6*i + j + 18] =
          covariance(ErrorStateVector::D_ORIENTATION_X + i, ErrorStateVector::D_POSITION_X + j);
      msg_odom->pose.covariance[6*i + j + 21] =
          covariance(ErrorStateVector::D_ORIENTATION_X + i, ErrorStateVector::D_ORIENTATION_X + j);
      msg_odom->twist.covariance[6*i + j] =
          covariance(ErrorStateVector::D_LIN_VEL_X + i, ErrorStateVector::D_LIN_VEL_X + j);
      msg_odom->twist.covariance[6*i + j + 3] =
          covariance(ErrorStateVector::D_LIN_VEL_X + i, ErrorStateVector::D_GYRO_DRIFT_X + j);
      msg_odom->twist.covariance[6*i + j + 18] =
          covariance(ErrorStateVector::D_GYRO_DRIFT_X + i, ErrorStateVector::D_LIN_VEL_X + j);
      msg_odom->twist.covariance[6*i + j + 21] =
          covariance(ErrorStateVector::D_GYRO_DRIFT_X + i, ErrorStateVector::D_GYRO_DRIFT_X + j);
    }
  publ_odom_.publish(msg_odom);

  accel_bias_msg->header.stamp.fromSec(timestamp);
  accel_bias_msg->header.frame_id = frame_id_;
  accel_bias_msg->vector.x = state(NominalStateVector::ACC_BIAS_X);
  accel_bias_msg->vector.y = state(NominalStateVector::ACC_BIAS_Y);
  accel_bias_msg->vector.z = state(NominalStateVector::ACC_BIAS_Z);
  publ_accel_bias_.publish(accel_bias_msg);

  gyro_drift_msg->header.stamp.fromSec(timestamp);
  gyro_drift_msg->header.frame_id = frame_id_;
  gyro_drift_msg->vector.x = state(NominalStateVector::GYRO_DRIFT_X);
  gyro_drift_msg->vector.y = state(NominalStateVector::GYRO_DRIFT_Y);
  gyro_drift_msg->vector.z = state(NominalStateVector::GYRO_DRIFT_Z);
  publ_gyro_drift_.publish(gyro_drift_msg);

}
