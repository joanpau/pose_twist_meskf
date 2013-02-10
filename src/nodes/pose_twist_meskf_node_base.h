/**
 * @file
 * @brief Pose twist error state extended Kalman filter
 * with multiplicative orientation error node class (presentation).
 */


#ifndef POSE_TWIST_MESKF_NODE_BASE_H
#define POSE_TWIST_MESKF_NODE_BASE_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "pose_twist_meskf/Start.h"
#include "pose_twist_meskf/Stop.h"
#include "pose_twist_meskf.h"

namespace pose_twist_meskf
{

/**
 * @brief Base class for pose-twist error state Kalman filter with
 * multiplicative orientation error.
 *
 * Imu's gyroscope and accelerometer readings are treated as control inputs.
 * Visual pose estimates and depth estimates are treated as measurements.
 *
 * @par Subscribes:
 *
 *  - @b imu topic (sensor_msgs/Imu) IMU data (3D acceleration and angular
 *  velocity readings). Readings are assumed to be in body-fixed frame and the
 *  imu frame is totally ignored.
 *
 *  - @b depth topic (nav_msgs/Depth) Depth estimations (usually from pressure
 *  sensor readings). They are assumed to be the z-component of the vehicle's
 *  position (the z-component of the body-fixed frame origin in the reference
 *  frame).
 *
 *  - @b visual_odometry topic (nav_msgs/Odometry) Odometry from vision system,
 *  only velocities are used.
 *
 * @par Advertises:
 *
 * - @b odometry (nav_msgs/Odometry) Pose and twist with covariance
 * computed by the filter after correction from sensor measurements.
 *
 * - @b gyroscope_bias (geometry_msgs/Vector3Stamped) Gyroscopes' bias
 * in each axis (body-fixed frame).
 *
 * - @b accelerometer_bias (geometry_msgs/Vector3Stamped) Accelerometers' bias
 * in each axis (body-fixed frame).
 *
 * @par Published transforms:
 *
 * - @b visual_odometry_frame_id to frame_id.
 *
 * @par Services:
 *
 * - @b start (pose_twist_meskf/Start) Start/reset service with desired pose and
 * twist values and respective covariances. The node subscribes to input and
 * measurement topics and starts working.
 *
 * - @b stop (pose_twist_meskf/Stop) Stop service. The node unsubscribes from
 * input and measurement topics and stops publishing estimations.
 *
 * @par Parameters:
 *
 * - @b frame_id reference frame.
 * - @b child_frame_id body-fixed frame.
 * - @b visual_odometry_frame_id visual odometry reference frame.
 * - @b update_rate filter update frequency (input and measurement processing).
 */
class PoseTwistMESKFNodeBase
{
public:
  PoseTwistMESKFNodeBase(const ros::NodeHandle& node,
                         const ros::NodeHandle& priv);
  void initializeParameters();
  void advertiseTopics();
  void advertiseServices();

private:
  // Node handles, publishers, subscribers and service server.
  ros::NodeHandle node_;
  ros::NodeHandle priv_;
  ros::Subscriber subs_imu_;
  ros::Subscriber subs_visual_odom_;
  ros::Subscriber subs_depth_;
  ros::Publisher publ_odom_;
  ros::Publisher publ_odom_pre_;
  ros::Publisher publ_gyro_drift_;
  ros::Publisher publ_accel_bias_;
  ros::Timer update_timer_;
  ros::ServiceServer serv_start_;
  ros::ServiceServer serv_stop_;

  // TODO: Publish transforms.
  // Transforms.
  //tf::TransformBroadcaster tf_broadcaster_;

  // Frame names.
  std::string frame_id_;             //!> Reference frame.
  std::string child_frame_id_;       //!> Body-fixed frame.
  std::string visual_odom_frame_id_; //!> Visual odometry reference frame.

  // Update rate.
  double update_rate_; //!> Filter update and publishing frequency.
  
  // Standard gravity vector in reference frame.
  Eigen::Vector3d G_VEC_;

  // MESKF filter.
  PoseTwistMESKF filter_;

  // Callbacks and auxiliary functions.
  bool startCallback(pose_twist_meskf::Start::Request& req,
                     pose_twist_meskf::Start::Response& res);
  bool stopCallback(pose_twist_meskf::Stop::Request& req,
                    pose_twist_meskf::Stop::Response& res);
  void subscribeTopics();
  void unsubscribeTopics();
  void IMUCallback(const sensor_msgs::ImuConstPtr& msg);
  void visualCallback(const nav_msgs::OdometryConstPtr& msg);
  void updateCallback();
};

} // namespace

#endif // POSE_TWIST_MESKF_NODE_BASE_H
