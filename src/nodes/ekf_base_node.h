/** @file
 *
 * @brief Pose twist error state extended Kalman filter
 * with multiplicative orientation error.
 *
 * Imu's gyroscope and accelerometer readings are treated as control inputs.
 * Visual pose estimates and pressure readings are treated as measurements.
 *
 * @par Subscribes
 *
 *  - @b mag topic (memsense_imu/ImuMAG) IMU data including magnetometer readings.
 *
 *  - @b pressure topic (albatros_motorboard/PressureStamped) Pressure sensor readings.
 *
 *  - @b visual_odom topic (nav_msgs/Odometry) Odometry from vision system.
 *
 *
 * @par Advertises
 *
 * - @b odometry (nav_msgs/Odometry) Pose and twist with covariance
 * computed by the filter after correction from sensor measurements.
 *
 * - @b odometry_pre (nav_msgs/Odometry) Pose and twist with covariance
 * computed by the filter without correction from sensor measurements.
 *
 * @par Parameters
 *
 * - @b frame_id reference frame
 * - @b child_frame_id body-fixed frame
 * - @b output_rate frequency of combined odometry publications
 *
 */


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <memsense_imu/ImuMAG.h>
#include <srv_msgs/Pressure.h>

class PoseTwistEKFBaseNode
{
public:
  PoseTwistEKFBaseNode(const ros::NodeHandle& node,
                       const ros::NodeHandle& priv);

  void initialize();

  void advertiseTopics();

  void subscribeTopics();

private:

  // Subscribers and publishers
  ros::Subscriber imu_subs_;
  ros::Subscriber vision_subs_;
  ros::Subscriber pressure_subs_;
  ros::Publisher odom_publ_;
  ros::Publisher odom_pre_publ_;

  // Frames
  std::string frame_id_;
  std::string child_frame_id_;

  // Transforms
  tf::TransformBroadcaster tf_broadcaster_;

  // time
  ros::Time last_time_, current_time_;
  
  // float precision
  double epsilon_;

  // standard gravity constant
  static const double G_STD_;
  static const tf::Vector3 G_VEC_;

  void IMUCallback(const sensor_msgs::ImuConstPtr& msg);
  void PressureCallback(const albatros_motorboard::PressureStamped& msg);
  void VisualOdomCallback(const nav_msgs::OdometryConstPtr& msg);
  void UpdateCallback();
  bool initializeOdometry(const sensor_msgs::ImuConstPtr& msg);

};
