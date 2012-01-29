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




const double ImuOdom::G_STD_ = 9.80665;
const Vector3 ImuOdom::G_VEC_(0.0,0.0,G_STD_);

ImuOdom::ImuOdom()
: odom_ready_(false)
{}

void PoseTwistEKFBaseNode::initParams()
{
  imu_subs_.
  // tf frame parameters
  priv_.param<std::string>("odometer_frame_id",frame_id_,"imu_odom");
  priv_.param<std::string>("imu_frame_id",child_frame_id_,"imu");
  priv_.param("epsilon",epsilon_,1e-3);
}

void ImuOdom::advertiseOutputTopics()
{
  ROS_INFO_STREAM("[imu_odom] Advertising output topics");
  odom_publ_ = node_.advertise<nav_msgs::Odometry>("imu_odom", 1);
}

void ImuOdom::subscribeInputTopics()
{
  ROS_INFO_STREAM("[imu_odom] Subscribing to IMU topic");
  imu_subs_ = node_.subscribe<memsense_imu::ImuMAG>("data", 1,
                                                &ImuOdom::dataReceivedCallback,
                                                this);
}

bool ImuOdom::initializeOdom(const memsense_imu::ImuMAGConstPtr& data)
{
  current_time_ = data->header.stamp;
  tf::vector3MsgToTF(data->linear_acceleration, lin_acc_ );
  tf::vector3MsgToTF(data->angular_velocity, ang_vel_ );
  tf::quaternionMsgToTF(data->orientation, ori_quat_);
  lin_vel_.setValue(0.0, 0.0, 0.0);
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
    {
      lin_acc_cov_[i][j] = data->linear_acceleration_covariance[3*i+j];
      ang_vel_cov_[i][j] = data->angular_velocity_covariance[3*i+j];
      ori_cov_[i][j] = data->orientation_covariance[3*i+j];
      lin_vel_cov_[i][j] = 0.0;
    }

  // check if orientation was actually in the Imu message
  if (ori_cov_[0][0]<0.0)
  {
    // compute orientation from gravity and magnetic field
    const double mx = data->magnetic_field.x;
    const double my = data->magnetic_field.y;
    const double mz = data->magnetic_field.z;
    const double gx = data->linear_acceleration.x;
    const double gy = data->linear_acceleration.y;
    const double gz = data->linear_acceleration.z;
    const double g_length = sqrt(gx*gx + gy*gy + gz*gz);
    if ( const double e = abs(g_length - G_STD_) > epsilon_ )
    {
      ROS_WARN_STREAM("Could not initialize imu data angular integration "
                      "from a non static condition (imu acceleration is "
                      << g_length << " m/s^2, deviation from static is condition is " 
                      << e << "  )");
      return false;
    }
    const double r = atan2(gy,gz);
    const double p = -asin(gx/g_length);
    const double y = atan2(my*cos(r)-mz*sin(r),
                           cos(p)*mx + sin(p)*sin(r)*my + sin(p)*cos(r)*mz);
    ori_quat_.setRPY(r,p,y);
    ori_cov_.setValue(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    ROS_INFO_STREAM("IMU odometer initialized with RPY : " 
                    << r << " " << p << " " << y );
    // compute covariance of roll, pitch and yaw
    // from linearization of above formula
    // (if [gx,gy,gz] covariance is S and [roll,pitch,yaw] = f(gx,gy,gx)
    //  [roll,pitch,yaw] covariance is  A . S . A^t  with A = J_f(gx,gy,gz))
    // const double a = gy*gy + gz*gz;
    // const double b = sqrt(a);
    // const double c = a + gx*gx;
    // ori_cov_.setValue( 0.0, gz/a, -gy/a, b/c , -(gx*gy)/(b*c) , -(gx*gz)/(b*c), 0.0, 0.0, 0.0 );
    // ori_cov_ *= lin_acc_cov_.timesTranspose(ori_cov_);
  }
  return true;
}

void ImuOdom::updateOdom(const memsense_imu::ImuMAGConstPtr& data)
{
  // update time values
  last_time_ = current_time_;
  current_time_ = data->header.stamp;
  const double dt = (current_time_ - last_time_).toSec();
  const double dt2 = dt*dt;

  // take angular velocity from message
  tf::vector3MsgToTF(data->angular_velocity, ang_vel_);
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      ang_vel_cov_ [i][j] = data->angular_velocity_covariance[3*i+j];

  // Update orientation in initial frame if it is not filled in the message.
  // An option is to the angular motion equation by an Euler method
  // as in p. 15 of Guidance and Control of Ocean Vehicles by Fossen.
  // The following uses a quaternion based multiplicative aproach
  // and a 3D vector error representation for the covariance as described
  // in http://www.ijs.si/~aude/publications/ras99.pdf
  // (Note that the factors order is inverted to follow ros convention
  // on quaternion-based attitude representation).
  if (data->orientation_covariance[0]<0.0)
  {
    // ori_quat_ += (ori_quat_*ang_vel_) * (dt*0.5);
    // ori_quat_.normalize();
    ori_quat_ *= btQuaternion(ang_vel_,dt*ang_vel_.length());
    // btMatrix3x3 does not provide a sum operator
    const double n0 = ang_vel_[0];
    const double n1 = ang_vel_[1];
    const double n2 = ang_vel_[2];
    const double l = ang_vel_.length();
    const double a = l*dt;
    const double c = (1-cos(a)) / l;
    const double s = sin(a) / l;
    const double f = (dt-s);
    btMatrix3x3 R( btQuaternion(ang_vel_, a) );
    btMatrix3x3 aux = R*ori_cov_.timesTranspose(R);
    ori_cov_.setValue( aux[0][0] + f*n0*n0 + s,    aux[0][1] + f*n0*n1 - c*n2, aux[0][2] + f*n0*n2 + c*n1,
                       aux[1][0] + f*n1*n0 + c*n2, aux[1][1] + f*n1*n1 + s,    aux[1][2] + f*n1*n2 - c*n0,
                       aux[2][0] + f*n2*n0 - c*n1, aux[2][1] + f*n2*n1 + c*n0, aux[2][2] + f*n2*n2 + s );
    ori_cov_[1] += dt2*ang_vel_cov_[1];
    ori_cov_[2] += dt2*ang_vel_cov_[2];
  }
  else
  {
    tf::quaternionMsgToTF(data->orientation,ori_quat_);
    ori_cov_.setFromOpenGLSubMatrix(data->orientation_covariance.data());
//    for (int i=0; i<3; i++)
//      for (int j=0; j<3; j++)
//        ori_cov_ [i][j] = data->orientation_covariance[3*i+j];
  }

  // update acceleration correcting gravity in the body-fixed frame
  tf::Vector3 reading;
  tf::vector3MsgToTF(data->linear_acceleration, reading);
  lin_acc_ = tf::Transform(ori_quat_).inverse()*G_VEC_ - reading;
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      lin_acc_cov_ [i][j] = data->linear_acceleration_covariance[3*i+j];

  // update velocity in body-fixed frame
  lin_vel_ += dt * lin_acc_;
  lin_vel_cov_[0] += dt2*lin_acc_cov_[0];
  lin_vel_cov_[1] += dt2*lin_acc_cov_[1];
  lin_vel_cov_[2] += dt2*lin_acc_cov_[2];

  // TODO check linear velocity frame and orientation and velocity covariances

}

void ImuOdom::publishMsgAndTf()
{
  nav_msgs::OdometryPtr odom_msg(new nav_msgs::Odometry);
  odom_msg->header.stamp = current_time_;
  odom_msg->header.frame_id = frame_id_;
  odom_msg->child_frame_id = child_frame_id_;
  tf::quaternionTFToMsg(ori_quat_, odom_msg->pose.pose.orientation);
  tf::vector3TFToMsg(lin_vel_, odom_msg->twist.twist.linear);
  tf::vector3TFToMsg(ang_vel_, odom_msg->twist.twist.angular);
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
    {
      odom_msg->pose.covariance [6*i+j  ]  = (i==j) ? -1.0 : 0.0;
      odom_msg->pose.covariance [6*i+j+3]  = ori_cov_[i][j];
      odom_msg->twist.covariance[6*i+j+18] = lin_vel_cov_[i][j];
      odom_msg->twist.covariance[6*i+j+21] = ang_vel_cov_[i][j];
    }

  tf::StampedTransform odom_tf;
  odom_tf.frame_id_ = frame_id_;
  odom_tf.child_frame_id_ = child_frame_id_;
  odom_tf.stamp_ = current_time_;
  odom_tf.setData(tf::Transform(ori_quat_));
  tf_broadcaster_.sendTransform(odom_tf);
  odom_publ_.publish(odom_msg);

}

void ImuOdom::dataReceivedCallback(const memsense_imu::ImuMAGConstPtr& data)
{
  
  if (odom_ready_)
  {
    // update pose and twist from new sample
    updateOdom(data);
    // send pose and transform
    publishMsgAndTf();
  }
  else if (initializeOdom(data))
  {
    // odometer initialized from first message arrival in a static condition
    odom_ready_ = true;
    publishMsgAndTf();
  }
}


int main(int argc, char* argv[])
{
  ros::init(argc,argv,"imu_odom_node");

  ImuOdom imu_odom;

  imu_odom.initParams();

  imu_odom.advertiseOutputTopics();

  imu_odom.subscribeInputTopics();

  ros::spin();
}
