/**
 * @file analyticconditionalgaussian_posetwisterrorstate.cpp
 * @brief Conditional distribution for pose-twist error state (presentation)
 * @author Joan Pau Beltran
 *
 * This is the needed implementation of the pose-twist error state conditional
 * distribution with additive noise.
 *
 * Actually it is a hack because BFL does not implement error state filters
 * directly, and an ordinary ExtendeKalmanFilter is used to deal
 * only with the error. The problem is how to implement the conditional pdf
 * since the state transition depends on the values of the nominal state.
 *
 * The adopted solution is this:
 *
 * - The filter only takes care of the error-state update and correct.
 *
 * - The error state conditional pdf keeps the nominal state internally.
 *
 * - Nominal state update is performed during the error state update.
 *   Before an error state system update, this nominal state must be updated
 *   and the new nominal values must be used to update the error state.
 *
 * - Reset is performed manually (correcting the nominal state
 *   with the filter's post mean and resetting it to zero).
 *
 * The variable time step problem is solved
 * passing the time step as an element of the input.
 *
 * The details about the model may be found in:
 * - Joan Sola,
 *   Quaternion kinematics for the error-state KF
 *   http://homepages.laas.fr/jsola/JoanSola/objectes/notes/kinematics.pdf
 *
 * - Nikolas Trawny and Stergios I. Roumeliotis,
 *   Indirect Kalman Filter for 3D Attitude Estimation
 *   http://www.asl.ethz.ch/education/master/aircraft/2010_L9_Quaternions_3D.pdf
 *
 * - Ales Ude,
 *   Filtering in a Unit Quaternion Space for Model-Based Object Tracking
 *   http://www.cns.atr.jp/~aude/publications/ras99.pdf
 *
 * - F. Landis Markley,
 *   Attitude Error Representations for Kalman Filtering
 *   http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20020060647_2002098710.pdf
 *
 */

#include "analyticconditionalgaussian_posetwisterrorstate.h"
#include "vector_conversions.h"

const double BFL::AnalyticConditionalGaussianPoseTwistErrorState::G_CONS = 9.80665;
const Eigen::Vector3d
BFL::AnalyticConditionalGaussianPoseTwistErrorState::G_VECT = Eigen::Vector3d(0.0,0.0,G_CONS);

const double
BFL::AnalyticConditionalGaussianPoseTwistErrorState::ANGULAR_RATE_EPSILON = 1E-3;

/**
 * @brief Build the skew-symmetric matrix that computes the cross product.
 * @param v left vector in cross product.
 * @return matrix $M$ such that $M w = v \cross w$ for all $w$.
 */
Eigen::Matrix3d BFL::AnalyticConditionalGaussianPoseTwistErrorState::
skew(const Eigen::Vector3d& v) const
{
  Eigen::Matrix3d m;
  m <<  0.0 , -v(3),  v(2),
        v(3),  0.0 , -v(1),
       -v(2),  v(1),  0.0 ;
  return m;
}


/**
 * @brief Default constructor.
 * @param noise additive system noise pdf.
 * @return
 */
BFL::AnalyticConditionalGaussianPoseTwistErrorState::
AnalyticConditionalGaussianPoseTwistErrorState(const Gaussian& noise)
: AnalyticConditionalGaussianAdditiveNoise(noise, 2) // 2 conditional arguments
{}


/**
 * @brief Transfer the error to the nominal state and reset it.
 *
 * The update is performed according to these rules:
 * - pose:
 *      p_t = p + dp
 * - orientation:
 *      q_t = q * exp(1/2*dq)  with exp(1/2*dq) = [ cos(1/2*|dq|) , sin(1/2*|dq|) * dq/|dq| ]
 * - linear velocity:
 *      v_t = v + dv
 * - accelerometer bias:
 *      b_t = b + db
 * - gyroscope drift:
 *      d_t = d + dd
 * - linear acceleration:
 *      a_t = R'g - a_s + b + db
 * - angular velocity:
 *      w_t = w_s - d - dd
 */
void BFL::AnalyticConditionalGaussianPoseTwistErrorState::
ResetErrorState()
{
  // Correct the nominal state.
  pose_ += d_pose_;
  orientation_ *= Eigen::Quaterniond(Eigen::AngleAxisd(d_orientation_.norm(),
                                                       d_orientation_.normalized()));
  // orientation_.normalize();
  lin_vel_ += d_lin_vel_;
  acc_bias_ += d_acc_bias_;
  gyro_drift_ += d_gyro_drift_;
  lin_acc_ += d_acc_bias_;
  ang_vel_ -= d_gyro_drift_;

  // Reset the error state.
  d_pose_ = 0.0;
  d_orientation_ = 0.0;
  d_lin_vel_ = 0.0;
  d_acc_bias_ = 0.0;
  d_gyro_drift_ = 0.0;
}

/**
 * @brief Set the current nominal state.
 * @param x nominal state vector.
 */
void
BFL::AnalyticConditionalGaussianPoseTwistErrorState::
NominalStateSet(const MatrixWrapper::ColumnVector& x)
{
  pose_twist_meskf::decomposeNominalState(x,
                                          pose_,orientation_,lin_vel_,
                                          acc_bias_,gyro_drift_,
                                          ang_vel_,lin_acc_);
}

/**
 * @brief Get the current nominal state.
 * @return current nominal state vector.
 */
MatrixWrapper::ColumnVector
BFL::AnalyticConditionalGaussianPoseTwistErrorState::NominalStateGet()
{
  MatrixWrapper::ColumnVector x(pose_twist_meskf::NOMINAL_STATE_DIM);
  pose_twist_meskf::composeNominalState(x,
                                        pose_,orientation_,lin_vel_,
                                        acc_bias_,gyro_drift_,
                                        ang_vel_,lin_acc_);
  return x;
}

/**
 * @brief Update the error state and implicitly the nominal state.
 *
 * The nominal state update is performed according to these rules:
 * - linear acceleration:
 *      a = R'g - a_s + b
 * - angular velocity:
 *      w = w_s - d   (angle turned = dt*|w| , axis turned = w / |w|)
 * - pose:
 *      p_next = p + dt*R*v + 0.5*dt^2*[g - R*(a_s-b)] = p + dt*R*v + 0.5*dt^2*R*a
 * - orientation:
 *      q_next = q * q(angle turned, axis turned)
 * - linear velocity:
 *      v_next = v + dt*(R'*g - a_s + b) = v + dt*a
 * - accelerometer bias:
 *      b_next = b
 * - gyroscope drift:
 *      d_next = d
 *
 * The error state update may be omitted
 * since it will lead to the same zero error state
 * (provided that the previous error is zero,
 * which always the case because of the filter reset).
 * In any case, these are the rules:
 * - pose:
 *      dp_next = dp + dt*R*dv + (-dt*R*[v]_x* + 0.5*dt^2*a)*dq + 0.5*dt^2*R*db
 * - orientation:
 *      dq_next = R(angle turned, axis turned)'*dq
 *                + { -dt*I + (1-cos(dt*|w|))/|w|^2*[w]_x - (dt*|w|-sin(dt*|w|))/|w|^3*[w]_x^2 }*db
 * - linear velocity:
 *      dv_next = v + dt*[R'*g]_x*dq + dt*db
 * - accelerometer bias:
 *      db_next = db
 * - gyroscope drift:
 *      dd_next = dd
 *
 */
MatrixWrapper::ColumnVector
BFL::AnalyticConditionalGaussianPoseTwistErrorState::ExpectedValueGet() const
{
  // Get the values of the current state and input.
  ColumnVector e = ConditionalArgumentGet(0);
  ColumnVector u = ConditionalArgumentGet(1);

  // Update the nominal state following the above rules.
  Eigen::Vector3d w_s, a_s;
  double dt;
  pose_twist_meskf::decomposeInput(u,w_s,a_s,dt);
  const double dt2 = dt*dt;

  Eigen::Matrix3d R = orientation_.toRotationMatrix();

  lin_acc_ = R.transpose()*G_VECT - a_s + acc_bias_;
  ang_vel_ = w_s - gyro_drift_;

  const double rate = ang_vel_.norm();
  const double angle = dt*rate;
  const Eigen::Vector3d axis = ang_vel_.normalized();
  Eigen::AngleAxisd angle_axis(angle,axis);

  pose_ += R*(dt*lin_vel_ + 0.5*dt2*lin_acc_);
  lin_vel_ += dt*lin_acc_;
  orientation_*= Eigen::Quaterniond(angle_axis);
  // orientation_.normalize();
  // gyro_drift_ and acc_bias_ are constant, so keep their value.

  // Update the error state.
  // If the error state is zero (it should be because of the reset)
  // the update will not change it.
  // However these are the equations:
  pose_twist_meskf::decomposeErrorState(e,
                                        d_pose_, d_orientation_, d_lin_vel_,
                                        d_acc_bias_, d_gyro_drift_);
  d_pose_ += dt*R*d_lin_vel_ - dt*R*lin_vel_.cross(d_orientation_)
           + 0.5*dt2*lin_acc_.cross(d_orientation_)
           + 0.5*dt2*R*d_acc_bias_;
  d_lin_vel_ += dt*skew(R.transpose()*G_VECT)*d_orientation_ + dt*d_acc_bias_;
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3,3);
  const Eigen::Matrix3d S = skew(ang_vel_);
  const Eigen::Matrix3d S2 = S*S;
  const Eigen::Matrix3d TH = angle_axis.toRotationMatrix().transpose();
                        // = I - (sin(angle)/rate)*S + ((1-cos(angle))/pow(rate,2))*S2
                        // ~ I - dt*S + 0.5*dt2*S2
  const Eigen::Matrix3d PS = ( rate < ANGULAR_RATE_EPSILON )
                             ? - dt*I
                             : - dt*I + ((1-cos(angle))/pow(rate,2))*S - ((angle-sin(angle))/pow(rate,3))*S2;
  d_orientation_ = TH*d_orientation_ + PS*d_gyro_drift_;
  // gyro_drift_ and acc_bias_ are constant, so their errors keep their value.

  pose_twist_meskf::composeErrorState(e,
                                      d_pose_, d_orientation_, d_lin_vel_,
                                      d_acc_bias_, d_gyro_drift_);
  return e;
}

MatrixWrapper::Matrix dfGet(unsigned int i) const;
