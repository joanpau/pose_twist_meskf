/**
 * @file
 * @author Joan Pau Beltran
 * @brief Conditional distribution for pose-twist error state (presentation).
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
#include "error_state_vector.h"
#include "input_vector.h"


const double BFL::AnalyticConditionalGaussianPoseTwistErrorState::G_CONS = 9.80665;
const Eigen::Vector3d
BFL::AnalyticConditionalGaussianPoseTwistErrorState::G_VECT = Eigen::Vector3d(0.0,0.0,-G_CONS);

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
  m <<  0.0 , -v(2),  v(1),
        v(2),  0.0 , -v(0),
       -v(1),  v(0),  0.0 ;
  return m;
}


/**
 * @brief Default constructor.
 * @param noise additive system noise pdf.
 * @return
 */
BFL::AnalyticConditionalGaussianPoseTwistErrorState::
AnalyticConditionalGaussianPoseTwistErrorState(const double& acc_var,
                                               const double& gyro_var,
                                               const double& acc_bias_var,
                                               const double& gyro_drift_var)
: AnalyticConditionalGaussian(pose_twist_meskf::ErrorStateVector::DIMENSION,2), // 2 conditional arguments
  ACC_VAR_(acc_var),
  GYRO_VAR_(gyro_var),
  ACC_BIAS_VAR_(acc_bias_var),
  GYRO_DRIFT_VAR_(gyro_drift_var)
{}


/**
 * @brief Destructor (doing nothing).
 * @return
 */
BFL::AnalyticConditionalGaussianPoseTwistErrorState::
~AnalyticConditionalGaussianPoseTwistErrorState()
{}


/**
 * @brief Correct nominal state with given error.
 *
 * The correction is performed according to these rules:
 * - position:
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
CorrectNominalState(const MatrixWrapper::ColumnVector& e)
{
  pose_twist_meskf::ErrorStateVector error;
  error.fromVector(e);

  // Correct the nominal state.
  nominal_state_.position_ += error.d_position_;
  nominal_state_.lin_vel_ += error.d_lin_vel_;
  nominal_state_.orientation_ *= Eigen::Quaterniond(Eigen::AngleAxisd(error.d_orientation_.norm(),
                                                                      error.d_orientation_.normalized()));
  // orientation_.normalize();
  nominal_state_.acc_bias_ += error.d_acc_bias_;
  nominal_state_.gyro_drift_ += error.d_gyro_drift_;
  nominal_state_.lin_acc_ += error.d_acc_bias_;
  nominal_state_.ang_vel_ -= error.d_gyro_drift_;
}


/**
 * @brief Set the current nominal state.
 * @param x nominal state vector.
 */
void
BFL::AnalyticConditionalGaussianPoseTwistErrorState::
NominalStateSet(const MatrixWrapper::ColumnVector& x)
{
  nominal_state_.fromVector(x);
}


/**
 * @brief Get the current nominal state.
 * @return current nominal state vector.
 */
MatrixWrapper::ColumnVector
BFL::AnalyticConditionalGaussianPoseTwistErrorState::NominalStateGet()
{
  return nominal_state_.toVector();
}


/**
 * @brief Update the error state and implicitly the nominal state.
 *
 * The nominal state update is performed according to these rules:
 * - linear acceleration:
 *      a = R'g - a_s + b
 * - angular velocity:
 *      w = w_s - d   (angle turned = dt*|w| , axis turned = w / |w|)
 * - position:
 *      p_next = p + dt*R*v + 0.5*dt^2*[g - R*(a_s-b)] = p + dt*R*v + 0.5*dt^2*R*a
 * - orientation:
 *      q_next = q * q(angle turned, axis turned)
 * - linear velocity:
 *      v_next = v + dt*(R'*g-a_s+b) = v + dt*a
 * - accelerometer bias:
 *      b_next = b
 * - gyroscope drift:
 *      d_next = d
 *
 * The error state update may be omitted
 * since it will lead to the same zero error state
 * (provided that the previous error is zero,
 * which is always the case because of the filter reset).
 * Anyway, these are the rules:
 * - position:
 *      dp_next = dp + dt*R*dv + (-dt*R*[v]_x + 0.5*dt^2*R*[a_s - b]_x)*dq + 0.5*dt^2*R*db
 * - orientation:
 *      dq_next = R(angle turned, axis turned)'*dq
 *                + { -dt*I + (1-cos(dt*|w|))/|w|^2*[w]_x - (dt*|w|-sin(dt*|w|))/|w|^3*[w]_x^2 }*db
 * - linear velocity:
 *      dv_next = v + dt*[R'*g]_x*dq + dt*db
 * - accelerometer bias:
 *      db_next = db
 * - gyroscope drift:
 *      dd_next = dd
 */
MatrixWrapper::ColumnVector
BFL::AnalyticConditionalGaussianPoseTwistErrorState::ExpectedValueGet() const
{
  // Update the nominal state following the above rules.
  pose_twist_meskf::InputVector input;
  input.fromVector(ConditionalArgumentGet(1));

  Eigen::Matrix3d R = nominal_state_.orientation_.toRotationMatrix();
  nominal_state_.lin_acc_ = R.transpose()*G_VECT - input.lin_acc_ + nominal_state_.acc_bias_;
  nominal_state_.ang_vel_ = input.ang_vel_ - nominal_state_.gyro_drift_;

  const double dt = input.time_incr_;
  const double dt2 = pow(dt,2);
  const double rate = nominal_state_.ang_vel_.norm();
  const Eigen::Vector3d axis = nominal_state_.ang_vel_.normalized();
  const double angle = dt*rate;
  Eigen::AngleAxisd angle_axis(angle,axis);

  nominal_state_.position_ += R*(dt*nominal_state_.lin_vel_ + 0.5*dt2*nominal_state_.lin_acc_);
  nominal_state_.lin_vel_ += dt*nominal_state_.lin_acc_;
  nominal_state_.orientation_*= Eigen::Quaterniond(angle_axis);
  // orientation_.normalize();
  // gyro_drift_ and acc_bias_ are constant, so keep their value.

  // Update the error state.
  // If the error state is zero (it should be because of the reset)
  // the update will not change it.
  // However these are the equations:
  pose_twist_meskf::ErrorStateVector error;
  error.fromVector(ConditionalArgumentGet(0));
  error.d_position_ += dt*R*error.d_lin_vel_
                    + dt*R*error.d_orientation_.cross(nominal_state_.lin_vel_)
                    - 0.5*dt2*R*error.d_orientation_.cross(input.lin_acc_ - nominal_state_.acc_bias_)
                    + 0.5*dt2*R*error.d_acc_bias_;
  error.d_lin_vel_ += dt*(R.transpose()*G_VECT).cross(error.d_orientation_)
                   + dt*error.d_acc_bias_;

  const double dt3 = pow(dt,3);
  const double rate2 = pow(rate,2);
  const double rate3 = pow(rate,3);
  const double cos_angle = cos(angle);
  const double sin_angle = sin(angle);

  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d S = skew(nominal_state_.ang_vel_);
  const Eigen::Matrix3d S2 = S*S;
  const Eigen::Matrix3d TH = angle_axis.toRotationMatrix().transpose();
                        // = I - sin_angle/rate*S + (1-cos_angle)/rate2*S2
                        // ~ I - dt*S + 0.5*dt2*S2
  const Eigen::Matrix3d PS =
      ( rate < ANGULAR_RATE_EPSILON )
      ? ( - dt*I + dt2/2.0*S - dt3/6.0*S2 )
      : ( - dt*I + (1-cos_angle)/rate2*S - (angle-sin_angle)/rate3*S2 );

  error.d_orientation_ = TH*error.d_orientation_ + PS*error.d_gyro_drift_;
  // gyro_drift_ and acc_bias_ are constant, so their errors keep their value.

  return error.toVector();
}


/**
 * @brief Return the Jacobian of the state transition function.
 *
 * Only implemented for the first conditional argument (the previous state).
 * The matrix is build according to the equations in ExpectedValueGet().
 * Some values are approximated by simpler expressions when the angular rate
 * is smaller than ANGULAR_RATE_EPSILON.
 *
 * @param i conditional argument index.
 * @return the state transition matrix.
 */
MatrixWrapper::Matrix BFL::AnalyticConditionalGaussianPoseTwistErrorState::
dfGet(unsigned int i) const
{
  if(i!=0)
  {
    cerr << "The df is not implemented for the" << i << "th conditional argument\n";
    exit(-BFL_ERRMISUSE);
  }

  pose_twist_meskf::InputVector input;
  input.fromVector(ConditionalArgumentGet(1));

  const double dt = input.time_incr_;
  const double rate = nominal_state_.ang_vel_.norm();
  const Eigen::Vector3d axis = nominal_state_.ang_vel_.normalized();
  const double angle = dt*rate;
  Eigen::AngleAxisd angle_axis(angle,axis);

  const double dt2 = pow(dt,2);

  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R = nominal_state_.orientation_.toRotationMatrix();

  const Eigen::Matrix3d dp_dp = I;
  const Eigen::Matrix3d dp_dq = - dt*R*skew(nominal_state_.lin_vel_)
                                + 0.5*dt2*R*skew(input.lin_acc_ - nominal_state_.acc_bias_);
  const Eigen::Matrix3d dp_dv = dt*R;
  const Eigen::Matrix3d dp_db = 0.5*dt2*R;
  const Eigen::Matrix3d dv_dv = I;
  const Eigen::Matrix3d dv_dq = dt*skew(R.transpose()*G_VECT);
  const Eigen::Matrix3d dv_db = dt*I;

  const double dt3 = pow(dt,3);
  const double rate2 = pow(rate,2);
  const double rate3 = pow(rate,3);
  const double cos_angle = cos(angle);
  const double sin_angle = sin(angle);

  const Eigen::Matrix3d S = skew(nominal_state_.ang_vel_);
  const Eigen::Matrix3d S2 = S*S;

  const Eigen::Matrix3d dq_dq = angle_axis.toRotationMatrix().transpose();
                        // = I - (sin(angle)/rate)*S + ((1-cos(angle))/pow(rate,2))*S2
                        // ~ I - dt*S + 0.5*dt2*S2
  const Eigen::Matrix3d dq_dd =
      ( rate < ANGULAR_RATE_EPSILON )
      ? ( - dt*I + dt2/2.0*S - dt3/6.0*S2 )
      : ( - dt*I + (1-cos_angle)/rate2*S - (angle-sin_angle)/rate3*S2 );

  const Eigen::Matrix3d db_db = I;
  const Eigen::Matrix3d dd_dd = I;

  const int dim = DimensionGet();
  MatrixWrapper::Matrix dF(dim, dim);
  dF = 0.0;
  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3; j++)
    {
      // Position:
      dF(pose_twist_meskf::ErrorStateVector::D_POSITION_X + i,
         pose_twist_meskf::ErrorStateVector::D_POSITION_X + j) = dp_dp(i,j);
      dF(pose_twist_meskf::ErrorStateVector::D_POSITION_X + i,
         pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + j) = dp_dq(i,j);
      dF(pose_twist_meskf::ErrorStateVector::D_POSITION_X + i,
         pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X + j) = dp_dv(i,j);
      dF(pose_twist_meskf::ErrorStateVector::D_POSITION_X + i,
         pose_twist_meskf::ErrorStateVector::D_ACC_BIAS_X + j) = dp_db(i,j);
      // Velocity:
      dF(pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X + i,
         pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X + j) = dv_dv(i,j);
      dF(pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X + i,
         pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + j) = dv_dq(i,j);
      dF(pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X + i,
         pose_twist_meskf::ErrorStateVector::D_ACC_BIAS_X + j) = dv_db(i,j);
      // Orientation:
      dF(pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + i,
         pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + j) = dq_dq(i,j);
      dF(pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + i,
         pose_twist_meskf::ErrorStateVector::D_GYRO_DRIFT_X + j) = dq_dd(i,j);
      // Accelerometers' bias:
      dF(pose_twist_meskf::ErrorStateVector::D_ACC_BIAS_X + i,
         pose_twist_meskf::ErrorStateVector::D_ACC_BIAS_X + j) = db_db(i,j);
      // Gyroscopes' drift:
      dF(pose_twist_meskf::ErrorStateVector::D_GYRO_DRIFT_X + i,
         pose_twist_meskf::ErrorStateVector::D_GYRO_DRIFT_X + j) = dd_dd(i,j);
    }
  }
  return dF;
}

/**
 * @brief Compute the noise covariance.
 * The gyroscopes' output noise and drift noise
 * are supposed to be independent, gaussian, and isotropic.
 * Then, the covariance may be computed according to the second citation.
 * Accelerometers' output noise and bias noise
 * are supposed to be independent, gaussian, and isotropic too.
 * But the covariance is only an approximation due to the system model
 * complexity.
 * @return the noise covariance.
 */
MatrixWrapper::SymmetricMatrix
BFL::AnalyticConditionalGaussianPoseTwistErrorState::CovarianceGet() const
{
  pose_twist_meskf::InputVector input;
  input.fromVector(ConditionalArgumentGet(1));

  const double dt = input.time_incr_;
  const double rate = nominal_state_.ang_vel_.norm();
  const Eigen::Vector3d axis = nominal_state_.ang_vel_.normalized();
  const double angle = dt*rate;

  const double dt2 = pow(dt,2);

  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

  const Eigen::Matrix3d Qpp = 0.5*dt2*ACC_VAR_*I;
  const Eigen::Matrix3d Qvv = dt*ACC_VAR_*I;

  const Eigen::Matrix3d S = skew(nominal_state_.ang_vel_);
  const Eigen::Matrix3d S2 = S*S;

  const double dt3 = pow(dt,3);
  const double dt4 = pow(dt,4);
  const double dt5 = pow(dt,5);
  const double rate3 = pow(rate,3);
  const double rate4 = pow(rate,4);
  const double rate5 = pow(rate,5);
  const double angle2 = pow(angle,2);
  const double angle3 = pow(angle,3);
  const double cos_angle = cos(angle);
  const double sin_angle = sin(angle);

  const Eigen::Matrix3d Qqq = dt*GYRO_VAR_*I
                            + GYRO_DRIFT_VAR_*
                            (
                              ( rate < ANGULAR_RATE_EPSILON )
                              ? ( dt3/3.0*I + dt5/60.0*S2 )
                              : ( dt3/3.0*I + (angle3/3.0+2.0*(sin_angle-angle))/rate5*S2 )
                            );
  const Eigen::Matrix3d Qqd = -GYRO_DRIFT_VAR_*
                            (
                              ( rate < ANGULAR_RATE_EPSILON )
                              ? ( dt2/2.0*I - dt3/6.0*S + dt4/24.0*S2)
                              : ( dt2/2.0*I - (angle-sin_angle)/rate3*S + (angle2/2.0+cos_angle-1)/rate4*S2 )
                            );

  const Eigen::Matrix3d Qbb = dt*ACC_BIAS_VAR_*I;
  const Eigen::Matrix3d Qdd = dt*GYRO_DRIFT_VAR_*I;

  MatrixWrapper::SymmetricMatrix Q(DimensionGet());
  Q = 0.0;

  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3; j++)
    {
      // Position:
      Q(pose_twist_meskf::ErrorStateVector::D_POSITION_X+i,
        pose_twist_meskf::ErrorStateVector::D_POSITION_X+j) = Qpp(i,j);
      // Velocity:
      Q(pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X+i,
        pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X+j) = Qvv(i,j);
      // Orientation:
      Q(pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X+i,
        pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X+j) = Qqq(i,j);
      Q(pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X+i,
        pose_twist_meskf::ErrorStateVector::D_GYRO_DRIFT_X+j) = Qqd(i,j);
      // Accelerometers' bias:
      Q(pose_twist_meskf::ErrorStateVector::D_ACC_BIAS_X+i,
        pose_twist_meskf::ErrorStateVector::D_ACC_BIAS_X+j) = Qbb(i,j);
      // Gyroscopes' drift:
      Q(pose_twist_meskf::ErrorStateVector::D_GYRO_DRIFT_X+i,
        pose_twist_meskf::ErrorStateVector::D_GYRO_DRIFT_X+j) = Qdd(i,j);
      Q(pose_twist_meskf::ErrorStateVector::D_GYRO_DRIFT_X+i,
        pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X+j) = Qqd(j,i);
    }
  }
  return Q;
}
