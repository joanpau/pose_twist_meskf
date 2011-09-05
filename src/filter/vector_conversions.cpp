/**
 * @file
 * @brief Vector conversions for the pose-twist filter (implementation).
 * @author Joan Pau Beltran
 *
 * This file describes the nominal state, error state, input and measurement
 * vectors used by the BFL library. Also provides useful conversions for
 * composing and decomposing such vectors into its diferrent entities.
 */


#include "vector_conversions.h"

/**
 * @brief Compose a BFL vector for the nominal state.
 * @param x vector to compose (should have the nominal state dimension).
 * @param p pose in reference-frame.
 * @param q orientation quaternion in reference-frame.
 * @param v linear velocity in body-fixed frame.
 * @param b accelerometers' bias.
 * @param d gyroscopes' drift.
 * @param w angular velocity in body-fixed frame.
 * @param a linear acceleration in body-fixed frame.
 */
void pose_twist_meskf::composeNominalState(MatrixWrapper::ColumnVector& x,
                                           const Eigen::Vector3d& p,
                                           const Eigen::Quaterniond& q,
                                           const Eigen::Vector3d& v,
                                           const Eigen::Vector3d& b,
                                           const Eigen::Vector3d& d,
                                           const Eigen::Vector3d& w,
                                           const Eigen::Vector3d& a)
{
  x(nominal_state::POSE_X) = p(0);
  x(nominal_state::POSE_Y) = p(1);
  x(nominal_state::POSE_Z) = p(2);
  x(nominal_state::ORIENTATION_X) = q.x();
  x(nominal_state::ORIENTATION_Y) = q.y();
  x(nominal_state::ORIENTATION_Z) = q.z();
  x(nominal_state::ORIENTATION_W) = q.w();
  x(nominal_state::LIN_VEL_X) = v(0);
  x(nominal_state::LIN_VEL_Y) = v(1);
  x(nominal_state::LIN_VEL_Z) = v(2);
  x(nominal_state::ACC_BIAS_X) = b(0);
  x(nominal_state::ACC_BIAS_Y) = b(1);
  x(nominal_state::ACC_BIAS_Z) = b(2);
  x(nominal_state::GYRO_DRIFT_X) = d(0);
  x(nominal_state::GYRO_DRIFT_Y) = d(1);
  x(nominal_state::GYRO_DRIFT_Z) = d(2);
  x(nominal_state::ANG_VEL_X) = w(0);
  x(nominal_state::ANG_VEL_Y) = w(1);
  x(nominal_state::ANG_VEL_Z) = w(2);
  x(nominal_state::LIN_ACC_X) = a(0);
  x(nominal_state::LIN_ACC_Y) = a(1);
  x(nominal_state::LIN_ACC_Z) = a(2);
}

/**
 * @brief Decompose a BFL vector for the nominal state into its entities.
 * @param x vector to decompose.
 * @param p pose in reference-frame.
 * @param q orientation quaternion in reference-frame.
 * @param v linear velocity in body-fixed frame.
 * @param b accelerometers' bias.
 * @param d gyroscopes' drift.
 * @param w angular velocity in body-fixed frame.
 * @param a linear acceleration in body-fixed frame.
 */
void pose_twist_meskf::decomposeNominalState(const MatrixWrapper::ColumnVector& x,
                                             Eigen::Vector3d& p,
                                             Eigen::Quaterniond& q,
                                             Eigen::Vector3d& v,
                                             Eigen::Vector3d& b,
                                             Eigen::Vector3d& d,
                                             Eigen::Vector3d& w,
                                             Eigen::Vector3d& a)
{
  p(0) = x(nominal_state::POSE_X);
  p(1) = x(nominal_state::POSE_Y);
  p(2) = x(nominal_state::POSE_Z);
  q.x() = x(nominal_state::ORIENTATION_X);
  q.y() = x(nominal_state::ORIENTATION_Y);
  q.z() = x(nominal_state::ORIENTATION_Z);
  q.w() = x(nominal_state::ORIENTATION_W);
  v(0) = x(nominal_state::LIN_VEL_X);
  v(1) = x(nominal_state::LIN_VEL_Y);
  v(2) = x(nominal_state::LIN_VEL_Z);
  b(0) = x(nominal_state::ACC_BIAS_X);
  b(1) = x(nominal_state::ACC_BIAS_Y);
  b(2) = x(nominal_state::ACC_BIAS_Z);
  d(0) = x(nominal_state::GYRO_DRIFT_X);
  d(1) = x(nominal_state::GYRO_DRIFT_Y);
  d(2) = x(nominal_state::GYRO_DRIFT_Z);
  w(0) = x(nominal_state::ANG_VEL_X);
  w(1) = x(nominal_state::ANG_VEL_Y);
  w(2) = x(nominal_state::ANG_VEL_Z);
  a(0) = x(nominal_state::LIN_ACC_X);
  a(1) = x(nominal_state::LIN_ACC_Y);
  a(2) = x(nominal_state::LIN_ACC_Z);
}

/**
 * @brief Compose a BFL vector for the error state.
 * @param e vector to compose (should have the error state dimension).
 * @param dp pose error.
 * @param dq orientation error.
 * @param dv linear velocity error.
 * @param db accelerometers' bias error.
 * @param dd gyroscopes' drift error.
 */
void pose_twist_meskf::composeErrorState(MatrixWrapper::ColumnVector& e,
                                         const Eigen::Vector3d& dp,
                                         const Eigen::Vector3d& dq,
                                         const Eigen::Vector3d& dv,
                                         const Eigen::Vector3d& db,
                                         const Eigen::Vector3d& dd)
{
  e(error_state::D_POSE_X) = dp(0);
  e(error_state::D_POSE_Y) = dp(1);
  e(error_state::D_POSE_Z) = dp(2);
  e(error_state::D_ORIENTATION_X) = dq(0);
  e(error_state::D_ORIENTATION_Y) = dq(1);
  e(error_state::D_ORIENTATION_Z) = dq(2);
  e(error_state::D_LIN_VEL_X) = dv(0);
  e(error_state::D_LIN_VEL_Y) = dv(1);
  e(error_state::D_LIN_VEL_Z) = dv(2);
  e(error_state::D_ACC_BIAS_X) = db(0);
  e(error_state::D_ACC_BIAS_Y) = db(1);
  e(error_state::D_ACC_BIAS_Z) = db(2);
  e(error_state::D_GYRO_DRIFT_X) = dd(0);
  e(error_state::D_GYRO_DRIFT_Y) = dd(1);
  e(error_state::D_GYRO_DRIFT_Z) = dd(2);
}

/**
 * @brief Decompose a BFL vector for the error state into its entities.
 * @param e vector to decompose.
 * @param dp pose error.
 * @param dq orientation error.
 * @param dv linear velocity error.
 * @param db accelerometers' bias error.
 * @param dd gyroscopes' drift error.
 */
void pose_twist_meskf::decomposeErrorState(const MatrixWrapper::ColumnVector& e,
                                           Eigen::Vector3d& dp,
                                           Eigen::Vector3d& dq,
                                           Eigen::Vector3d& dv,
                                           Eigen::Vector3d& db,
                                           Eigen::Vector3d& dd)
{
  dp(0) = e(error_state::D_POSE_X);
  dp(1) = e(error_state::D_POSE_Y);
  dp(2) = e(error_state::D_POSE_Z);
  dq(0) = e(error_state::D_ORIENTATION_X);
  dq(1) = e(error_state::D_ORIENTATION_Y);
  dq(2) = e(error_state::D_ORIENTATION_Z);
  dv(0) = e(error_state::D_LIN_VEL_X);
  dv(1) = e(error_state::D_LIN_VEL_Y);
  dv(2) = e(error_state::D_LIN_VEL_Z);
  db(0) = e(error_state::D_ACC_BIAS_X);
  db(1) = e(error_state::D_ACC_BIAS_Y);
  db(2) = e(error_state::D_ACC_BIAS_Z);
  dd(0) = e(error_state::D_GYRO_DRIFT_X);
  dd(1) = e(error_state::D_GYRO_DRIFT_Y);
  dd(2) = e(error_state::D_GYRO_DRIFT_Z);
}


/**
 * @brief Compose a BFL vector for the input.
 * @param u vector to compose.
 * @param w angular velocity (sensed).
 * @param a linear acceleration (sensed).
 * @param dt time step.
 */
void pose_twist_meskf::composeInput(MatrixWrapper::ColumnVector& u,
                                    const Eigen::Vector3d& w,
                                    const Eigen::Vector3d& a,
                                    const double& dt)
{
  u(input::ANG_VEL_X) = w(0);
  u(input::ANG_VEL_Y) = w(1);
  u(input::ANG_VEL_Z) = w(2);
  u(input::LIN_ACC_X) = a(0);
  u(input::LIN_ACC_Y) = a(1);
  u(input::LIN_ACC_Z) = a(2);
  u(input::TIME_STEP) = dt;
}


void pose_twist_meskf::decomposeInput(const MatrixWrapper::ColumnVector& u,
                                      Eigen::Vector3d& w,
                                      Eigen::Vector3d& a,
                                      double& dt)
{
  w(0) = u(input::ANG_VEL_X);
  w(1) = u(input::ANG_VEL_Y);
  w(2) = u(input::ANG_VEL_Z);
  a(0) = u(input::LIN_ACC_X);
  a(1) = u(input::LIN_ACC_Y);
  a(2) = u(input::LIN_ACC_Z);
  dt = u(input::TIME_STEP);
}
