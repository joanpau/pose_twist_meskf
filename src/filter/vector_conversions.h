/**
 * @file
 * @brief Vector conversions for the pose-twist filter (presentation).
 * @author Joan Pau Beltran
 *
 * This file describes the nominal state, error state, input and measurement
 * vectors used by the BFL library. Also provides useful conversions for
 * composing and decomposing such vectors into its diferrent entities.
 */

#ifndef VECTOR_CONVERSIONS_H
#define VECTOR_CONVERSIONS_H

#include <wrappers/matrix/vector_wrapper.h> // BFL vector class
#include <Eigen/Core> // Eigen vector and quaternion classes

namespace pose_twist_meskf
{

void composeNominalState(MatrixWrapper::ColumnVector& x,
                         const Eigen::Vector3d& p,
                         const Eigen::Quaterniond& q,
                         const Eigen::Vector3d& v,
                         const Eigen::Vector3d& b,
                         const Eigen::Vector3d& d,
                         const Eigen::Vector3d& w,
                         const Eigen::Vector3d& a);

void decomposeNominalState(const MatrixWrapper::ColumnVector& x,
                           Eigen::Vector3d& p,
                           Eigen::Quaterniond& q,
                           Eigen::Vector3d& v,
                           Eigen::Vector3d& b,
                           Eigen::Vector3d& d,
                           Eigen::Vector3d& w,
                           Eigen::Vector3d& a);

void composeErrorState(MatrixWrapper::ColumnVector& e,
                       const Eigen::Vector3d& dp,
                       const Eigen::Vector3d& dq,
                       const Eigen::Vector3d& dv,
                       const Eigen::Vector3d& db,
                       const Eigen::Vector3d& dd);

void decomposeErrorState(const MatrixWrapper::ColumnVector& e,
                         Eigen::Vector3d& dp,
                         Eigen::Vector3d& dq,
                         Eigen::Vector3d& dv,
                         Eigen::Vector3d& db,
                         Eigen::Vector3d& dd);

void composeInput(MatrixWrapper::ColumnVector& u,
                  const Eigen::Vector3d& w,
                  const Eigen::Vector3d& a,
                  const double& dt);

void decomposeInput(const MatrixWrapper::ColumnVector& u,
                    Eigen::Vector3d& w,
                    Eigen::Vector3d& a,
                    double& dt);


// Index for the nominal state vector
namespace nominal_state
{
enum NominalStateIndex
{
  POSE_X = 1,
  POSE_Y,
  POSE_Z,
  ORIENTATION_X,
  ORIENTATION_Y,
  ORIENTATION_Z,
  ORIENTATION_W,
  LIN_VEL_X,
  LIN_VEL_Y,
  LIN_VEL_Z,
  ACC_BIAS_X,
  ACC_BIAS_Y,
  ACC_BIAS_Z,
  GYRO_DRIFT_X,
  GYRO_DRIFT_Y,
  GYRO_DRIFT_Z,
  ANG_VEL_X,
  ANG_VEL_Y,
  ANG_VEL_Z,
  LIN_ACC_X,
  LIN_ACC_Y,
  LIN_ACC_Z
};  // enum
}  // namespace

// Nominal state vector dimension
const int NOMINAL_STATE_DIM = 22;


// Index for the error state vector
namespace error_state
{
enum ErrorStateIndex
{
  D_POSE_X = 1,
  D_POSE_Y,
  D_POSE_Z,
  D_ORIENTATION_X,
  D_ORIENTATION_Y,
  D_ORIENTATION_Z,
  D_LIN_VEL_X,
  D_LIN_VEL_Y,
  D_LIN_VEL_Z,
  D_ACC_BIAS_X,
  D_ACC_BIAS_Y,
  D_ACC_BIAS_Z,
  D_GYRO_DRIFT_X,
  D_GYRO_DRIFT_Y,
  D_GYRO_DRIFT_Z
}; // enum
} // namespace

// Error state vector dimension
const int ERROR_STATE_DIM = 15;


// Index for the input vector
namespace input
{
enum InputIndex
{
  ANG_VEL_X = 1,
  ANG_VEL_Y,
  ANG_VEL_Z,
  LIN_ACC_X,
  LIN_ACC_Y,
  LIN_ACC_Z,
  TIME_STEP
}; // enum
} // namespace

// Input vector dimension
const int INPUT_DIM = 7;

}

#endif // VECTOR_CONVERSIONS_H
