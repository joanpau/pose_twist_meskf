/**
 * @file
 * @brief Input state vector for the pose-twist filter (presentation).
 * @author Joan Pau Beltran
 *
 * This file describes the input vector for the filter,
 * and provides conversions to the vector format used by the BFL library.
 */

#ifndef INPUT_VECTOR_H
#define INPUT_VECTOR_H

#include <wrappers/matrix/vector_wrapper.h> // BFL vector class
#include <Eigen/Core> // Eigen vector and quaternion classes

namespace pose_twist_meskf
{

/**
 * @brief Input vector.
 */
struct InputVector
{
  void fromVector(const MatrixWrapper::ColumnVector& u);
  void toVector(MatrixWrapper::ColumnVector& u) const;

  Eigen::Vector3d ang_vel_; //!< Gyroscopes' reading.
  Eigen::Vector3d lin_acc_; //!< Accelerometers' reading.
  double time_step_;        //!< Elapsed time since previous input.

  // Index for the input vector
  static const int DIMENSION = 7; //!< Input vector dimension.
  enum Index
  {
    ANG_VEL_X = 1,
    ANG_VEL_Y,
    ANG_VEL_Z,
    LIN_ACC_X,
    LIN_ACC_Y,
    LIN_ACC_Z,
    TIME_STEP
  }; // enum

}; // struct

} // namespace

#endif // INPUT_VECTOR_H
