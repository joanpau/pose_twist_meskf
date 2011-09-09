/**
 * @file
 * @brief Visual measurement vector for the pose-twist filter (presentation).
 * @author Joan Pau Beltran
 *
 * This file describes the visual measurement vector for the filter,
 * and provides conversions to the vector format used by the BFL library.
 */

#ifndef VISUAL_MEASUREMENT_VECTOR_H
#define VISUAL_MEASUREMENT_VECTOR_H

#include <wrappers/matrix/vector_wrapper.h> // BFL vector class
#include <Eigen/Geometry> // Eigen vector and quaternion classes

namespace pose_twist_meskf
{

/**
 * @brief Visual measurement vector.
 */
struct VisualMeasurementVector
{
  void fromVector(const MatrixWrapper::ColumnVector& z);
  void toVector(MatrixWrapper::ColumnVector& z) const;

  Eigen::Vector3d position_;       //!< Position estimate.
  Eigen::Vector3d lin_vel_;        //!< Linear velocity estimate.
  Eigen::Quaterniond orientation_; //!< Orientation estimate.

  // Index for the input vector
  static const int DIMENSION = 10; //!< Visual measurement vector dimension.
  enum Index
  {
    POSITION_X = 1,
    POSITION_Y,
    POSITION_Z,
    LIN_VEL_X,
    LIN_VEL_Y,
    LIN_VEL_Z,
    ORIENTATION_X,
    ORIENTATION_Y,
    ORIENTATION_Z,
    ORIENTATION_W
  }; // enum

}; // struct

} // namespace

#endif // VISUAL_MEASUREMEMT_VECTOR_H
