/**
 * @file
 * @brief Visual measurement vector for the pose-twist filter (implementation).
 * @author Joan Pau Beltran
 *
 * This file describes the visual measurement vector for the filter,
 * and provides conversions to the vector format used by the BFL library.
 */

#include "visual_measurement_vector.h"

/**
 * @brief Read the visual measurement entities from a BFL vector.
 * @param z vector to decompose.
 */
void pose_twist_meskf::VisualMeasurementVector::fromVector(const MatrixWrapper::ColumnVector& z)
{
  position_(0) = z(POSITION_X);
  position_(1) = z(POSITION_Y);
  position_(2) = z(POSITION_Z);
  lin_vel_(0) = z(LIN_VEL_X);
  lin_vel_(1) = z(LIN_VEL_Y);
  lin_vel_(2) = z(LIN_VEL_Z);
  orientation_.x() = z(ORIENTATION_X);
  orientation_.y() = z(ORIENTATION_Y);
  orientation_.z() = z(ORIENTATION_Z);
  orientation_.w() = z(ORIENTATION_W);
}


/**
 * @brief Write the visual measurement entities to a BFL vector.
 * @param z vector to compose (should have the correct dimension).
 */
void pose_twist_meskf::VisualMeasurementVector::toVector(MatrixWrapper::ColumnVector& z) const
{
  z(POSITION_X) = position_(0);
  z(POSITION_Y) = position_(1);
  z(POSITION_Z) = position_(2);
  z(LIN_VEL_X) = lin_vel_(0);
  z(LIN_VEL_Y) = lin_vel_(1);
  z(LIN_VEL_Z) = lin_vel_(2);
  z(ORIENTATION_X) = orientation_.x();
  z(ORIENTATION_Y) = orientation_.y();
  z(ORIENTATION_Z) = orientation_.z();
  z(ORIENTATION_W) = orientation_.w();
}
