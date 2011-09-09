/**
 * @file
 * @brief Visual measurement error vector for the pose-twist filter (implementation).
 * @author Joan Pau Beltran
 *
 * This file describes the visual measurement error vector for the filter,
 * and provides conversions to the vector format used by the BFL library.
 */

#include "visual_measurement_error_vector.h"

/**
 * @brief Read the visual measurement error entities from a BFL vector.
 * @param z vector to decompose.
 */
void pose_twist_meskf::VisualMeasurementErrorVector::fromVector(const MatrixWrapper::ColumnVector& z)
{
  d_position_(0) = z(D_POSITION_X);
  d_position_(1) = z(D_POSITION_Y);
  d_position_(2) = z(D_POSITION_Z);
  d_lin_vel_(0) = z(D_LIN_VEL_X);
  d_lin_vel_(1) = z(D_LIN_VEL_Y);
  d_lin_vel_(2) = z(D_LIN_VEL_Z);
  d_orientation_.x() = z(D_ORIENTATION_X);
  d_orientation_.y() = z(D_ORIENTATION_Y);
  d_orientation_.z() = z(D_ORIENTATION_Z);
}


/**
 * @brief Write the visual measurement error entities to a BFL vector.
 * @param z vector to compose (should have the correct dimension).
 */
void pose_twist_meskf::VisualMeasurementErrorVector::toVector(MatrixWrapper::ColumnVector& z) const
{
  z(D_POSITION_X) = d_position_(0);
  z(D_POSITION_Y) = d_position_(1);
  z(D_POSITION_Z) = d_position_(2);
  z(D_LIN_VEL_X) = d_lin_vel_(0);
  z(D_LIN_VEL_Y) = d_lin_vel_(1);
  z(D_LIN_VEL_Z) = d_lin_vel_(2);
  z(D_ORIENTATION_X) = d_orientation_.x();
  z(D_ORIENTATION_Y) = d_orientation_.y();
  z(D_ORIENTATION_Z) = d_orientation_.z();
}
