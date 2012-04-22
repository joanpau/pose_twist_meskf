/**
 * @file
 * @author Joan Pau Beltran
 * @brief Linear Conditional Gaussian for measurement from visual sensors (implementation).
 */

#include "linearanalyticconditionalgaussian_visualmeasurement.h"
#include "nominal_state_vector.h"
#include "error_state_vector.h"
#include "visual_measurement_vector.h"
#include "visual_measurement_error_vector.h"


/**
 * @brief Compute error measurements between visual measurements and nominal state.
 * @param z visual measurement vector.
 * @param x nominal state vector.
 * @return measurement error vector.
 */
MatrixWrapper::ColumnVector
BFL::LinearAnalyticConditionalGaussianVisualMeasurement::ErrorMeasurement(
  const MatrixWrapper::ColumnVector& z,
  const MatrixWrapper::ColumnVector& x) const
{
  pose_twist_meskf::VisualMeasurementVector measurement;
  measurement.fromVector(z);
  pose_twist_meskf::NominalStateVector nominal_state;
  nominal_state.fromVector(x);
  pose_twist_meskf::VisualMeasurementErrorVector error;
  error.d_position_ = measurement.position_ - nominal_state.position_;
  Eigen::AngleAxisd aa(nominal_state.orientation_.inverse() * measurement.orientation_);
  error.d_orientation_ = aa.angle() * aa.axis();
  error.d_lin_vel_ = measurement.lin_vel_ - nominal_state.lin_vel_;
  MatrixWrapper::ColumnVector e(pose_twist_meskf::ErrorStateVector::DIMENSION);
  error.toVector(e);
  return e;
}

const MatrixWrapper::Matrix
BFL::LinearAnalyticConditionalGaussianVisualMeasurement::initH()
{
  MatrixWrapper::Matrix H = MatrixWrapper::Matrix(pose_twist_meskf::VisualMeasurementErrorVector::DIMENSION,
                                                  pose_twist_meskf::ErrorStateVector::DIMENSION);
  H = 0.0;
  for (int i=0; i<3; i++)
  {
    // Position:
    H(pose_twist_meskf::VisualMeasurementErrorVector::D_POSITION_X + i,
      pose_twist_meskf::ErrorStateVector::D_POSITION_X + i) = 1.0;
    // Velocity:
    H(pose_twist_meskf::VisualMeasurementErrorVector::D_LIN_VEL_X + i,
      pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X + i) = 1.0;
    // Orientation.
    H(pose_twist_meskf::VisualMeasurementErrorVector::D_ORIENTATION_X + i,
      pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + i) = 1.0;
  }
  return H;
};

const MatrixWrapper::Matrix
BFL::LinearAnalyticConditionalGaussianVisualMeasurement::
H = BFL::LinearAnalyticConditionalGaussianVisualMeasurement::initH();

/**
 * @brief Constructor setting the measurement noise.
 * @param additiveNoise pdf representing the additive Gaussian uncertainty.
 * @return
 */
BFL::LinearAnalyticConditionalGaussianVisualMeasurement::
LinearAnalyticConditionalGaussianVisualMeasurement(const Gaussian& additiveNoise)
: LinearAnalyticConditionalGaussianErrorMeasurement(H,additiveNoise)
{}


/**
 * @brief Destructor (doing nothing).
 * @return
 */
BFL::LinearAnalyticConditionalGaussianVisualMeasurement::
~LinearAnalyticConditionalGaussianVisualMeasurement()
{}


/**
 * @brief Clone function.
 * @return pointer to a new object that is an exact copy of this.
 */
BFL::LinearAnalyticConditionalGaussianVisualMeasurement*
BFL::LinearAnalyticConditionalGaussianVisualMeasurement::Clone() const
{
  return new LinearAnalyticConditionalGaussianVisualMeasurement(*this);
}

