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
 * only with the error. The problem is how to implement the conditional
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
 */

#ifndef ANALYTICCONDITIONALGAUSSIAN_POSETWISTERRORSTATE_H
#define ANALYTICCONDITIONALGAUSSIAN_POSETWISTERRORSTATE_H

#include <pdf/analyticconditionalgaussian.h>
#include <Eigen/Dense>
#include "nominal_state_vector.h"

namespace BFL
{

class AnalyticConditionalGaussianPoseTwistErrorState
: public BFL::AnalyticConditionalGaussian
{
public:
  // Default constructor specifying system noise.
  AnalyticConditionalGaussianPoseTwistErrorState(const double& acc_var,
                                                 const double& gyro_var,
                                                 const double& acc_bias_var,
                                                 const double& gyro_drift_var);

  // Error state filtering specific functions.
  void NominalStateSet(const MatrixWrapper::ColumnVector& x);
  MatrixWrapper::ColumnVector NominalStateGet();
  void CorrectNominalState(const MatrixWrapper::ColumnVector& e);

  // Redefine virtual functions.
  virtual MatrixWrapper::ColumnVector ExpectedValueGet() const;
  virtual MatrixWrapper::Matrix dfGet(unsigned int i) const;
  virtual MatrixWrapper::SymmetricMatrix CovarianceGet() const;

  virtual ~AnalyticConditionalGaussianPoseTwistErrorState();

private:

  // Nominal state
  mutable pose_twist_meskf::NominalStateVector nominal_state_;

  // Error state not needed because it is stored in the filter pdf.
  // In addition, the error state is always zero since it is only modified
  // by a measurement update, and after that the reset step transfers
  // its values to the nominal state and sets it to zero again.

  // Gravity constant vector in reference frame
  static const double G_CONS;  //!< standard gravity scalar constant.
  static const Eigen::Vector3d G_VECT; //!< standard gravity vector.

  static const double ANGULAR_RATE_EPSILON;  //!< tolerance for small angle approximations.

  // Sensor variances
  const double ACC_VAR_;        //!< Accelerometer variance.
  const double GYRO_VAR_;       //!< Gyroscope variance.
  const double ACC_BIAS_VAR_;   //!< Accelerometer variance.
  const double GYRO_DRIFT_VAR_; //!< Gyroscope variance.

  // Auxiliar functions:
  Eigen::Matrix3d skew(const Eigen::Vector3d& v) const;

};

} // namespace

#endif // ANALYTICCONDITIONALGAUSSIAN_POSETWISTERRORSTATE_H
