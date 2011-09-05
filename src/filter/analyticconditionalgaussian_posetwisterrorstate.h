/**
 * @file analyticconditionalgaussian_posetwisterrorstate.h
 * @brief Conditional distribution for pose-twist error state (presentation)
 * @author Joan Pau Beltran
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

#ifndef ERRORSTATEANALYTICCONDITIONALGAUSSIANPOSETWIST_H
#define ERRORSTATEANALYTICCONDITIONALGAUSSIANPOSETWIST_H

#include <pdf/analyticconditionalgaussian_additivenoise.h>
#include <Eigen/Dense>

namespace BFL
{

class AnalyticConditionalGaussianPoseTwistErrorState
: public AnalyticConditionalGaussian
{
public:
  // Default constructor specifying system noise.
  AnalyticConditionalGaussianPoseTwistErrorState(const double& acc_var,
                                                 const double& gyro_var);

  // Error state filtering specific functions.
  void NominalStateSet(const MatrixWrapper::ColumnVector& x);
  MatrixWrapper::ColumnVector NominalStateGet();
  void ErrorStateSet(const MatrixWrapper::ColumnVector& x);
  MatrixWrapper::ColumnVector ErrorStateGet();
  void ResetErrorState();

  // redefine virtual functions.
  virtual MatrixWrapper::ColumnVector ExpectedValueGet() const;
  virtual MatrixWrapper::Matrix dfGet(unsigned int i) const;

  virtual ~AnalyticConditionalGaussianPoseTwistErrorState();

private:
  // Nominal state
  mutable Eigen::Vector3d pose_;           //!< pose in reference frame.
  mutable Eigen::Quaterniond orientation_; //!< orientation in reference frame.
  mutable Eigen::Vector3d lin_vel_;        //!< linear velocity in body-fixed frame.
  mutable Eigen::Vector3d acc_bias_;       //!< accelerometers' bias.
  mutable Eigen::Vector3d gyro_drift_;     //!< gyroscopes' drift.
  mutable Eigen::Vector3d ang_vel_;        //!< angular velocity in body-fixed frame.
  mutable Eigen::Vector3d lin_acc_;        //!< linear acceleration in body-fixed frame.

  // Error state
  mutable Eigen::Vector3d d_pose_;           //!< pose in reference frame.
  mutable Eigen::Quaterniond d_orientation_; //!< orientation in reference frame.
  mutable Eigen::Vector3d d_lin_vel_;        //!< linear velocity in body-fixed frame.
  mutable Eigen::Vector3d d_acc_bias_;       //!< accelerometers' bias.
  mutable Eigen::Vector3d d_gyro_drift_;     //!< gyroscopes' drift.

  // Gravity constant vector in reference frame
  static const double G_CONS;  //!< standard gravity scalar constant.
  static const Eigen::Vector3d G_VECT; //!< standard gravity vector.

  static const double ANGULAR_RATE_EPSILON;  //!< tolerance for small angle approximations.

  // Auxiliar functions:
  Eigen::Matrix3d skew(const Eigen::Vector3d& v) const;

};

} // namespace

#endif // ANALYTICCONDITIONALGAUSSIANPOSETWISTERRORSTATE_H
