/** @file
 *
 * @brief Pose twist error-state extended Kalman filter
 * with multiplicative orientation error.
 *
 * The filter is implemented using the Bayesian Filter Library.
 */

#ifndef POSE_TWIST_MESKF_H
#define POSE_TWIST_MESKF_H

#include <queue>
#include <filter/extendedkalmanfilter.h>
#include <wrappers/matrix/matrix_wrapper.h>
#include <wrappers/matrix/vector_wrapper.h>
#include <model/analyticsystemmodel_gaussianuncertainty.h>
#include <model/analyticmeasurementmodel_gaussianuncertainty.h>
#include "analyticconditionalgaussian_posetwisterrorstate.h"

namespace pose_twist_meskf
{

class PoseTwistMESKF
{
public:
  typedef MatrixWrapper::ColumnVector Vector;
  typedef MatrixWrapper::SymmetricMatrix SymmetricMatrix;
  typedef double TimeStamp;
  typedef int MeasurementIndex;

  PoseTwistMESKF();
  void setUpSystem(const Vector& noise_mean, const SymmetricMatrix& noise_cov);

  void initialize(const Vector& x, const SymmetricMatrix& P, const TimeStamp& t);

  bool addInput(const TimeStamp& t, const Vector& u);
  bool addMeasurement(const MeasurementIndex& i, const TimeStamp& t,
                      const Vector& z, const SymmetricMatrix Q);
  void update();

  TimeStamp getTimeStamp() const;
  SymmetricMatrix getCovariance() const;
  Vector getEstimate() const;
  void getEstimation(Vector& x, SymmetricMatrix& P, TimeStamp& t) const;

private:

  struct Measurement
  {
    TimeStamp t_;
    Vector z_;
    SymmetricMatrix Q_;
    Measurement(const TimeStamp& t, const Vector& z, const SymmetricMatrix& Q)
    :  t_(t), z_(z), Q_(Q)
    {}
    bool operator<(const Measurement& r) const {return t_ > r.t_;}
  };

  struct Input
  {
    TimeStamp t_;
    Vector u_;
    Input(const TimeStamp& t, const Vector& u)
    :  t_(t), u_(u)
    {}
    bool operator<(const Measurement& r) const {return t_ > r.t_;}
  };

  BFL::ExtendedKalmanFilter*                           filter_;
  BFL::AnalyticSystemModelGaussianUncertainty*         system_model_;
  BFL::AnalyticConditionalGaussianPoseTwistErrorState* system_pdf_;
  BFL::Gaussian*                                       system_prior_;
  std::priority_queue<Input>                           input_queue_;

  std::vector< BFL::AnalyticConditionalGaussianAdditiveNoise* >    measurement_pdfs_;
  std::vector< BFL::AnalyticMeasurementModelGaussianUncertainty* > measurement_models_;
  std::vector< std::priority_queue<Measurement> >                  measurement_queues_;

  Vector current_input_;
  TimeStamp current_time_;

  bool updateFilterSys(const Input& u);
  bool updateFilterMeas(const MeasurementIndex& i, const Measurement& m);
};

} // namespace

#endif // POSE_TWIST_MESKF_H
