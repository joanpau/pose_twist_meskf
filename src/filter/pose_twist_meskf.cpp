/** @file
 * @brief Pose twist error-state extended Kalman filter
 * with multiplicative orientation error.
 * @author Joan Pau Beltran
 *
 * The filter is implemented using the Bayesian Filter Library.
 */

#include "pose_twist_meskf.h"

/**
 * @brief Default constructor doing nothing.
 *
 * After creating an object of the class,
 * the filter should be set up with the setUpFilter() function.
 *
 * @return
 */
pose_twist_meskf::PoseTwistMESKF::PoseTwistMESKF()
{}


/**
 * @brief Destructor.
 *
 * Free dynamically allocated members if needed.
 *
 * @return
 */
pose_twist_meskf::PoseTwistMESKF::~PoseTwistMESKF()
{
  if (filter_)
    delete filter_;
  if (system_model_)
    delete system_model_;
  if(system_pdf_)
    delete system_pdf_;
  if(system_prior_)
    delete system_prior_;
  for (int i = measurement_models_.size(); i>=0; i--)
    if(measurement_models_[i])
      delete measurement_models_[i];
  for (int i = measurement_pdfs_.size(); i>=0; i--)
    if(measurement_pdfs_[i])
      delete measurement_pdfs_[i];
}

void pose_twist_meskf::PoseTwistMESKF::setUpSystem(const double& acc_var,
                                                   const double& gyro_var,
                                                   const double& acc_bias_var,
                                                   const double& gyro_drift_var)
{
  system_pdf_ = new BFL::AnalyticConditionalGaussianPoseTwistErrorState(acc_var,
                                                                        gyro_var,
                                                                        acc_bias_var,
                                                                        gyro_drift_var);
  system_model_ = new BFL::AnalyticSystemModelGaussianUncertainty(system_pdf_);
}


/**
 * @brief Initialize the filter with the nominal state and the error-state covariance.
 *
 * Note that the error-state is implicitly initialized to zero.
 *
 * @param x initial nominal state value.
 * @param P error-state initial covariance.
 * @param t initial time.
 */
void pose_twist_meskf::PoseTwistMESKF::initialize(const Vector& x,
                                                  const SymmetricMatrix& P,
                                                  const TimeStamp& t)
{

  int dim = system_pdf_->DimensionGet();
  Vector prior_mean(dim);
  SymmetricMatrix prior_cov(dim);
  prior_mean = 0;
  prior_cov = P;
  system_prior_ = new BFL::Gaussian(prior_mean, prior_cov);
  filter_ = new BFL::ExtendedKalmanFilterResetCapable(system_prior_);
  system_pdf_->NominalStateSet(x);
  current_time_ = t;
}

/**
 * @brief Add an input reading to the filter queue.
 *
 * The given input must be more recent than the current filter time,
 * otherwise is dropped.
 *
 * @param t input time.
 * @param u input value.
 * @return whether the input has been added to the queue.
 */
bool pose_twist_meskf::PoseTwistMESKF::addInput(const TimeStamp& t,
                                                const Vector& u)
{
  if (t<current_time_)
    return false;
  input_queue_.push(Input(t,u));
  return true;
}


/**
 * @brief Add a measurement reading to the filter queue.
 *
 * The given measurement must be more recent than the current filter time,
 * otherwise is dropped.
 *
 * @param t measurement time.
 * @param z measurement value.
 * @param Q measurement covariance.
 * @return whether the input has been added to the queue.
 */

bool pose_twist_meskf::PoseTwistMESKF::addMeasurement(const MeasurementIndex& i,
                                                      const TimeStamp& t,
                                                      const Vector& z,
                                                      const SymmetricMatrix Q)
{
  if (t<current_time_)
    return false;
  measurement_queues_[i].push(Measurement(t,z,Q));
  return true;
}


/**
 * @brief Update the filter until some measurement queue is empty.
 *
 * While there is a complete set of measurement updates to perform
 * (i.e. from each sensor there is at least one measurement pending to process),
 * all inputs and measurements are processed in order as follows:
 *   - process all inputs previous to the next measurement.
 *   - since the inputs and the measurements are not synchronized,
 *     update the system with the last input until the next measurement time.
 *   - correct the system state with the measurement and repeat.
 *
 */
void pose_twist_meskf::PoseTwistMESKF::update()
{
  bool pending = true;
  do
  {
    int qnext, q;
    for (qnext=q=measurement_queues_.size()-1; q>=0; q--)
      if (measurement_queues_[q].empty())
        pending = false;
      else
        if( (measurement_queues_[q].top().t_) < (measurement_queues_[qnext].top().t_) )
          qnext = q;
    if(pending)
    {
      const Measurement& m = measurement_queues_[qnext].top();
      while( (!input_queue_.empty()) && (input_queue_.top().t_<=m.t_) )
      {
        updateFilterSys(input_queue_.top());
        input_queue_.pop();
      }
      if(current_time_<m.t_)
        updateFilterSys(Input(m.t_,current_input_));
      updateFilterMeas(qnext, m);
      measurement_queues_[qnext].pop();
    }
  } while (pending);
}

/**
 * @brief Update nominal and error state systems with given input.
 *
 * The update is done inside the filter,
 * which updates the error state conditional pdf.
 * The nominal state is updated implicitly during this process
 * (in the ExpectedValueGet function).
 * The current time is set to the input time.
 * The current input is updated too.
 * @param u input.
 * @return whether the filter updated the system (currently always true)
 */
bool pose_twist_meskf::PoseTwistMESKF::updateFilterSys(const Input& u)
{
  TimeStamp input_t = u.t_;
  Vector input_val = u.u_;
  int input_dim = u.u_.rows();
  Vector in(input_dim+1);
  for (int i=0; i<input_dim; i++)
    in(i)=input_val(i);
  in(input_dim) = input_t - current_time_;
  bool success = filter_->Update(system_model_,in);
  current_time_ = input_t;
  current_input_ = input_val;
  return success;
}


/**
 * @brief Correct nominal and error with given measurement.
 *
 * The error state correction given by the measurement is done by the filter.
 * The nominal state is corrected resetting the error state conditional pdf.
 * The filter time is supposed to be the same than the measurement time.
 * If not the system is not updated.
 *
 * @param i index of the measurement.
 * @param m measurement.
 * @return whether the filter corrected the system.
 */
bool pose_twist_meskf::PoseTwistMESKF::updateFilterMeas(const MeasurementIndex& i,
                                                        const Measurement& m)
{
  TimeStamp measurement_t = m.t_;
  Vector measurement_val = m.z_;
  SymmetricMatrix measurement_cov = m.Q_;
  bool success = false;
  if (measurement_t == current_time_)
  {
    measurement_pdfs_[i]->AdditiveNoiseSigmaSet(measurement_cov);
    success = filter_->Update(measurement_models_[i],measurement_val);
  }
  if (success)
  {
    system_pdf_->CorrectNominalState(filter_->PostGet()->ExpectedValueGet());
    filter_->PostMuSet(MatrixWrapper::ColumnVector(system_pdf_->DimensionGet(),0.0));
  }
  return success;
}


/**
 * @brief Get the the current filter time.
 * @return the time stamp of the last processed input.
 */
pose_twist_meskf::PoseTwistMESKF::TimeStamp
pose_twist_meskf::PoseTwistMESKF::getTimeStamp() const
{
  return current_time_;
};


/**
 * @brief Get the covariance of the current error state.
 * @return current error state covariance matrix.
 */
pose_twist_meskf::PoseTwistMESKF::SymmetricMatrix
pose_twist_meskf::PoseTwistMESKF::getCovariance() const
{
  return filter_->PostGet()->CovarianceGet();
}


/**
 * Get the current state estimate.
 * @return current nominal state vector.
 */
pose_twist_meskf::PoseTwistMESKF::Vector
pose_twist_meskf::PoseTwistMESKF::getEstimate() const
{
  return system_pdf_->NominalStateGet();
}


/**
 * @brief Get estimation with time stamp and covariance.
 * @param x vector to store the current state.
 * @param P matrix to store the current covariance.
 * @param t current filter time.
 */
void pose_twist_meskf::PoseTwistMESKF::getEstimate(Vector& x,
                                                   SymmetricMatrix& P,
                                                   TimeStamp& t) const
{
  x = getEstimate();
  P = getCovariance();
  t = getTimeStamp();
}
