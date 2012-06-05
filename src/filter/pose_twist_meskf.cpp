/** @file
 * @brief Pose twist error state extended Kalman filter
 * with multiplicative orientation error.
 * @author Joan Pau Beltran
 *
 * The filter is implemented using the Bayesian Filter Library.
 */

#include "pose_twist_meskf.h"
#include "analyticconditionalgaussian_visualmeasurement.h"
#include "model/linearanalyticmeasurementmodel_gaussianuncertainty.h"
#include "visual_measurement_error_vector.h"

/**
 * @brief Default constructor doing nothing.
 *
 * After creating an object of the class,
 * the filter should be set up with the setUpFilter() function.
 *
 * @return
 */
pose_twist_meskf::PoseTwistMESKF::PoseTwistMESKF()
: filter_(0), system_model_(0), system_pdf_(0), system_prior_(0)
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
  if (system_model_)
    delete system_model_;
  if(system_pdf_)
    delete system_pdf_;
  if(system_prior_)
    delete system_prior_;
  if (filter_)
    delete filter_;
  for (int i = measurement_models_.size()-1; i>=0; i--)
    if(measurement_models_[i])
      delete measurement_models_[i];
  for (int i = measurement_pdfs_.size()-1; i>=0; i--)
    if(measurement_pdfs_[i])
      delete measurement_pdfs_[i];
}


/**
 * @brief Get the the current filter time.
 * @return the time stamp of the last processed input.
 */
pose_twist_meskf::PoseTwistMESKF::TimeStamp
pose_twist_meskf::PoseTwistMESKF::getFilterTime() const
{
  return filter_time_;
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
  t = getFilterTime();
}


/**
 * @brief Set up the system model used by the filter and its pdf.
 * @param acc_var accelerometer variance (same for all axes).
 * @param gyro_var gyroscope variance (same for all axes).
 * @param acc_bias_var accelerometer bias variance (same for all axes).
 * @param gyro_drift_var gyroscope bias variance (same for all axes).
 */
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
 * @brief Set up the measurement models used by the filter and its pdf's.
 */
void pose_twist_meskf::PoseTwistMESKF::setUpMeasurementModels()
{
  // Set up vectors
  measurement_pdfs_.resize(NUM_MEASUREMENT_TYPES);
  measurement_models_.resize(NUM_MEASUREMENT_TYPES);
  measurement_queues_.resize(NUM_MEASUREMENT_TYPES);

  BFL::AnalyticConditionalGaussianVisualMeasurement* visual_meas_pdf =
      new BFL::AnalyticConditionalGaussianVisualMeasurement();
  MatrixWrapper::ColumnVector visual_meas_noise_mean(visual_meas_pdf->DimensionGet());
  visual_meas_noise_mean = 0.0;
  visual_meas_pdf->AdditiveNoiseMuSet(visual_meas_noise_mean);
  // Covariance is set on measurement update,so it does not need initialization.
  BFL::AnalyticMeasurementModelGaussianUncertainty* visual_meas_model =
      new BFL::AnalyticMeasurementModelGaussianUncertainty(visual_meas_pdf);
  measurement_pdfs_[VISUAL] = visual_meas_pdf;
  measurement_models_[VISUAL] = visual_meas_model;
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

  const int dim = system_pdf_->DimensionGet();
  Vector prior_mean(dim);
  SymmetricMatrix prior_cov(dim);
  prior_mean = 0.0;
  prior_cov = P;
  system_prior_ = new BFL::Gaussian(prior_mean, prior_cov);
  filter_ = new BFL::ExtendedKalmanFilterResetCapable(system_prior_);
  system_pdf_->NominalStateSet(x);
  filter_time_ = t;
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
  if (t<filter_time_)
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

bool pose_twist_meskf::PoseTwistMESKF::addMeasurement(const MeasurementType& m,
                                                      const Vector& z,
                                                      const SymmetricMatrix Q,
                                                      const TimeStamp& t)
{
  if (t<filter_time_)
    return false;
  measurement_queues_[m].push(Measurement(t,z,Q));
  return true;
}


/**
 * @brief Update the filter until some measurement queue is empty.
 * @return whether filter has been updated properly.
 *
 * While there is a complete set of measurement updates to perform
 * (i.e. from each sensor there is at least one measurement pending to process),
 * all inputs and measurements are processed in order as follows:
 *   - process all inputs previous to the next measurement.
 *   - since the inputs and the measurements are not synchronized,
 *     update the system with the last input until the next measurement time.
 *   - correct the system state with the measurement and repeat.
 */
bool pose_twist_meskf::PoseTwistMESKF::update()
{
  bool success = true;
  while( success && !input_queue_.empty() )
  {
    success = updateFilterSys(input_queue_.top());
    input_queue_.pop();
  }
  /*
  bool empty_queue = false;
  while (!empty_queue)
  {
    int qnext = measurement_queues_.size()-1;
    empty_queue = measurement_queues_[qnext].empty();
    for (int q = qnext-1 ; !empty_queue && (q >= 0); q--)
    {
      if (measurement_queues_[q].empty())
        empty_queue = true;
      else if( measurement_queues_[q].top().t_ < measurement_queues_[qnext].top().t_ )
        qnext = q;
    }
    if(!empty_queue)
    {
      while( (!input_queue_.empty()) &&
             (input_queue_.top().t_<=measurement_queues_[qnext].top().t_) )
      {
        updateFilterSys(input_queue_.top());
        input_queue_.pop();
      }
      if(filter_time_<measurement_queues_[qnext].top().t_)
      {
        updateFilterSys(Input(measurement_queues_[qnext].top().t_,filter_input_));
      }
      updateFilterMeas(qnext, measurement_queues_[qnext].top());
      measurement_queues_[qnext].pop();
    }
  }
  */
  return success;
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
bool pose_twist_meskf::PoseTwistMESKF::updateFilterSys(const Input& in)
{
  TimeStamp input_t = in.t_;
  Vector input_val = in.u_;
  int input_dim = input_val.rows();
  Vector u(input_dim+1);
  for (int i=1; i<=input_dim; i++)
    u(i)=input_val(i);
  u(input_dim+1) = input_t - filter_time_;
  bool success = filter_->Update(system_model_,u);
  if (success)
  {
    filter_time_ = input_t;
    filter_input_ = input_val;
  }
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
#include "error_state_vector.h"
bool pose_twist_meskf::PoseTwistMESKF::updateFilterMeas(const MeasurementIndex& i,
                                                        const Measurement& meas)
{
  bool success = false;
  if (meas.t_ == filter_time_)
  {
    Vector x = system_pdf_->NominalStateGet();
    measurement_pdfs_[i]->AdditiveNoiseSigmaSet(meas.Q_);
    success = filter_->Update(measurement_models_[i],
                              measurement_pdfs_[i]->ErrorMeasurement(meas.z_,x),
                              x);
  }
  if (success)
  {
    const Vector error_state = filter_->PostGet()->ExpectedValueGet();
    system_pdf_->CorrectNominalState(filter_->PostGet()->ExpectedValueGet());
    Vector zero_error_state(system_pdf_->DimensionGet());
    zero_error_state = 0.0;
    filter_->PostMuSet(zero_error_state);
  }
  return success;
}
