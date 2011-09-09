/**
 * @file
 * @author Joan Pau Beltran
 * @brief Linear Conditional Gaussian for error measurements in ESKF (implementation).
 *
 * Provide an abstract class that extends the BFL conditional pdf class
 * for measurements in error state Kalman filtering.
 * It just adds a pure virtual member function
 * to convert nominal state measurement to an error state mesaurements.
  */

#include "linearanalyticconditionalgaussian_errormeasurement.h"

/**
 * @brief Constructor bypassing its arguments to the base conditional pdf constructor.
 * @param ratio vector containing the different matrices
 *              relating the conditional arguments and the expected value.
 * @param additiveNoise pdf representing the additive Gaussian uncertainty.
 * @return
 */
BFL::LinearAnalyticConditionalGaussianErrorMeasurement::
LinearAnalyticConditionalGaussianErrorMeasurement(const vector<Matrix> & ratio,
                                                  const Gaussian& additiveNoise)
: LinearAnalyticConditionalGaussian(ratio,additiveNoise)
{}


/**
 * @brief Constructor bypassing its arguments to the base conditional pdf constructor.
 * @param a matrix relating the conditional arguments and the expected value.
 * @param additiveNoise pdf representing the additive Gaussian uncertainty.
 * @return
 */
BFL::LinearAnalyticConditionalGaussianErrorMeasurement::
LinearAnalyticConditionalGaussianErrorMeasurement(const Matrix& a,
                                                  const Gaussian& additiveNoise)
: LinearAnalyticConditionalGaussian(a,additiveNoise)
{}


/**
 * @brief Destructor (doing nothing).
 * @return
 */
BFL::LinearAnalyticConditionalGaussianErrorMeasurement::
~LinearAnalyticConditionalGaussianErrorMeasurement()
{}


