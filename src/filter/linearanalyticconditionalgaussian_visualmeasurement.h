/**
 * @file
 * @author Joan Pau Beltran
 * @brief Linear Conditional Gaussian for measurement from visual sensors (presentation).
 */

#ifndef LINEARANALYTICCONDITIONALGAUSSIAN_VISUALMEASUREMENT_H
#define LINEARANALYTICCONDITIONALGAUSSIAN_VISUALMEASUREMENT_H

#include "linearanalyticconditionalgaussian_errormeasurement.h"

namespace BFL
{
/**
* @brief Linear Conditional Gaussian for visual sensor measurements.
*
*/
class LinearAnalyticConditionalGaussianVisualMeasurement
: public LinearAnalyticConditionalGaussianErrorMeasurement
{
public:

  LinearAnalyticConditionalGaussianVisualMeasurement(const Gaussian& additiveNoise);

  // Default copy constructor will do

  // Destructor
  virtual ~LinearAnalyticConditionalGaussianVisualMeasurement();

  // Clone function
  virtual LinearAnalyticConditionalGaussianVisualMeasurement* Clone() const;

  // Nominal state measurement to error state measurement conversion
  MatrixWrapper::ColumnVector ErrorMeasurement(const MatrixWrapper::ColumnVector& mesurement,
                                               const MatrixWrapper::ColumnVector& nominal_state) const;

  static const MatrixWrapper::Matrix H;  //!< Measurement matrix.

  static const MatrixWrapper::Matrix initH();  //!< Measurement matrix initialization.

};

} // End namespace BFL

#endif // LINEARANALYTICCONDITIONALGAUSSIAN_VISUALMEASUREMENT_H
