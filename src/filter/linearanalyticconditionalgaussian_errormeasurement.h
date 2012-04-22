/**
 * @file
 * @author Joan Pau Beltran
 * @brief Linear Conditional Gaussian for error measurements in ESKF (presentation).
 *
 * Provide an abstract class that extends the BFL conditional pdf class
 * for measurements in error state Kalman filtering.
 * It just adds a pure virtual member function
 * to convert nominal state measurement to an error state mesaurements.
  */

#ifndef LINEARANALYTICCONDITIONALGAUSSIAN_ERRORMEASUREMENT_H
#define LINEARANALYTICCONDITIONALGAUSSIAN_ERRORMEASUREMENT_H

#include <pdf/linearanalyticconditionalgaussian.h>

namespace BFL
{
  /**
   * @brief Linear Conditional Gaussian for error measurements.
   *
   * This abstract class is intended to be used with error state Kalman filters.
   * It just adds a pure virtual member function ErrorMeasurement()
   * that converts a nominal state measurement into an error state mesaurement
   * given the current nominal state.
   */
  class LinearAnalyticConditionalGaussianErrorMeasurement
  : public LinearAnalyticConditionalGaussian
  {
  public:

      LinearAnalyticConditionalGaussianErrorMeasurement(const vector<MatrixWrapper::Matrix> & ratio,
                                                        const Gaussian& additiveNoise);

      LinearAnalyticConditionalGaussianErrorMeasurement(const MatrixWrapper::Matrix& a,
                                                        const Gaussian& additiveNoise);

      // Default copy constructor will do

      // Destructor
      virtual ~LinearAnalyticConditionalGaussianErrorMeasurement();

      // No clone function since it is an abstract class and can not
      // return a pointer to an object of it.
      // virtual LinearAnalyticConditionalGaussianErrorMeasurement* Clone() const;

      // Virtual functions inherited from base class are enough.
      // virtual MatrixWrapper::ColumnVector    ExpectedValueGet()    const;
      // virtual MatrixWrapper::Matrix          dfGet(unsigned int i) const;

      // Pure virtual function to convert nominal measurements to error measurements.
      virtual MatrixWrapper::ColumnVector ErrorMeasurement(const MatrixWrapper::ColumnVector& z,
                                                           const MatrixWrapper::ColumnVector& x) const = 0;

    };

} // End namespace BFL

#endif // LINEARANALYTICCONDITIONALGAUSSIAN_ERRORMEASUREMENT_H
