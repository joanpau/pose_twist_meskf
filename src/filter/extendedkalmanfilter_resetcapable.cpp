
#include "extendedkalmanfilter_resetcapable.h"

namespace BFL
{
  using namespace MatrixWrapper;


  ExtendedKalmanFilterResetCapable::ExtendedKalmanFilterResetCapable(Gaussian * prior)
    : ExtendedKalmanFilter(prior)
  {}

  ExtendedKalmanFilterResetCapable::~ExtendedKalmanFilterResetCapable()
  {}

  void
  ExtendedKalmanFilterResetCapable::PostSigmaSet( const MatrixWrapper::SymmetricMatrix& s)
  {
    ExtendedKalmanFilter::PostSigmaSet(s);
  }

  void
  ExtendedKalmanFilterResetCapable::PostMuSet( const MatrixWrapper::ColumnVector& c)
  {
    ExtendedKalmanFilter::PostMuSet(c);
  }

}
