/**
 * @file
 * @author Joan Pau Beltran
 */

#include "pose_twist_meskf.h"
#include "nominal_state_vector.h"
#include "error_state_vector.h"
#include "input_vector.h"
#include "visual_measurement_vector.h"
#include "visual_measurement_error_vector.h"
#include <fstream>
#include <string>
#include <iomanip>
#include <Eigen/Dense>
#include <iterator>
#include <ctime>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

typedef std::map<double, pose_twist_meskf::PoseTwistMESKF::Vector> VectorSerie;


void writeEstimates(std::ostream& out, const VectorSerie& output)
{
  for (VectorSerie::const_iterator it = output.begin(); it != output.end(); it++)
  {
    double t = it->first;
    pose_twist_meskf::NominalStateVector x;
    x.fromVector(it->second);
    out << std::setprecision(15) << std::fixed
        << t
//        << ' ' << x.position_.transpose()
//        << ' ' << (x.orientation_.toRotationMatrix()*x.lin_vel_).transpose()
//        << ' ' << x.acc_bias_.transpose()
        << ' ' << x.ang_vel_.transpose()
        << ' ' << x.gyro_drift_.transpose()
//        << ' ' << x.orientation_.w() << ' ' << x.orientation_.vec().transpose()
        << ' ' << x.orientation_.toRotationMatrix().col(0).transpose()
        << ' ' << x.orientation_.toRotationMatrix().col(1).transpose()
        << ' ' << x.orientation_.toRotationMatrix().col(2).transpose()
        << '\n';
  }
}


int main(int argc, char* argv[])
{
  std::ostream& out = std::cout;

  VectorSerie samples_imu;
  VectorSerie samples_vicon;

  const double G_CONS = 9.80665;
  const Eigen::Vector3d G_VECT = Eigen::Vector3d(0.0,0.0,-G_CONS);

  const double VAR_ACC = 4e-2;
  const double VAR_ACC_BIAS = 1e-4;
  const double VAR_GYRO = 1e-2;
  const double VAR_GYRO_DRIFT = 1e-9;

  const double VAR_MEAS_ANG_VEL = VAR_GYRO + 1e-12;
  const double VAR_MEAS_ORIENTATION = 1e-4;

  const double LOCAL_ANG_VEL_RATE = 2.0 * M_PI * 0.1;
  const Eigen::Vector3d LOCAL_ANG_VEL_AXIS(0.0, 0.0, 1.0);
  const Eigen::Vector3d LOCAL_ANG_VEL_VECT = LOCAL_ANG_VEL_RATE*LOCAL_ANG_VEL_AXIS;
  const Eigen::Vector3d LOCAL_ANG_VEL_DRIFT = 2.0*M_PI * Eigen::Vector3d(0.01, 0.0, -0.02);

  const size_t INSTANTS = 20;
  const double TIME_STEP = 2.0 * M_PI / LOCAL_ANG_VEL_RATE / INSTANTS;

  boost::mt19937 rand_gen(static_cast<unsigned int>(std::time(0)));
  boost::normal_distribution<double> norm_dist(0.0, 1.0);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > norm_var(rand_gen, norm_dist);
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  for (size_t i=0; i<INSTANTS; i++)
  {
    const double time = i*TIME_STEP;
    const Eigen::Vector3d gyro_drift_white_noise(norm_var(), norm_var(), norm_var());
    const Eigen::Vector3d gyro_drift_noise = sqrt(VAR_GYRO_DRIFT)*time*gyro_drift_white_noise;
    const Eigen::Vector3d gyro_white_noise(norm_var(), norm_var(), norm_var());
    const Eigen::Vector3d gyro_noise = sqrt(VAR_GYRO)*TIME_STEP*gyro_white_noise;
    const Eigen::Vector3d gyro_ang_vel = LOCAL_ANG_VEL_VECT + LOCAL_ANG_VEL_DRIFT + gyro_noise + gyro_drift_noise;
    pose_twist_meskf::PoseTwistMESKF::Vector input(6);
    input(pose_twist_meskf::InputVector::LIN_ACC_X) = G_VECT(0);
    input(pose_twist_meskf::InputVector::LIN_ACC_Y) = G_VECT(1);
    input(pose_twist_meskf::InputVector::LIN_ACC_Z) = G_VECT(2);
    input(pose_twist_meskf::InputVector::ANG_VEL_X) = gyro_ang_vel(0);
    input(pose_twist_meskf::InputVector::ANG_VEL_Y) = gyro_ang_vel(1);
    input(pose_twist_meskf::InputVector::ANG_VEL_Z) = gyro_ang_vel(2);
    samples_imu.insert(samples_imu.end(),
                       VectorSerie::value_type(time, input));
    const Eigen::Vector3d vicon_white_noise(norm_var(), norm_var(), norm_var());
    const Eigen::Vector3d vicon_noise = sqrt(VAR_MEAS_ORIENTATION)*vicon_white_noise;
    pose_twist_meskf::VisualMeasurementVector measurement;
    measurement.orientation_ = orientation * Eigen::Quaterniond(Eigen::AngleAxisd(vicon_noise.norm(), vicon_noise.normalized()));
    measurement.ang_vel_ = LOCAL_ANG_VEL_VECT + vicon_noise;
    samples_vicon.insert(samples_vicon.end(),
                         VectorSerie::value_type(time, measurement.toVector()));
    orientation *= Eigen::Quaterniond(Eigen::AngleAxisd(LOCAL_ANG_VEL_RATE*TIME_STEP, LOCAL_ANG_VEL_AXIS));
  }

  std::clog << samples_imu.size() << " imu readings." << std::endl;
  std::clog << samples_vicon.size() << " vicon readings." << std::endl;

  pose_twist_meskf::PoseTwistMESKF filter;

  // Filter setup.
  filter.setUpSystem(VAR_ACC, VAR_GYRO, VAR_ACC_BIAS, VAR_GYRO_DRIFT, G_VECT);
  filter.setUpMeasurementModels();
  // Visual measurement uncertainty.
  pose_twist_meskf::PoseTwistMESKF::SymmetricMatrix
    R(pose_twist_meskf::VisualMeasurementErrorVector::DIMENSION);
  R = 0.0;
  for (int i=0; i<3; i++)
  {
//    R(pose_twist_meskf::VisualMeasurementErrorVector::D_POSITION_X + i,
//      pose_twist_meskf::VisualMeasurementErrorVector::D_POSITION_X + i) = 1e-2;
//    R(pose_twist_meskf::VisualMeasurementErrorVector::D_LIN_VEL_X + i,
//      pose_twist_meskf::VisualMeasurementErrorVector::D_LIN_VEL_X + i) = VAR_MEAS_LIN_VEL;
//    R(pose_twist_meskf::VisualMeasurementErrorVector::D_ACC_BIAS_X + i,
//      pose_twist_meskf::VisualMeasurementErrorVector::D_ACC_BIAS_X + i) = 1e-4;
    R(pose_twist_meskf::VisualMeasurementErrorVector::D_ORIENTATION_X + i,
      pose_twist_meskf::VisualMeasurementErrorVector::D_ORIENTATION_X + i) = VAR_MEAS_ORIENTATION;
//    R(pose_twist_meskf::VisualMeasurementErrorVector::D_GYRO_DRIFT_X + i,
//      pose_twist_meskf::VisualMeasurementErrorVector::D_GYRO_DRIFT_X + i) = VAR_MEAS_ANG_VEL;
  }

  // Filter initialization.
  double t0 = samples_imu.begin()->first;
  pose_twist_meskf::VisualMeasurementVector measurement;
  measurement.fromVector(samples_vicon.begin()->second);
  pose_twist_meskf::NominalStateVector nominal_state;
//  nominal_state.position_ = measurement.position_;
//  nominal_state.lin_vel_ = measurement.lin_vel_;
//  nominal_state.acc_bias_ = Eigen::Vector3d::Zero();
  nominal_state.orientation_ = Eigen::Quaterniond::Identity(); //measurement.orientation_;
  nominal_state.gyro_drift_ = Eigen::Vector3d::Zero();
//  nominal_state.lin_acc_ = measurement.lin_acc_;
  nominal_state.ang_vel_ = measurement.ang_vel_;

  pose_twist_meskf::PoseTwistMESKF::Vector x0 = nominal_state.toVector();
  pose_twist_meskf::PoseTwistMESKF::SymmetricMatrix
    P0(pose_twist_meskf::ErrorStateVector::DIMENSION);
  P0 = 0.0;
  for (int i=0; i<3; i++)
  {
//    P0(pose_twist_meskf::ErrorStateVector::D_POSITION_X + i,
//       pose_twist_meskf::ErrorStateVector::D_POSITION_X + i) = 1e-1;
//    P0(pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X + i,
//    P0(pose_twist_meskf::ErrorStateVector::D_ACC_BIAS_X + i,
//       pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X + i) = 1000; // 1e-1;
//       pose_twist_meskf::ErrorStateVector::D_ACC_BIAS_X + i) = 1000; // 1.0;
    P0(pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + i,
       pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + i) = 1e-1;
    P0(pose_twist_meskf::ErrorStateVector::D_GYRO_DRIFT_X + i,
       pose_twist_meskf::ErrorStateVector::D_GYRO_DRIFT_X + i) = 1e+6; // 1.0;
  }

  double t = t0;
  pose_twist_meskf::PoseTwistMESKF::Vector x = x0;
  pose_twist_meskf::PoseTwistMESKF::SymmetricMatrix P = P0;
  VectorSerie samples_filter;
  samples_filter.insert(samples_filter.end(),VectorSerie::value_type(t,x));
  filter.initialize(x, P, t);

  int added_measurements = 0;
  int added_inputs = 0;
  VectorSerie::const_iterator tu_it = ++samples_imu.begin();
  VectorSerie::const_iterator tz_it = ++samples_vicon.begin();
  double ellapsed_time = t;
  const double update_rate = 5;
  while (tu_it != samples_imu.end())
  {
    ellapsed_time += 1.0/update_rate;
    for (;tu_it != samples_imu.end() && tu_it->first <= ellapsed_time; tu_it++)
    {
      for (;tz_it != samples_vicon.end() && tz_it->first <= tu_it->first; tz_it++)
      {
        bool ok = filter.addMeasurement(pose_twist_meskf::PoseTwistMESKF::VISUAL,
                                        tz_it->second, R, tz_it->first);
        if (ok)
          added_measurements++;
        else
          std::clog << "Error adding measurement " << added_measurements+1 << ".\n";
      }
      bool ok = filter.addInput(tu_it->first,tu_it->second);
      if (ok)
        added_inputs++;
      else
        std::clog << "Error adding input " << added_measurements+1 << ".\n";
    }
    bool success = filter.update();
    if (! success)
      std::cerr << "Filter update failure.\n";
    filter.getEstimate(x,P,t);
    samples_filter.insert(samples_filter.end(),VectorSerie::value_type(t,x));
  }

  std::clog << "Added inputs : " << added_inputs << ".\n";
  std::clog << "Added measurements : " << added_measurements << ".\n";
  writeEstimates(out, samples_filter);

}