/*
 * sfly_test_main.cpp
 *
 *  Created on: 06/09/2011
 *      Author: usuari
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

typedef std::map<double, pose_twist_meskf::PoseTwistMESKF::Vector> VectorSerie;

Eigen::Quaterniond RPYtoQuaternion(double roll, double pitch, double yaw)
{
  Eigen::Quaterniond r(Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX()));
  Eigen::Quaterniond p(Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()));
  Eigen::Quaterniond y(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()));
  return y * p * r;
}

void usage()
{
  const std::string usage = "sfly_test IMU_FILE VICON_FILE\n";
  std::cout << usage;
}

bool parseCmdLine(int argc, char* argv[],
                  std::string* filename_imu,
                  std::string* filename_vicon)
{
  if ( argc < 3 )
    return false;
  filename_imu->assign(argv[1]);
  filename_vicon->assign(argv[2]);
  return true;
}

bool readInputs(std::istream& in_imu, VectorSerie* input_imu )
{
  const int SIZE_IMU_LINE = 10;
  bool success_imu = true;
  while (in_imu.peek() == '%')
    in_imu.ignore(std::numeric_limits<int>::max(),'\n');
  while (success_imu)
  {
    double line[SIZE_IMU_LINE];
    int count = 0;
    while ( count < SIZE_IMU_LINE && in_imu >> line[count++] )
    {}
    if ( count < SIZE_IMU_LINE )
    {
      success_imu = false;
    }
    else
    {
      double t = line[0];
      pose_twist_meskf::PoseTwistMESKF::Vector u(pose_twist_meskf::InputVector::DIMENSION);
      u = 0.0;
      u(pose_twist_meskf::InputVector::LIN_ACC_X) = line[1];
      u(pose_twist_meskf::InputVector::LIN_ACC_Y) = line[2];
      u(pose_twist_meskf::InputVector::LIN_ACC_Z) = line[3];
      u(pose_twist_meskf::InputVector::ANG_VEL_X) = line[4];
      u(pose_twist_meskf::InputVector::ANG_VEL_Y) = line[5];
      u(pose_twist_meskf::InputVector::ANG_VEL_Z) = line[6];
      u(pose_twist_meskf::InputVector::TIME) = t;
      input_imu->insert(input_imu->end(),VectorSerie::value_type(t,u));
    }
  }
  return ! input_imu->empty();
}


bool readMeasurements(std::istream& in_vicon, VectorSerie* input_vicon )
{
  const int SIZE_VICON_LINE = 10;
  bool success_vicon = true;
  while (in_vicon.peek() == '%')
      in_vicon.ignore(std::numeric_limits<int>::max(),'\n');
  while(success_vicon)
  {
    double line[SIZE_VICON_LINE];
    int count = 0;
    while ( count < SIZE_VICON_LINE && in_vicon >> line[count++] )
    {}
    if ( count < SIZE_VICON_LINE )
    {
      success_vicon = false;
    }
    else
    {
      double t = line[0];
      pose_twist_meskf::PoseTwistMESKF::Vector z(pose_twist_meskf::VisualMeasurementVector::DIMENSION);
      z = 0.0;
      z(pose_twist_meskf::VisualMeasurementVector::POSITION_X) = line[1];
      z(pose_twist_meskf::VisualMeasurementVector::POSITION_Y) = line[2];
      z(pose_twist_meskf::VisualMeasurementVector::POSITION_Z) = line[3];
      double roll = line[4];
      double pitch = line[5];
      double yaw = line[6];
      Eigen::Quaterniond q = RPYtoQuaternion(roll, pitch, yaw);
      z(pose_twist_meskf::VisualMeasurementVector::ORIENTATION_W) = q.w();
      z(pose_twist_meskf::VisualMeasurementVector::ORIENTATION_X) = q.x();
      z(pose_twist_meskf::VisualMeasurementVector::ORIENTATION_Y) = q.y();
      z(pose_twist_meskf::VisualMeasurementVector::ORIENTATION_Z) = q.z();
      z(pose_twist_meskf::VisualMeasurementVector::ANG_VEL_X) = line[7];
      z(pose_twist_meskf::VisualMeasurementVector::ANG_VEL_Y) = line[8];
      z(pose_twist_meskf::VisualMeasurementVector::ANG_VEL_Z) = line[9];
      input_vicon->insert(input_vicon->end(),VectorSerie::value_type(t,z));
    }
  }
  return ! input_vicon->empty();
}


void computeVelocityAndAccelerationMeasurements(VectorSerie* input_vicon)
{
  VectorSerie::iterator curr, next;
  for (next = input_vicon->begin(), curr = next++; next != input_vicon->end(); curr = next++)
  {
    const double curr_t = curr->first;
    const double next_t = next->first;
    const double dt = next_t - curr_t;
    pose_twist_meskf::VisualMeasurementVector curr_z, next_z;
    curr_z.fromVector(curr->second);
    next_z.fromVector(next->second);
    if (curr == input_vicon->begin())
    {
//      curr_z.ang_vel_ = Eigen::Vector3d::Zero();
      curr_z.lin_vel_ = Eigen::Vector3d::Zero();
      curr_z.lin_acc_ = Eigen::Vector3d::Zero();
    }
//    Eigen::AngleAxis<double> aa(curr_z.orientation_.inverse()*next_z.orientation_);
//    next_z.ang_vel_ = aa.axis()*aa.angle() / dt;
    next_z.lin_vel_ = curr_z.orientation_.toRotationMatrix().transpose() *
                      (next_z.position_ - curr_z.position_) / dt;
    next_z.lin_acc_ = (next_z.lin_vel_ - curr_z.lin_vel_) / dt;
    (next->second) = next_z.toVector();
  }
}


void filterInputs(VectorSerie* input_imu, const int n = 10)
{
  VectorSerie original_imu = *input_imu;
  for (VectorSerie::iterator it = input_imu->begin();
       it != input_imu->end();
       it++)
  {
    pose_twist_meskf::PoseTwistMESKF::Vector u(pose_twist_meskf::InputVector::DIMENSION);
    u = 0.0;
    const int p = std::distance(input_imu->begin(), it);
    VectorSerie::const_iterator first = original_imu.begin();
    VectorSerie::const_iterator last = original_imu.begin();
    std::advance(first, std::max(0,p-n));
    std::advance(last, std::min(size_t(p+n), original_imu.size()));
    const size_t w = std::distance(first, last);
    for (VectorSerie::const_iterator jt = first; jt != last; jt++)
    {
      u += jt->second;
    }
    u /= w;
    u(pose_twist_meskf::InputVector::TIME) = it->first;
  }
}


void filterMeasurements(VectorSerie* input_vicon, const int n = 10)
{
  VectorSerie original_vicon = *input_vicon;
  for (VectorSerie::iterator it = input_vicon->begin();
       it != input_vicon->end();
       it++)
  {
    pose_twist_meskf::VisualMeasurementVector m;
    m.fromVector(it->second);
    m.ang_vel_ = Eigen::Vector3d::Zero();
    m.lin_vel_ = Eigen::Vector3d::Zero();
    m.lin_acc_ = Eigen::Vector3d::Zero();
    const int p = std::distance(input_vicon->begin(), it);
    VectorSerie::const_iterator first = original_vicon.begin();
    VectorSerie::const_iterator last = original_vicon.begin();
    std::advance(first, std::max(0,p-n));
    std::advance(last, std::min(size_t(p+n), original_vicon.size()));
    const size_t w = std::distance(first, last);
    for (VectorSerie::const_iterator jt = first; jt != last; jt++)
    {
      pose_twist_meskf::VisualMeasurementVector s;
      s.fromVector(jt->second);
      m.ang_vel_ += s.ang_vel_;
      m.lin_vel_ += s.lin_vel_;
      m.lin_acc_ += s.lin_acc_;
    }
    m.ang_vel_ /= w;
    m.lin_vel_ /= w;
    m.lin_acc_ /= w;
    (it->second) = m.toVector();
  }
}


bool computeInputFromVicon(VectorSerie* imu, const VectorSerie& vicon)
{
  imu->clear();
  const Eigen::Vector3d G(0.0,0.0,-9.80665);
  for (VectorSerie::const_iterator it = vicon.begin(); it != vicon.end(); it++)
  {
    const double t = it->first;
    pose_twist_meskf::PoseTwistMESKF::Vector u(6);
    pose_twist_meskf::VisualMeasurementVector z;
    z.fromVector(it->second);
    Eigen::Vector3d g_local = z.orientation_.toRotationMatrix().inverse()*G;
    Eigen::Vector3d a = g_local - z.lin_acc_;
    Eigen::Vector3d w = z.ang_vel_;
    u(pose_twist_meskf::InputVector::LIN_ACC_X) = a(0);
    u(pose_twist_meskf::InputVector::LIN_ACC_Y) = a(1);
    u(pose_twist_meskf::InputVector::LIN_ACC_Z) = a(2);
    u(pose_twist_meskf::InputVector::ANG_VEL_X) = w(0);
    u(pose_twist_meskf::InputVector::ANG_VEL_Y) = w(1);
    u(pose_twist_meskf::InputVector::ANG_VEL_Z) = w(2);
    u(pose_twist_meskf::InputVector::TIME) = t;
    imu->insert(imu->end(),VectorSerie::value_type(t,u));
  }
  return true;
}


void writeEstimates(std::ostream& out, const VectorSerie& output)
{
  for (VectorSerie::const_iterator it = output.begin(); it != output.end(); it++)
  {
    double t = it->first;
    pose_twist_meskf::NominalStateVector x;
    x.fromVector(it->second);
    out << std::setprecision(15) << std::fixed
        << t
        << ' ' << x.position_.transpose()
        << ' ' << (x.orientation_.toRotationMatrix()*x.lin_vel_).transpose()
        << ' ' << x.acc_bias_.transpose()
        << ' ' << x.ang_vel_.transpose()
        << ' ' << x.gyro_drift_.transpose()
        << ' ' << x.orientation_.w() << ' ' << x.orientation_.vec().transpose()
        << '\n';
  }
}


int main(int argc, char* argv[])
{
  std::string filename_imu, filename_vicon;
  const bool args_ok = parseCmdLine(argc, argv,
                                    &filename_imu, &filename_vicon);
  if ( ! args_ok )
  {
    usage();
    return 1;
  }

  std::ostream& out = std::cout;

  std::ifstream in_imu(filename_imu.c_str());
  std::ifstream in_vicon(filename_vicon.c_str());

  VectorSerie samples_imu;
  VectorSerie samples_vicon;

  bool input_ok;

  input_ok = readMeasurements(in_vicon, &samples_vicon) &&
             readInputs(in_imu, &samples_imu);
  std::clog << samples_imu.size() << " imu readings." << std::endl;
  std::clog << samples_vicon.size() << " vicon readings." << std::endl;
  if ( ! input_ok )
    return 1;

  computeVelocityAndAccelerationMeasurements(&samples_vicon);

//  filterInputs(&samples_imu, 1);
//  filterMeasurements(&samples_vicon, 1);

//  input_ok = computeInputFromVicon(&samples_imu, samples_vicon);
//  if ( ! input_ok )
//    return 1;

  const double TIME_STEP = 1e-1;

  const double G_CONS = 9.80665;
  const Eigen::Vector3d G_VECT = Eigen::Vector3d(0.0,0.0,-G_CONS);

  const double VAR_ACC = 4e-2;
  const double VAR_ACC_BIAS = 1e-4;
  const double VAR_GYRO = 1e-4;
  const double VAR_GYRO_DRIFT = 1e-8;

  const double VAR_MEAS_LIN_VEL = 1e-4;
  const double VAR_MEAS_ANG_VEL = 1e-4 + 1e-8; //1e-4 + 1e+14;

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
//    R(pose_twist_meskf::VisualMeasurementErrorVector::D_ORIENTATION_X + i,
//      pose_twist_meskf::VisualMeasurementErrorVector::D_ORIENTATION_X + i) = 1e-4;
    R(pose_twist_meskf::VisualMeasurementErrorVector::D_LIN_VEL_X + i,
      pose_twist_meskf::VisualMeasurementErrorVector::D_LIN_VEL_X + i) = VAR_MEAS_LIN_VEL;
//    R(pose_twist_meskf::VisualMeasurementErrorVector::D_ACC_BIAS_X + i,
//      pose_twist_meskf::VisualMeasurementErrorVector::D_ACC_BIAS_X + i) = 1e-4;
    R(pose_twist_meskf::VisualMeasurementErrorVector::D_GYRO_DRIFT_X + i,
      pose_twist_meskf::VisualMeasurementErrorVector::D_GYRO_DRIFT_X + i) = VAR_MEAS_ANG_VEL;
  }

  // Filter initialization.
  double t0 = samples_imu.begin()->first;
  pose_twist_meskf::VisualMeasurementVector measurement;
  measurement.fromVector(samples_vicon.begin()->second);
  pose_twist_meskf::NominalStateVector nominal_state;
  nominal_state.position_ = measurement.position_;
  nominal_state.lin_vel_ = measurement.lin_vel_;
  nominal_state.acc_bias_ = Eigen::Vector3d::Zero();
  nominal_state.orientation_ = measurement.orientation_;
  nominal_state.gyro_drift_ = Eigen::Vector3d::Zero();
  nominal_state.lin_acc_ = measurement.lin_acc_;
  nominal_state.ang_vel_ = measurement.ang_vel_;
  nominal_state.time_ = t0;

  pose_twist_meskf::PoseTwistMESKF::Vector x0 = nominal_state.toVector();
  pose_twist_meskf::PoseTwistMESKF::SymmetricMatrix
    P0(pose_twist_meskf::ErrorStateVector::DIMENSION);
  P0 = 0.0;
  for (int i=0; i<3; i++)
  {
    P0(pose_twist_meskf::ErrorStateVector::D_POSITION_X + i,
       pose_twist_meskf::ErrorStateVector::D_POSITION_X + i) = 1e-1;
    P0(pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X + i,
       pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X + i) = 1e+12; // 1e-1;
    P0(pose_twist_meskf::ErrorStateVector::D_ACC_BIAS_X + i,
       pose_twist_meskf::ErrorStateVector::D_ACC_BIAS_X + i) = 1e+12; // 1.0;
    P0(pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + i,
       pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + i) = 1e-1;
    P0(pose_twist_meskf::ErrorStateVector::D_GYRO_DRIFT_X + i,
       pose_twist_meskf::ErrorStateVector::D_GYRO_DRIFT_X + i) = 1e+12; // 1.0;
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
  while (tu_it != samples_imu.end())
  {
    t += TIME_STEP;
    for (;tu_it != samples_imu.end() && tu_it->first <= t; tu_it++)
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
