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
      pose_twist_meskf::PoseTwistMESKF::Vector v(6);
      v = 0.0;
      v(pose_twist_meskf::InputVector::LIN_ACC_X) = line[1];
      v(pose_twist_meskf::InputVector::LIN_ACC_Y) = line[2];
      v(pose_twist_meskf::InputVector::LIN_ACC_Z) = line[3];
      v(pose_twist_meskf::InputVector::ANG_VEL_X) = line[4];
      v(pose_twist_meskf::InputVector::ANG_VEL_Y) = line[5];
      v(pose_twist_meskf::InputVector::ANG_VEL_Z) = line[6];
      input_imu->insert(input_imu->end(),VectorSerie::value_type(t,v));
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
      pose_twist_meskf::PoseTwistMESKF::Vector u(pose_twist_meskf::VisualMeasurementVector::DIMENSION);
      u = 0.0;
      u(pose_twist_meskf::VisualMeasurementVector::POSITION_X) = line[1];
      u(pose_twist_meskf::VisualMeasurementVector::POSITION_Y) = line[2];
      u(pose_twist_meskf::VisualMeasurementVector::POSITION_Z) = line[3];
      double roll = line[4];
      double pitch = line[5];
      double yaw = line[6];
      Eigen::Quaterniond q = RPYtoQuaternion(roll, pitch, yaw);
      u(pose_twist_meskf::VisualMeasurementVector::ORIENTATION_W) = q.w();
      u(pose_twist_meskf::VisualMeasurementVector::ORIENTATION_X) = q.x();
      u(pose_twist_meskf::VisualMeasurementVector::ORIENTATION_Y) = q.y();
      u(pose_twist_meskf::VisualMeasurementVector::ORIENTATION_Z) = q.z();
      input_vicon->insert(input_vicon->end(),VectorSerie::value_type(t,u));
    }
  }
  return ! input_vicon->empty();
}


void filterMeasurements(VectorSerie* input_vicon, const int n = 10)
{
  VectorSerie filtered_vicon;
  for (VectorSerie::const_iterator it = input_vicon->begin();
       it != input_vicon->end(); std::advance(it,n))
    filtered_vicon.insert(filtered_vicon.end(), *it);
  *input_vicon = filtered_vicon;
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
      curr_z.ang_vel_ = Eigen::Vector3d::Zero();
      curr_z.lin_vel_ = curr_z.orientation_.toRotationMatrix().transpose() *
          (next_z.position_ - curr_z.position_) / dt;
      curr_z.lin_acc_ = Eigen::Vector3d::Zero();
    }
    Eigen::AngleAxis<double> aa(next_z.orientation_*curr_z.orientation_.inverse());
    next_z.ang_vel_ = aa.axis()*aa.angle() / dt;
    next_z.lin_acc_ = curr_z.orientation_.toRotationMatrix().transpose() *
                      2.0/(dt*dt) * (next_z.position_ - curr_z.position_ - dt * curr_z.orientation_.toRotationMatrix()*curr_z.lin_vel_ );
    next_z.lin_vel_ = next_z.orientation_.toRotationMatrix().transpose() *
                      curr_z.orientation_.toRotationMatrix() *
                      (curr_z.lin_vel_ + dt * curr_z.lin_acc_);
    next_z.toVector(next->second);
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
    out << t << ' '
        << x.position_(0) << ' ' << x.position_(1) << ' ' << x.position_(2)
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

//  filterMeasurements(&samples_vicon, 10);
  computeVelocityAndAccelerationMeasurements(&samples_vicon);

//  input_ok = computeInputFromVicon(&samples_imu, samples_vicon);
//  if ( ! input_ok )
//    return 1;

  const double TIME_STEP = 1e-1;
  const double VAR_ACC = 100;
  const double VAR_ACC_BIAS = 10; // 1e-2;
  const double VAR_GYRO = 100;
  const double VAR_GYRO_DRIFT = 10; // 1e-4;

  pose_twist_meskf::PoseTwistMESKF filter;

  filter.setUpSystem(VAR_ACC,VAR_GYRO,VAR_ACC_BIAS,VAR_GYRO_DRIFT);

  filter.setUpMeasurementModels();

  double t0 = samples_imu.begin()->first;
  pose_twist_meskf::VisualMeasurementVector measurement;
  measurement.fromVector(samples_vicon.begin()->second);
  pose_twist_meskf::NominalStateVector nominal_state;
  nominal_state.position_ = measurement.position_;
  nominal_state.orientation_ = measurement.orientation_;
  nominal_state.lin_vel_ = measurement.lin_vel_;
  nominal_state.lin_acc_ = measurement.lin_acc_;
  nominal_state.ang_vel_ = measurement.ang_vel_;
  nominal_state.acc_bias_ = Eigen::Vector3d::Zero();
  nominal_state.gyro_drift_ = Eigen::Vector3d::Zero();
  pose_twist_meskf::PoseTwistMESKF::Vector
    x0(pose_twist_meskf::NominalStateVector::DIMESION);
  nominal_state.toVector(x0);
  pose_twist_meskf::PoseTwistMESKF::SymmetricMatrix
    P0(pose_twist_meskf::ErrorStateVector::DIMENSION);
  P0 = 0.0;
  for (int i=1; i<pose_twist_meskf::ErrorStateVector::DIMENSION; i++)
    P0(i,i) = 1e-4;
  pose_twist_meskf::PoseTwistMESKF::SymmetricMatrix
    Q(pose_twist_meskf::VisualMeasurementErrorVector::DIMENSION);
  Q = 0.0;
  for (int i=1; i<=pose_twist_meskf::VisualMeasurementErrorVector::DIMENSION; i++)
    Q(i,i) = 1e-12;

  double t = t0;
  pose_twist_meskf::PoseTwistMESKF::Vector x = x0;
  pose_twist_meskf::PoseTwistMESKF::SymmetricMatrix P = P0;

  VectorSerie samples_filter;
  filter.initialize(x, P, t);
  samples_filter.insert(samples_filter.end(),VectorSerie::value_type(t,x));
  VectorSerie::const_iterator tu_it = ++samples_imu.begin();
  VectorSerie::const_iterator tz_it = ++samples_vicon.begin();
  int added_measurements = 0;
  int added_inputs = 0;
  while (tu_it != samples_imu.end())
  {
    t += TIME_STEP;
    for (;tu_it != samples_imu.end() && tu_it->first <= t; tu_it++)
    {
      for (;tz_it != samples_vicon.end() && tz_it->first <= tu_it->first; tz_it++)
      {
        bool ok = filter.addMeasurement(pose_twist_meskf::PoseTwistMESKF::VISUAL,
                                        tz_it->second, Q, tz_it->first);
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
