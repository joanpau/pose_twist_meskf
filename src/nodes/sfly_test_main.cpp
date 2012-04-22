/*
 * sfly_test_main.cpp
 *
 *  Created on: 06/09/2011
 *      Author: usuari
 */

#include "pose_twist_meskf.h"
#include "nominal_state_vector.h"
#include "input_vector.h"
#include "visual_measurement_vector.h"
#include "error_state_vector.h"
#include <fstream>
#include <string>

typedef std::map<double, pose_twist_meskf::PoseTwistMESKF::Vector> VectorSerie;

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

bool readInputsAndMeasurements(std::istream& in_imu,
                               std::istream& in_vicon,
                               VectorSerie* input_imu,
                               VectorSerie* input_vicon )
{
  const int SIZE_IMU_LINE = 10;
  const int SIZE_VICON_LINE = 10;

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
      pose_twist_meskf::PoseTwistMESKF::Vector
        v(pose_twist_meskf::InputVector::DIMENSION,0.0);
      v(pose_twist_meskf::InputVector::LIN_ACC_X) = line[1];
      v(pose_twist_meskf::InputVector::LIN_ACC_Y) = line[2];
      v(pose_twist_meskf::InputVector::LIN_ACC_Z) = line[3];
      v(pose_twist_meskf::InputVector::ANG_VEL_X) = line[4];
      v(pose_twist_meskf::InputVector::ANG_VEL_Y) = line[5];
      v(pose_twist_meskf::InputVector::ANG_VEL_Z) = line[6];
      input_imu->insert(input_imu->end(),VectorSerie::value_type(t,v));
    }
  }

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
      pose_twist_meskf::PoseTwistMESKF::Vector
        v(pose_twist_meskf::VisualMeasurementVector::DIMENSION, 0.0);
      v(pose_twist_meskf::VisualMeasurementVector::POSITION_X) = line[1];
      v(pose_twist_meskf::VisualMeasurementVector::POSITION_Y) = line[2];
      v(pose_twist_meskf::VisualMeasurementVector::POSITION_Z) = line[3];
      Eigen::Quaterniond roll(Eigen::AngleAxisd(line[4],Eigen::Vector3d::UnitX()));
      Eigen::Quaterniond pitch(Eigen::AngleAxisd(line[5],Eigen::Vector3d::UnitY()));
      Eigen::Quaterniond yaw(Eigen::AngleAxisd(line[6],Eigen::Vector3d::UnitZ()));
      Eigen::Quaterniond q = yaw * pitch * roll;
      v(pose_twist_meskf::VisualMeasurementVector::ORIENTATION_W) = q.w();
      v(pose_twist_meskf::VisualMeasurementVector::ORIENTATION_X) = q.x();
      v(pose_twist_meskf::VisualMeasurementVector::ORIENTATION_Y) = q.y();
      v(pose_twist_meskf::VisualMeasurementVector::ORIENTATION_Z) = q.z();
      input_vicon->insert(input_vicon->end(),VectorSerie::value_type(t,v));
    }
  }
  return ! ( input_imu->empty() || input_vicon->empty() );
}

void computeVelocityMeasurements(VectorSerie* input_vicon)
{
  VectorSerie::iterator curr, next;
  for (curr = input_vicon->begin(), next = ++(input_vicon->begin()); next != input_vicon->end(); curr++, next++)
  {
    const double curr_t = curr->first;
    const double next_t = next->first;
    pose_twist_meskf::VisualMeasurementVector curr_z, next_z;
    curr_z.fromVector(curr->second);
    curr_z.fromVector(next->second);
    curr_z.lin_vel_ = curr_z.orientation_.toRotationMatrix()
                      * (next_z.position_-curr_z.position_) / (next_t - curr_t);
    curr_z.toVector(curr->second);
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

  std::ifstream in_imu(filename_imu.c_str());
  std::ifstream in_vicon(filename_vicon.c_str());

  VectorSerie samples_imu, samples_vicon;
  const bool input_ok = readInputsAndMeasurements(in_imu, in_vicon,
                                                  &samples_imu, &samples_vicon);
  std::cout << samples_imu.size() << " imu readings." << std::endl;
  std::cout << samples_vicon.size() << " vicon readings." << std::endl;
  if ( ! input_ok )
    return 1;

  computeVelocityMeasurements(&samples_vicon);

  pose_twist_meskf::PoseTwistMESKF filter;
  filter.setUpSystem(1e-2,1e-2,1e-4,1e-4);

  filter.setUpMeasurementModels();

  const double t = samples_vicon.begin()->first;
  pose_twist_meskf::VisualMeasurementVector measurement;
  measurement.fromVector(samples_vicon.begin()->second);
  pose_twist_meskf::InputVector input;
  input.fromVector(samples_imu.begin()->second);
  pose_twist_meskf::NominalStateVector nominal_state;
  nominal_state.position_ = measurement.position_;
  nominal_state.orientation_ = measurement.orientation_;
  nominal_state.lin_vel_ = measurement.lin_vel_;
  nominal_state.lin_acc_ = input.lin_acc_;
  nominal_state.ang_vel_ = input.ang_vel_;
  nominal_state.acc_bias_ = Eigen::Vector3d::Zero();
  nominal_state.gyro_drift_ = Eigen::Vector3d::Zero();
  pose_twist_meskf::PoseTwistMESKF::SymmetricMatrix P(pose_twist_meskf::ErrorStateVector::DIMENSION,0.0);
  for (int i=1; i<pose_twist_meskf::ErrorStateVector::DIMENSION; i++)
    P(i,i) = 1e-4;
  pose_twist_meskf::PoseTwistMESKF::Vector x(pose_twist_meskf::NominalStateVector::DIMESION);
  nominal_state.toVector(x);
  filter.initialize(x, P, t);
}
