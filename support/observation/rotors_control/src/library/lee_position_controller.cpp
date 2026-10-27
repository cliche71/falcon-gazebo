/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rotors_control/lee_position_controller.h"

namespace rotors_control {

LeePositionController::LeePositionController()
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
}

LeePositionController::~LeePositionController() {}

void LeePositionController::InitializeParameters() {
  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_attitude_gain_ = controller_parameters_.attitude_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_angular_rate_gain_ = controller_parameters_.angular_rate_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();

  Eigen::Matrix4d I;
  I.setZero();
  I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
  I(3, 3) = 1;
  angular_acc_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // A^{ \dagger} = A^T*(A*A^T)^{-1}
  angular_acc_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse() * I;
  initialized_params_ = true;
}

void LeePositionController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
  assert(rotor_velocities);
  assert(initialized_params_);

// TURN_OBSERVATION_BEGIN
  if(turn_observation::enabled()){turn_observation_.copy_ns=0;turn_observation_.active=controller_active_;}
// TURN_OBSERVATION_END
  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }

// TURN_OBSERVATION_BEGIN
  if(turn_observation::enabled()){const double turn_begin=turn_observation::mono();
  turn_observation_.p=command_trajectory_.position_W;turn_observation_.v=command_trajectory_.velocity_W;turn_observation_.a=command_trajectory_.acceleration_W;
  turn_observation_.yaw=command_trajectory_.getYaw();turn_observation_.yaw_rate=command_trajectory_.getYawRate();
  turn_observation_.odom_p=odometry_.position;turn_observation_.odom_v=odometry_.velocity;turn_observation_.odom_w=odometry_.angular_velocity;turn_observation_.odom_q=odometry_.orientation.coeffs();
  turn_observation_.copy_ns+=(turn_observation::mono()-turn_begin)*1e9;}
// TURN_OBSERVATION_END
  Eigen::Vector3d acceleration;
  ComputeDesiredAcceleration(&acceleration);

  Eigen::Vector3d angular_acceleration;
  ComputeDesiredAngularAcc(acceleration, &angular_acceleration);

  // Project thrust onto body z axis.
  double thrust = -vehicle_parameters_.mass_ * acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));

  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
  angular_acceleration_thrust(3) = thrust;

  *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
// TURN_OBSERVATION_BEGIN
  if(turn_observation::enabled()){const double turn_begin=turn_observation::mono();
  turn_observation_.thrust=thrust;turn_observation_.requested_torque=vehicle_parameters_.inertia_*angular_acceleration;turn_observation_.squared_raw=*rotor_velocities;
  turn_observation_.copy_ns+=(turn_observation::mono()-turn_begin)*1e9;}
// TURN_OBSERVATION_END

  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
// TURN_OBSERVATION_BEGIN
  if(turn_observation::enabled()){const double turn_begin=turn_observation::mono();
  turn_observation_.squared_clamped=*rotor_velocities;
  turn_observation_.copy_ns+=(turn_observation::mono()-turn_begin)*1e9;}
// TURN_OBSERVATION_END

  *rotor_velocities = rotor_velocities->cwiseSqrt();
// TURN_OBSERVATION_BEGIN
  if(turn_observation::enabled()){const double turn_begin=turn_observation::mono();
  turn_observation_.motor_target=*rotor_velocities;
  turn_observation_.copy_ns+=(turn_observation::mono()-turn_begin)*1e9;}
// TURN_OBSERVATION_END

}

void LeePositionController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void LeePositionController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

void LeePositionController::ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) const {
  assert(acceleration);

  Eigen::Vector3d position_error;
  position_error = odometry_.position - command_trajectory_.position_W;

  // Transform velocity to world frame.
  const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
  Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;
  Eigen::Vector3d velocity_error;
  velocity_error = velocity_W - command_trajectory_.velocity_W;

  Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());

  *acceleration = (position_error.cwiseProduct(controller_parameters_.position_gain_)
      + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)) / vehicle_parameters_.mass_
      - vehicle_parameters_.gravity_ * e_3 - command_trajectory_.acceleration_W;
// TURN_OBSERVATION_BEGIN
  if(turn_observation::enabled()){const double turn_begin=turn_observation::mono();
  turn_observation_.pos_error=position_error;turn_observation_.vel_error=velocity_error;turn_observation_.accel=*acceleration;
  turn_observation_.copy_ns+=(turn_observation::mono()-turn_begin)*1e9;}
// TURN_OBSERVATION_END

}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void LeePositionController::ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                                     Eigen::Vector3d* angular_acceleration) const {
  assert(angular_acceleration);

  Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();

  // Get the desired rotation matrix.
  Eigen::Vector3d b1_des;
  double yaw = command_trajectory_.getYaw();
  b1_des << cos(yaw), sin(yaw), 0;

  Eigen::Vector3d b3_des;
  b3_des = -acceleration / acceleration.norm();

  Eigen::Vector3d b2_des;
  b2_des = b3_des.cross(b1_des);
  b2_des.normalize();

  Eigen::Matrix3d R_des;
  R_des.col(0) = b2_des.cross(b3_des);
  R_des.col(1) = b2_des;
  R_des.col(2) = b3_des;
// TURN_OBSERVATION_BEGIN
  if(turn_observation::enabled()){const double turn_begin=turn_observation::mono();
  turn_observation_.R=R;turn_observation_.R_des=R_des;
  turn_observation_.copy_ns+=(turn_observation::mono()-turn_begin)*1e9;}
// TURN_OBSERVATION_END


  // Angle error according to lee et al.
  Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
  Eigen::Vector3d angle_error;
  vectorFromSkewMatrix(angle_error_matrix, &angle_error);

  // TODO(burrimi) include angular rate references at some point.
  Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
  angular_rate_des[2] = command_trajectory_.getYawRate();

  Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;

  *angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
                           - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
                           + odometry_.angular_velocity.cross(odometry_.angular_velocity); // we don't need the inertia matrix here
// TURN_OBSERVATION_BEGIN
  if(turn_observation::enabled()){const double turn_begin=turn_observation::mono();
  turn_observation_.angle_error=angle_error;turn_observation_.w_des=angular_rate_des;turn_observation_.w_error=angular_rate_error;turn_observation_.angular_acc=*angular_acceleration;
  turn_observation_.copy_ns+=(turn_observation::mono()-turn_begin)*1e9;}
// TURN_OBSERVATION_END

}
}
