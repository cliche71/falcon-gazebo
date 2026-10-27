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

#ifndef ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H
#define ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/common.h"
#include "rotors_control/parameters.h"
// TURN_OBSERVATION_BEGIN
#include "turn_observation/trace.h"
// TURN_OBSERVATION_END


namespace rotors_control {

// Default values for the lee position controller and the Asctec Firefly.
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(6, 6, 6);
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 3, 0.035);
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 0.52, 0.025);

class LeePositionControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LeePositionControllerParameters()
      : position_gain_(kDefaultPositionGain),
        velocity_gain_(kDefaultVelocityGain),
        attitude_gain_(kDefaultAttitudeGain),
        angular_rate_gain_(kDefaultAngularRateGain) {
    calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

  Eigen::Matrix4Xd allocation_matrix_;
  Eigen::Vector3d position_gain_;
  Eigen::Vector3d velocity_gain_;
  Eigen::Vector3d attitude_gain_;
  Eigen::Vector3d angular_rate_gain_;
  RotorConfiguration rotor_configuration_;
};

// TURN_OBSERVATION_BEGIN
struct TurnObservation {
 bool active=false;double copy_ns=0,thrust=0;
 Eigen::Vector3d p=Eigen::Vector3d::Zero(),v=Eigen::Vector3d::Zero(),a=Eigen::Vector3d::Zero();double yaw=0,yaw_rate=0;
 Eigen::Vector3d odom_p=Eigen::Vector3d::Zero(),odom_v=Eigen::Vector3d::Zero(),odom_w=Eigen::Vector3d::Zero();Eigen::Vector4d odom_q=Eigen::Vector4d::Zero();
 Eigen::Vector3d pos_error=Eigen::Vector3d::Zero(),vel_error=Eigen::Vector3d::Zero(),accel=Eigen::Vector3d::Zero(),angle_error=Eigen::Vector3d::Zero(),w_des=Eigen::Vector3d::Zero(),w_error=Eigen::Vector3d::Zero(),angular_acc=Eigen::Vector3d::Zero();
 Eigen::Matrix3d R_des=Eigen::Matrix3d::Identity(),R=Eigen::Matrix3d::Identity();
 Eigen::Vector3d requested_torque=Eigen::Vector3d::Zero();Eigen::VectorXd squared_raw,squared_clamped,motor_target;
};
// TURN_OBSERVATION_END

class LeePositionController {
 public:
  LeePositionController();
  ~LeePositionController();
  void InitializeParameters();
  void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;

  void SetOdometry(const EigenOdometry& odometry);
  void SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory);

// TURN_OBSERVATION_BEGIN
  const TurnObservation& GetTurnObservation() const{return turn_observation_;}
// TURN_OBSERVATION_END
  LeePositionControllerParameters controller_parameters_;
  VehicleParameters vehicle_parameters_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
// TURN_OBSERVATION_BEGIN
  mutable TurnObservation turn_observation_;
// TURN_OBSERVATION_END
  bool initialized_params_;
  bool controller_active_;

  Eigen::Vector3d normalized_attitude_gain_;
  Eigen::Vector3d normalized_angular_rate_gain_;
  Eigen::MatrixX4d angular_acc_to_rotor_velocities_;

  mav_msgs::EigenTrajectoryPoint command_trajectory_;
  EigenOdometry odometry_;

  void ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                Eigen::Vector3d* angular_acceleration) const;
  void ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) const;
};
}

#endif // ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H
