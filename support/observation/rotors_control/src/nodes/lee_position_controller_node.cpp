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

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "lee_position_controller_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {
// TURN_OBSERVATION_BEGIN
static turn_observation::Sink& turn_node_sink(){static turn_observation::Sink sink("controller");return sink;}
// TURN_OBSERVATION_END


LeePositionControllerNode::LeePositionControllerNode(
  const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  :nh_(nh),
   private_nh_(private_nh){
  InitializeParams();

  cmd_pose_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_POSE, 1,
      &LeePositionControllerNode::CommandPoseCallback, this);

  cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &LeePositionControllerNode::MultiDofJointTrajectoryCallback, this);

  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &LeePositionControllerNode::OdometryCallback, this);

  motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  command_timer_ = nh_.createTimer(ros::Duration(0), &LeePositionControllerNode::TimedCommandCallback, this,
                                  true, false);
}

LeePositionControllerNode::~LeePositionControllerNode() { }

void LeePositionControllerNode::InitializeParams() {

  // Read parameters from rosparam.
  GetRosParameter(private_nh_, "position_gain/x",
                  lee_position_controller_.controller_parameters_.position_gain_.x(),
                  &lee_position_controller_.controller_parameters_.position_gain_.x());
  GetRosParameter(private_nh_, "position_gain/y",
                  lee_position_controller_.controller_parameters_.position_gain_.y(),
                  &lee_position_controller_.controller_parameters_.position_gain_.y());
  GetRosParameter(private_nh_, "position_gain/z",
                  lee_position_controller_.controller_parameters_.position_gain_.z(),
                  &lee_position_controller_.controller_parameters_.position_gain_.z());
  GetRosParameter(private_nh_, "velocity_gain/x",
                  lee_position_controller_.controller_parameters_.velocity_gain_.x(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.x());
  GetRosParameter(private_nh_, "velocity_gain/y",
                  lee_position_controller_.controller_parameters_.velocity_gain_.y(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.y());
  GetRosParameter(private_nh_, "velocity_gain/z",
                  lee_position_controller_.controller_parameters_.velocity_gain_.z(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.z());
  GetRosParameter(private_nh_, "attitude_gain/x",
                  lee_position_controller_.controller_parameters_.attitude_gain_.x(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(private_nh_, "attitude_gain/y",
                  lee_position_controller_.controller_parameters_.attitude_gain_.y(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(private_nh_, "attitude_gain/z",
                  lee_position_controller_.controller_parameters_.attitude_gain_.z(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(private_nh_, "angular_rate_gain/x",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(private_nh_, "angular_rate_gain/y",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(private_nh_, "angular_rate_gain/z",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(private_nh_, &lee_position_controller_.vehicle_parameters_);
  lee_position_controller_.InitializeParameters();
}
void LeePositionControllerNode::Publish() {
}

void LeePositionControllerNode::CommandPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {
// TURN_OBSERVATION_BEGIN
  const double turn_receipt_mono=turn_observation::mono(),turn_receipt_ros=ros::Time::now().toSec();
// TURN_OBSERVATION_END

  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference);

  lee_position_controller_.SetTrajectoryPoint(commands_.front());
// TURN_OBSERVATION_BEGIN
  if(turn_observation::enabled()){++turn_reference_seq_;turn_reference_stamp_=pose_msg->header.stamp.toSec();turn_reference_receipt_mono_=turn_receipt_mono;turn_reference_receipt_ros_=turn_receipt_ros;turn_reference_kind_=2;turn_point_index_=0;turn_reference_selected_mono_=turn_observation::mono();turn_reference_selected_ros_=ros::Time::now().toSec();}
// TURN_OBSERVATION_END

  commands_.pop_front();
// TURN_OBSERVATION_BEGIN
  if(turn_observation::enabled()){turn_observation::Packet p;p.kind=2;p.seq=turn_reference_seq_;p.monotonic=turn_observation::mono();p.wall_time=turn_observation::wall();p.sim=ros::Time::now().toSec();
  p.add(turn_reference_stamp_);p.add(turn_reference_receipt_mono_);p.add(turn_reference_receipt_ros_);p.add(turn_reference_selected_mono_);p.add(turn_reference_selected_ros_);p.add(turn_reference_kind_);p.add(turn_point_index_);turn_node_sink().submit(p);}
// TURN_OBSERVATION_END

}

void LeePositionControllerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
// TURN_OBSERVATION_BEGIN
  const double turn_receipt_mono=turn_observation::mono(),turn_receipt_ros=ros::Time::now().toSec();
// TURN_OBSERVATION_END

  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  for (size_t i = 1; i < n_commands; ++i) {
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];

    mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

    commands_.push_back(eigen_reference);
    command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
  }

  // We can trigger the first command immediately.
  lee_position_controller_.SetTrajectoryPoint(commands_.front());
// TURN_OBSERVATION_BEGIN
  if(turn_observation::enabled()){++turn_reference_seq_;turn_reference_stamp_=msg->header.stamp.toSec();turn_reference_receipt_mono_=turn_receipt_mono;turn_reference_receipt_ros_=turn_receipt_ros;turn_reference_kind_=1;turn_point_index_=0;turn_reference_selected_mono_=turn_observation::mono();turn_reference_selected_ros_=ros::Time::now().toSec();}
// TURN_OBSERVATION_END

  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
// TURN_OBSERVATION_BEGIN
  if(turn_observation::enabled()){turn_observation::Packet p;p.kind=2;p.seq=turn_reference_seq_;p.monotonic=turn_observation::mono();p.wall_time=turn_observation::wall();p.sim=ros::Time::now().toSec();
  p.add(turn_reference_stamp_);p.add(turn_reference_receipt_mono_);p.add(turn_reference_receipt_ros_);p.add(turn_reference_selected_mono_);p.add(turn_reference_selected_ros_);p.add(turn_reference_kind_);p.add(turn_point_index_);turn_node_sink().submit(p);}
// TURN_OBSERVATION_END

}

void LeePositionControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  lee_position_controller_.SetTrajectoryPoint(commands_.front());
// TURN_OBSERVATION_BEGIN
  if(turn_observation::enabled()){++turn_point_index_;turn_reference_selected_mono_=turn_observation::mono();turn_reference_selected_ros_=ros::Time::now().toSec();}
// TURN_OBSERVATION_END

  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
// TURN_OBSERVATION_BEGIN
  if(turn_observation::enabled()){turn_observation::Packet p;p.kind=2;p.seq=turn_reference_seq_;p.monotonic=turn_observation::mono();p.wall_time=turn_observation::wall();p.sim=ros::Time::now().toSec();
  p.add(turn_reference_stamp_);p.add(turn_reference_receipt_mono_);p.add(turn_reference_receipt_ros_);p.add(turn_reference_selected_mono_);p.add(turn_reference_selected_ros_);p.add(turn_reference_kind_);p.add(turn_point_index_);turn_node_sink().submit(p);}
// TURN_OBSERVATION_END

}

void LeePositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
// TURN_OBSERVATION_BEGIN
  const double turn_callback_mono=turn_observation::mono(),turn_callback_ros=ros::Time::now().toSec();
// TURN_OBSERVATION_END


  ROS_INFO_ONCE("LeePositionController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  lee_position_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
// TURN_OBSERVATION_BEGIN
  const double turn_compute_begin=turn_observation::mono();
// TURN_OBSERVATION_END
  lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);
// TURN_OBSERVATION_BEGIN
  const double turn_compute_end=turn_observation::mono();
// TURN_OBSERVATION_END


  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_.publish(actuator_msg);
// TURN_OBSERVATION_BEGIN
  if(turn_observation::enabled()){
  const double turn_capture_begin=turn_observation::mono();const auto& t=lee_position_controller_.GetTurnObservation();turn_observation::Packet p;p.kind=1;p.seq=++turn_compute_seq_;p.monotonic=turn_callback_mono;p.wall_time=turn_observation::wall();p.sim=turn_callback_ros;
  p.add(turn_reference_seq_);p.add(turn_reference_stamp_);p.add(turn_reference_receipt_mono_);p.add(turn_reference_receipt_ros_);p.add(turn_reference_selected_mono_);p.add(turn_reference_selected_ros_);p.add(turn_reference_kind_);p.add(turn_point_index_);
  p.add(odometry_msg->header.stamp.toSec());p.add(odometry_msg->header.seq);p.add(turn_compute_begin);p.add(turn_compute_end);p.add(t.active);
  p.vec(t.p);p.vec(t.v);p.vec(t.a);p.add(t.yaw);p.add(t.yaw_rate);p.vec(t.odom_p);p.vec(t.odom_q);p.vec(t.odom_v);p.vec(t.odom_w);
  p.vec(t.accel);p.mat(t.R_des);p.mat(t.R);p.vec(t.pos_error);p.vec(t.vel_error);p.vec(t.angle_error);p.vec(t.w_des);p.vec(t.w_error);p.vec(t.angular_acc);p.add(t.thrust);p.vec(t.requested_torque);
  p.add(t.motor_target.size());p.vec(t.squared_raw);p.vec(t.squared_clamped);p.vec(t.motor_target);p.add(t.copy_ns);p.add((turn_observation::mono()-turn_capture_begin)*1e9);
  turn_node_sink().submit(p);}
// TURN_OBSERVATION_END

}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lee_position_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  rotors_control::LeePositionControllerNode lee_position_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}
