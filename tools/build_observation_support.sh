#!/bin/bash
set -e
falcon_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
falcon_support="${1:?Provide a NEW empty support workspace}"
[ ! -e "$falcon_support" ] || { echo 'Target must not exist'; exit 2; }
mkdir -p "$falcon_support/src"
cp -a "$falcon_root/support/observation/rotors_control" "$falcon_support/src/"
cp -a "$falcon_root/support/observation/rotors_gazebo_plugins" "$falcon_support/src/"
source /opt/ros/noetic/setup.bash
source "${FALCON_ROTORS_WS:-$(dirname "$falcon_root")/rotors_ws}/devel/setup.bash" --extend
cd "$falcon_support"
catkin_make -j2 -l2 -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --make-args lee_position_controller_node rotors_gazebo_motor_model rotors_gazebo_ros_interface_plugin -j2
