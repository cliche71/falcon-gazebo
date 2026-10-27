#!/bin/bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source env.sh
catkin_make -j2 -l2 -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 '-DCATKIN_WHITELIST_PACKAGES=quadrotor_msgs;exploration_utils;voxel_mapping;pathfinding;exploration_preprocessing;trajectory;fast_planner;exploration_manager;falcon_native' -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
