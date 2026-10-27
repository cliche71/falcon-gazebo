#!/bin/bash
FALCON_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for falcon_old in ${!FALCON_@}; do
 case "$falcon_old" in FALCON_WORKSPACE|FALCON_ROTORS_WS|FALCON_OBSERVER_WS|FALCON_LOG_DIR) ;; *) unset "$falcon_old" ;; esac
done
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH GAZEBO_PLUGIN_PATH
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/noetic/setup.bash
FALCON_ROTORS_WS="${FALCON_ROTORS_WS:-$(dirname "$FALCON_WORKSPACE")/rotors_ws}"
source "$FALCON_ROTORS_WS/devel/setup.bash" --extend
if [ -f "$FALCON_WORKSPACE/devel/setup.bash" ]; then source "$FALCON_WORKSPACE/devel/setup.bash" --extend; fi
FALCON_OBSERVER_WS="${FALCON_OBSERVER_WS:-$(dirname "$FALCON_WORKSPACE")/falcon_turn_observation_ws}"
source "$FALCON_OBSERVER_WS/devel/setup.bash" --extend
export GAZEBO_PLUGIN_PATH="$FALCON_OBSERVER_WS/devel/lib:$FALCON_WORKSPACE/devel/lib:$FALCON_ROTORS_WS/devel/lib:/opt/ros/noetic/lib"
for falcon_var in CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH; do
 falcon_result=""
 IFS=: read -ra falcon_parts <<< "${!falcon_var}"
 for falcon_part in "${falcon_parts[@]}"; do
  case "$falcon_part" in /home/ljz/falcon_gazebo_native_ws*|/home/ljz/falcon_short_segment_repair_ws*|/home/ljz/falcon_gazebo_ws*|/home/ljz/falcon_ws*) continue;; esac
  [ -z "$falcon_part" ] || falcon_result="${falcon_result:+$falcon_result:}$falcon_part"
 done
 export "$falcon_var=$falcon_result"
done
unset falcon_old falcon_var falcon_result falcon_parts falcon_part
export ROS_HOSTNAME=localhost
export PYTHONDONTWRITEBYTECODE=1
