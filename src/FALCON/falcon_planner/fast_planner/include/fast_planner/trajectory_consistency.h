#pragma once
#include "fast_planner/planner_manager.h"
namespace fast_planner {
// Refresh only derived data. Curve, clock, identifiers and endpoint-hold policy stay intact.
inline void refreshTrajectoryDerived(LocalTrajData& d) {
  d.velocity_traj_ = d.position_traj_.getDerivative();
  d.acceleration_traj_ = d.velocity_traj_.getDerivative();
  d.yawdot_traj_ = d.yaw_traj_.getDerivative();
  d.yawdotdot_traj_ = d.yawdot_traj_.getDerivative();
  d.duration_ = d.position_traj_.getTimeSum();
  d.start_pos_ = d.position_traj_.evaluateDeBoorT(0.0);
  d.end_pos_ = d.position_traj_.evaluateDeBoorT(d.duration_);
  d.start_yaw_ = d.yaw_traj_.evaluateDeBoorT(0.0)[0];
  d.end_yaw_ = d.yaw_traj_.evaluateDeBoorT(d.duration_)[0];
}
}
