#pragma once
#include "fast_planner/planner_manager.h"
#include "voxel_mapping/targeted_diagnostics.h"
namespace fast_planner {
inline void diagnosticTrajectory(LocalTrajData& d,const char* stage) {
  FALCON_DIAG(
    falcon_diag::Timer timer("trajectory_snapshot");
    falcon_diag::trajectory(d.traj_id_,d.start_time_.toSec());
    auto r=falcon_diag::record(stage);r["duration_cache"]=d.duration_;
    r["position"]=falcon_diag::spline(d.position_traj_);r["velocity_cache"]=falcon_diag::spline(d.velocity_traj_);
    r["acceleration_cache"]=falcon_diag::spline(d.acceleration_traj_);r["yaw"]=falcon_diag::spline(d.yaw_traj_);
    r["yawdot_cache"]=falcon_diag::spline(d.yawdot_traj_);r["yawdotdot_cache"]=falcon_diag::spline(d.yawdotdot_traj_);
    r["initial_position"]=falcon_diag::spline(d.init_traj_bspline_);r["map_identity"]=falcon_diag::mapIdentity();
    falcon_diag::emit(std::move(r));
  );
}
}
