#include "fast_planner/trajectory_consistency.h"
#include "voxel_mapping/targeted_diagnostics.h"
#include <fstream>
using namespace fast_planner;
using J=Json::Value;
NonUniformBspline readSpline(const J& v) {
  Eigen::MatrixXd cp(v["control_points"].size(),v["control_points"][0].size());
  for(int i=0;i<cp.rows();++i)for(int k=0;k<cp.cols();++k)cp(i,k)=v["control_points"][i][k].asDouble();
  Eigen::VectorXd u(v["knots"].size());for(int i=0;i<u.size();++i)u[i]=v["knots"][i].asDouble();
  NonUniformBspline s(cp,u.size()-cp.rows()-1,v["knot_span"].asDouble());s.setKnot(u);return s;
}
int main(int argc,char** argv) {
  if(argc!=3)return 2;
  std::ifstream in(argv[1]);J cases;in>>cases;J output(Json::arrayValue);
  for(const auto& c:cases){LocalTrajData d;d.position_traj_=readSpline(c["position"]);d.yaw_traj_=readSpline(c["yaw"]);d.traj_id_=c["id"].asInt();d.start_time_=ros::Time(c["start_time"].asDouble());
    refreshTrajectoryDerived(d);
    J r;r["id"]=c["id"];r["kind"]=c["kind"];r["duration"]=d.duration_;r["position"]=falcon_diag::spline(d.position_traj_);r["yaw"]=falcon_diag::spline(d.yaw_traj_);r["velocity"]=falcon_diag::spline(d.velocity_traj_);r["acceleration"]=falcon_diag::spline(d.acceleration_traj_);r["yawdot"]=falcon_diag::spline(d.yawdot_traj_);r["yawdotdot"]=falcon_diag::spline(d.yawdotdot_traj_);r["start_pos"]=falcon_diag::vector(d.start_pos_);r["end_pos"]=falcon_diag::vector(d.end_pos_);r["start_yaw"]=d.start_yaw_;r["end_yaw"]=d.end_yaw_;r["start_time"]=d.start_time_.toSec();
    double tr=c.get("prediction_t",0.0).asDouble();r["prediction_t"]=tr;r["predicted_p"]=falcon_diag::vector(d.position_traj_.evaluateDeBoorT(tr));r["predicted_v"]=falcon_diag::vector(d.velocity_traj_.evaluateDeBoorT(tr));r["predicted_a"]=falcon_diag::vector(d.acceleration_traj_.evaluateDeBoorT(tr));
    J z;z["p"]=falcon_diag::vector(d.position_traj_.evaluateDeBoorT(d.duration_));z["v_before_hold"]=falcon_diag::vector(d.velocity_traj_.evaluateDeBoorT(d.duration_));z["a_before_hold"]=falcon_diag::vector(d.acceleration_traj_.evaluateDeBoorT(d.duration_));z["yaw_before_hold"]=d.yaw_traj_.evaluateDeBoorT(d.duration_)[0];z["yawdot_before_hold"]=d.yawdot_traj_.evaluateDeBoorT(d.duration_)[0];r["natural_endpoint"]=z;
    output.append(std::move(r));
  }
  std::ofstream out(argv[2]);Json::StreamWriterBuilder b;b["precision"]=17;out<<Json::writeString(b,output);return 0;
}
