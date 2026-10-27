#pragma once
// Observation only; activated solely by FALCON_TARGETED_DIAG_DIR.
#include <jsoncpp/json/json.h>
#include <Eigen/Core>
#include <chrono>
#include <cstdint>
#include <vector>
namespace falcon_diag {
using J = Json::Value;
bool enabled() noexcept;
void initialize() noexcept;
void shutdown() noexcept;
void failure() noexcept;
double mono() noexcept;
void beginAttempt(int source_id, double source_start) noexcept;
void trajectory(int id, double start) noexcept;
void phase(const char* value) noexcept;
void emit(J record) noexcept;
J record(const char* stage);
struct Epoch { uint64_t begun,completed,input_ns; };
Epoch epoch() noexcept;
J mapIdentity();
void mapBegin(uint64_t stamp) noexcept;
void mapEnd(uint64_t stamp) noexcept;
void overhead(const char* stage, double seconds) noexcept;
struct Timer {
  const char* stage; double begin;
  explicit Timer(const char* s):stage(s),begin(mono()){}
  ~Timer(){ overhead(stage,mono()-begin); }
};
template<class D> J matrix(const Eigen::MatrixBase<D>& a) {
  J v(Json::arrayValue);
  for(int i=0;i<a.rows();++i){J row(Json::arrayValue);for(int k=0;k<a.cols();++k)row.append(a(i,k));v.append(std::move(row));}
  return v;
}
template<class D> J vector(const Eigen::MatrixBase<D>& a) {
  J v(Json::arrayValue);for(int i=0;i<a.size();++i)v.append(a(i/a.cols(),i%a.cols()));return v;
}
template<class T> J array(const std::vector<T>& a) {
  J v(Json::arrayValue);for(const auto& x:a)v.append(x);return v;
}
inline J vectors(const std::vector<Eigen::Vector3d>& a) {
  J v(Json::arrayValue);for(const auto& x:a)v.append(vector(x));return v;
}
template<class S> J spline(S& a) {
  J v;v["control_points"]=matrix(a.getControlPoint());v["knots"]=vector(a.getKnot());
  v["knot_span"]=a.getKnotSpan();return v;
}
}
#define FALCON_DIAG(...) do { if(falcon_diag::enabled()) { try { __VA_ARGS__; } catch(...) { falcon_diag::failure(); } } } while(false)
