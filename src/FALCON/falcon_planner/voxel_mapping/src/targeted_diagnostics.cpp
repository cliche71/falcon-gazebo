#include "voxel_mapping/targeted_diagnostics.h"
#include <ros/ros.h>
#include <atomic>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unistd.h>
namespace falcon_diag {
namespace {
const char* path=std::getenv("FALCON_TARGETED_DIAG_DIR");
std::atomic<uint64_t> drops{0},errors{0},submitted{0},written{0},max_queue{0};
std::atomic<uint64_t> mb{0},me{0},started_stamp{0},finished_stamp{0};
std::mutex mutex;std::condition_variable cv;std::deque<J> queue;
std::thread writer;bool closing=false;bool initialized=false;
thread_local int attempt=-1,tid=-1;thread_local double start=0.;thread_local const char* current_phase="unassigned";
int next_attempt=0;std::unordered_map<int,int> associations;
std::mutex stats_mutex;struct Stat{uint64_t count=0;double sum=0,max=0;};std::unordered_map<std::string,Stat> stats;
std::ofstream out;
void write(const J& r){Json::StreamWriterBuilder b;b["indentation"]="";b["precision"]=17;out<<Json::writeString(b,r)<<'\n';++written;}
void worker(){
  for(;;){J r;{std::unique_lock<std::mutex> l(mutex);cv.wait(l,[]{return closing||!queue.empty();});if(queue.empty()&&closing)break;r=std::move(queue.front());queue.pop_front();}write(r);}
  out.flush();
}
}
bool enabled() noexcept{return path&&*path;}
double mono() noexcept{return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
void failure() noexcept{++errors;}
void overhead(const char* name,double seconds) noexcept{try{std::unique_lock<std::mutex> l(stats_mutex,std::try_to_lock);if(l.owns_lock()){auto& s=stats[name];++s.count;s.sum+=seconds;s.max=std::max(s.max,seconds);}}catch(...){++errors;}}
void initialize() noexcept{if(!enabled())return;try{out.open(std::string(path)+"/diagnostics.jsonl");if(!out){++errors;return;}initialized=true;writer=std::thread(worker);}catch(...){++errors;}}
void shutdown() noexcept{if(!initialized)return;try{{std::lock_guard<std::mutex> l(mutex);closing=true;}cv.notify_one();if(writer.joinable())writer.join();J s;s["submitted"]=Json::UInt64(submitted.load());s["written"]=Json::UInt64(written.load());s["queue_drops"]=Json::UInt64(drops.load());s["errors"]=Json::UInt64(errors.load());s["max_queue"]=Json::UInt64(max_queue.load());s["capacity_records"]=128;for(const auto& x:stats){s["overhead"][x.first]["count"]=Json::UInt64(x.second.count);s["overhead"][x.first]["sum_seconds"]=x.second.sum;s["overhead"][x.first]["max_seconds"]=x.second.max;}std::ofstream f(std::string(path)+"/diagnostic_stats.json");f<<s;out.close();initialized=false;}catch(...){++errors;}}
void beginAttempt(int source_id,double source_start) noexcept{attempt=++next_attempt;tid=-1;start=source_start;current_phase="plan";}
void trajectory(int id,double source_start) noexcept{tid=id;start=source_start;if(associations.count(id))attempt=associations[id];else if(id>0&&attempt>0)associations[id]=attempt;}
void phase(const char* s) noexcept{current_phase=s;}
Epoch epoch() noexcept{return {mb.load(),me.load(),finished_stamp.load()};}
J mapIdentity(){J m;m["updates_started"]=Json::UInt64(mb.load());m["updates_completed"]=Json::UInt64(me.load());m["last_started_input_ns"]=Json::UInt64(started_stamp.load());m["last_completed_input_ns"]=Json::UInt64(finished_stamp.load());m["original_queries_are_not_globally_locked"]=true;return m;}
void mapBegin(uint64_t stamp) noexcept{started_stamp=stamp;++mb;}
void mapEnd(uint64_t stamp) noexcept{finished_stamp=stamp;++me;}
J record(const char* stage){J r;r["run_id"]=std::string(path?path:"");r["plan_attempt_id"]=attempt;r["trajectory_id"]=tid;r["phase"]=current_phase;r["stage"]=stage;r["ros_time"]=ros::Time::now().toSec();r["monotonic"]=mono();r["node"]=ros::this_node::getName();r["pid"]=int(getpid());r["thread"]=Json::UInt64(std::hash<std::thread::id>{}(std::this_thread::get_id()));r["current_trajectory_start_time"]=start;return r;}
void emit(J r) noexcept{if(!initialized)return;try{++submitted;std::unique_lock<std::mutex> l(mutex,std::try_to_lock);if(!l.owns_lock()||queue.size()>=128){++drops;return;}queue.push_back(std::move(r));max_queue.store(std::max<uint64_t>(max_queue.load(),queue.size()));l.unlock();cv.notify_one();}catch(...){++errors;}}
}
