#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <memory>
#include <cstdint>
#include <string>
#include <thread>
#include <unistd.h>
namespace turn_observation {
inline double mono(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
inline double wall(){return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();}
inline bool enabled(){static const bool value=std::getenv("FALCON_TURN_TRACE_DIR")!=nullptr;return value;}
inline double tap(double value,double& copy){copy=value;return value;}
struct Packet{
 int kind=0,n=0;unsigned long long seq=0;double monotonic=0,wall_time=0,sim=0;std::array<double,128> d{};
 void add(double v){if(n<int(d.size()))d[n++]=v;}
 template<class V> void vec(const V& v){for(int i=0;i<v.size();++i)add(v(i));}
 template<class M> void mat(const M& m){for(int i=0;i<m.rows();++i)for(int j=0;j<m.cols();++j)add(m(i,j));}
};
class Sink{
 static const size_t capacity_=8192;
 struct Cell{std::atomic<size_t> sequence;Packet packet;};
 std::unique_ptr<Cell[]> ring_;std::atomic<size_t> enqueue_pos_{0};size_t dequeue_pos_=0;
 std::thread worker_;std::atomic<bool> done_{false};std::ofstream out_;
 std::string path_;std::atomic<unsigned long long> submitted_{0},written_{0},dropped_{0},errors_{0},enqueue_ns_{0},enqueue_max_ns_{0};
 bool enqueue(const Packet& p){size_t pos=enqueue_pos_.load(std::memory_order_relaxed);Cell* cell;
  for(;;){cell=&ring_[pos%capacity_];size_t seq=cell->sequence.load(std::memory_order_acquire);intptr_t dif=static_cast<intptr_t>(seq)-static_cast<intptr_t>(pos);
   if(dif==0){if(enqueue_pos_.compare_exchange_weak(pos,pos+1,std::memory_order_relaxed))break;}
   else if(dif<0)return false;else pos=enqueue_pos_.load(std::memory_order_relaxed);}
  cell->packet=p;cell->sequence.store(pos+1,std::memory_order_release);return true;}
 bool dequeue(Packet& p){Cell& cell=ring_[dequeue_pos_%capacity_];size_t seq=cell.sequence.load(std::memory_order_acquire);
  if(static_cast<intptr_t>(seq)-static_cast<intptr_t>(dequeue_pos_+1)!=0)return false;
  p=cell.packet;cell.sequence.store(dequeue_pos_+capacity_,std::memory_order_release);++dequeue_pos_;return true;}
 void stats(){std::ofstream f(path_+".stats.tmp");f<<"{\"submitted\":"<<submitted_<<",\"written\":"<<written_<<",\"dropped\":"<<dropped_<<",\"write_errors\":"<<errors_<<",\"enqueue_ns_sum\":"<<enqueue_ns_<<",\"enqueue_ns_max\":"<<enqueue_max_ns_<<"}";f.close();std::rename((path_+".stats.tmp").c_str(),(path_+".stats.json").c_str());}
 void run(){double last_stats=mono(),last_flush=mono();for(;;){Packet p;bool any=false;
  while(dequeue(p)){any=true;out_<<p.kind<<','<<p.seq<<','<<std::setprecision(17)<<p.monotonic<<','<<p.wall_time<<','<<p.sim<<','<<p.n;for(int i=0;i<p.n;++i)out_<<','<<p.d[i];out_<<'\n';++written_;if(!out_)++errors_;}
  if(mono()-last_flush>=.1){out_.flush();last_flush=mono();}
  if(mono()-last_stats>=.5){stats();last_stats=mono();}
  if(done_.load()&&written_.load()+dropped_.load()==submitted_.load())break;
  if(!any)std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  out_.flush();stats();}
 public:
 explicit Sink(const std::string& name){if(!enabled())return;ring_.reset(new Cell[capacity_]);for(size_t i=0;i<capacity_;++i)ring_[i].sequence.store(i,std::memory_order_relaxed);
  path_=std::string(std::getenv("FALCON_TURN_TRACE_DIR"))+"/"+name;out_.open(path_+".csv",std::ios::out|std::ios::trunc);out_<<"kind,sequence,monotonic,wall,processing_sim,field_count,fields...\n";if(!out_)++errors_;worker_=std::thread([this]{run();});}
 ~Sink(){if(!worker_.joinable())return;done_.store(true);worker_.join();}
 void submit(const Packet& p){if(!enabled())return;double start=mono();++submitted_;if(!enqueue(p))++dropped_;
  auto ns=static_cast<unsigned long long>((mono()-start)*1e9);enqueue_ns_+=ns;auto old=enqueue_max_ns_.load();while(old<ns&&!enqueue_max_ns_.compare_exchange_weak(old,ns)){} }
};
}
