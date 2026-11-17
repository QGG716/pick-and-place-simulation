// Directed real-header contracts; no ROS runtime or devices.
#include "sc_composite.h"
#include <cassert>
#include <iostream>
#include <sys/stat.h>
#include <sys/sysmacros.h>
using J=nlohmann::json;
int main(int argc,char**argv) {
  const std::string mode=argc>1?argv[1]:"limits";
  if(mode=="limits") {
    m710::PrepareInput in{{0.,1.},std::vector<double>(12,0.),std::vector<double>(12,0.),std::vector<double>(12,0.)};
    m710::PrepareLimits good{std::vector<double>(6,-1.),std::vector<double>(6,1.),std::vector<double>(6,.1),std::vector<double>(6,.2)};
    for(int i=0;i<7;++i) {
      auto bad=good;
      if(i==0)bad.lower[0]=bad.upper[0]=0.;
      if(i==1)bad.lower[0]=-INFINITY;
      if(i==2)bad.upper[0]=NAN;
      if(i==3)bad.velocity[0]=0;
      if(i==4)bad.acceleration[0]=-1;
      if(i==5)bad.lower.pop_back();
      if(i==6)bad.velocity[0]=INFINITY;
      bool rejected=false;
      try {m710::nativePrepare(in,bad,1.,.005,1.);} catch(const std::runtime_error&e) {rejected=std::string(e.what())=="INVALID_LIMITS";}
      if(!rejected){std::cerr<<"INVALID_LIMITS case "<<i<<" accepted\n";return 1;}
    }
    assert(m710::nativePrepare(in,good,1.,.005,1.).t.size()==201);
  } else {
    const auto dir="/tmp/eco65-review-native-"+std::to_string(getpid());mkdir(dir.c_str(),0700);setenv("M710_WORKER_JOURNAL_DIR",dir.c_str(),1);
    J req={{"request_id","test"},{"task_id","task"},{"journal_directory",dir}};
    int calls=0;
    if(mode=="journal") {
      bool rejected=false;
      try {auto bad=req;bad["journal_directory"]=dir+"/missing";setenv("M710_WORKER_JOURNAL_DIR",(dir+"/missing").c_str(),1);m710::CompositeRunner runner([&](const J&){++calls;return J();},[]{},nullptr,bad);}catch(const std::runtime_error&e){rejected=std::string(e.what()).find("JOURNAL_OPEN_FAILED")!=std::string::npos;}
      assert(rejected&&calls==0);setenv("M710_WORKER_JOURNAL_DIR",dir.c_str(),1);
      auto filename=dir+"/m710_worker_journal_"+std::to_string(getpid())+".jsonl";
      assert(mknod(filename.c_str(),S_IFCHR|0600,makedev(1,7))==0);rejected=false;
      try {m710::CompositeRunner runner([&](const J&){++calls;return J{{"status","SUCCESS"}};},[]{},nullptr,req);runner.call(J{{"op","validate"}},"check",120);}catch(const std::runtime_error&e){rejected=std::string(e.what()).find("JOURNAL_WRITE_FAILED")!=std::string::npos;}
      assert(rejected&&calls==0);unlink(filename.c_str());
      m710::CompositeRunner* ptr=nullptr;
      m710::CompositeRunner runner([&](const J&){++calls;ptr->journal.setstate(std::ios::badbit);return J{{"status","SUCCESS"}};},[]{},nullptr,req);ptr=&runner;rejected=false;
      try {runner.call(J{{"op","validate"}},"end",120);}catch(const std::runtime_error&){rejected=true;}
      assert(rejected&&calls==1&&runner.calls.empty());unlink(filename.c_str());
    } else if(mode=="path") {
      m710::CompositeRunner runner([](const J&){return J{{"status","SUCCESS"}};},[]{},nullptr,req);
      J path={{0.,0.,0.,0.,0.,0.},{0.,.1,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.}};
      runner.call(J{{"op","validate"},{"probe_path",path}},"path",120);
      assert(runner.calls[0]["request"]["probe_path"]==path);unlink(runner.journal_path.c_str());
    }
    rmdir(dir.c_str());
  }
  std::cout<<mode<<" PASS\n";
}
