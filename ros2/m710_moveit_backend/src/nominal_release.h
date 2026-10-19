#pragma once
#include "coupled_receiver.h"
namespace m710 {
inline ProcessJson nominalDropWorldFailure(const ProcessJson& target,const ProcessJson& world) {
  const auto box=processBox(target);
  for(const auto& other:world) {
    if(other.at("id")==box.id)throw std::runtime_error("RELEASE_DUPLICATE_TARGET");
    const auto b=processBox(other);const double sat=processSignedGap(box,b);
    if(b.id=="conveyor_transverse" && sat>=-1e-9)continue;
    if(sat>.005000001)continue;
    const double gap=receiverBoxDistance(target,other);
    if(gap+1e-9<.005)return {{"reason","NOMINAL_DROP_WORLD_CLEARANCE"},{"pair",{box.id,b.id}},{"gap_m",gap}};
  }
  return nullptr;
}
inline void validateNominalDrop(const ProcessJson& req,const ProcessJson& binding) {
  const auto& spec=req.at("nominal_release");const auto& samples=spec.at("samples");
  if(!coupledBinding(binding) || spec.at("gravity_m_s2")!=9.81 ||
      spec.at("initial_linear_velocity_m_s")!=ProcessJson::array({0.,0.,0.}) ||
      spec.at("initial_angular_velocity_rad_s")!=ProcessJson::array({0.,0.,0.}) ||
      samples.size()<2 || samples.size()>4096 || req.at("start_velocity")!=ProcessJson::array({0.,0.,0.,0.,0.,0.}))
    throw std::runtime_error("NOMINAL_DROP_MODEL_INVALID");
  const auto start=processBox(req.at("process_policy").at("target"));
  const auto first=processBox(samples.front().at("box"));
  if((first.pose.matrix()-start.pose.matrix()).cwiseAbs().maxCoeff()>1e-7 || first.id!=start.id)
    throw std::runtime_error("RELEASE_POSE_JUMP");
  double previous=0.;double last_z=start.pose.translation().z();
  for(size_t i=0;i<samples.size();++i) {
    const auto& s=samples[i];const auto b=processBox(s.at("box"));const double t=s.at("t");
    Eigen::Isometry3d expected=start.pose;expected.translation().z()-=.5*9.81*t*t;
    if(!std::isfinite(t) || (i==0 && t!=0.) || (i>0 && t<=previous) || b.id!=start.id || b.half!=start.half ||
        (expected.matrix()-b.pose.matrix()).cwiseAbs().maxCoeff()>1e-9 ||
        last_z-b.pose.translation().z()>.002000000001)
      throw std::runtime_error("NOMINAL_DROP_SAMPLE_INVALID");
    previous=t;last_z=b.pose.translation().z();
  }
  ProcessBox belt;bool found=false;
  for(const auto& b:req.at("world"))if(b.at("id")=="conveyor_transverse"){belt=processBox(b);found=true;}
  if(!found)throw std::runtime_error("NOMINAL_DROP_RECEIVER_MISSING");
  const auto final=processBox(samples.back().at("box"));
  const double bottom=final.pose.translation().z()-final.pose.linear().row(2).cwiseAbs().dot(final.half);
  if(std::abs(bottom-belt.pose.translation().z()-belt.half.z())>1e-9)throw std::runtime_error("NOMINAL_FIRST_TOUCH_HEIGHT");
  const auto local=belt.pose.inverse()*final.pose;
  for(int axis=0;axis<2;++axis)if(std::abs(local.translation()[axis])+local.linear().row(axis).cwiseAbs().dot(final.half)>belt.half[axis]-.01+1e-9)
    throw std::runtime_error("NOMINAL_FIRST_TOUCH_FOOTPRINT");
}
}
