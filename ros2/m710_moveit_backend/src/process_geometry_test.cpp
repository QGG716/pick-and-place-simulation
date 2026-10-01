// Small literal counterexamples for new native geometry; never execution evidence.
#include "process_geometry.h"
#include <iostream>
using J=nlohmann::json;
void require(bool value,const char* reason) {if(!value) throw std::runtime_error(reason);}
J identity() {return {{1.,0.,0.,0.},{0.,1.,0.,0.},{0.,0.,1.,0.},{0.,0.,0.,1.}};}
J sealPolicy() {
  J cups=J::array(),active=J::array(),eligible=J::array();
  for(size_t i=0;i<72;++i) {cups.push_back({{"index",i},{"cup_id","cup_"+std::to_string(i)},
    {"center_contact_frame_m",{0.,0.,0.}},{"seal_radius_m",.0215}});active.push_back(i==0);eligible.push_back(true);}
  return {{"cups",cups},{"commanded_active_mask",active},{"geometrically_eligible_mask",eligible},{"actual_contact_mask",active},
    {"target_face","top"},{"flange_from_physical_contact",identity()},{"max_attachment_gap_m",.002},
    {"maximum_penetration_m",.0002},{"max_normal_misalignment_rad",.08726646259971647},{"suction_edge_margin_m",0.}};
}
int main() {
  m710::ProcessBox a,b;a.id="target";b.id="neighbor";a.half=b.half=Eigen::Vector3d(.1,.2,.3);
  b.pose.translation().x()=.21;require(std::abs(m710::processSignedGap(a,b)-.01)<1e-12,"SAT_POSITIVE_GAP");
  b.pose.translation().x()=.195;require(std::abs(m710::processSignedGap(a,b)+.005)<1e-12,"SAT_PENETRATION");
  b.pose.translation()=Eigen::Vector3d(0,0,-.6);require(m710::processSupportSeparated(a,b,.0002),"LEGAL_TOP_SUPPORT");
  b.pose.translation().z()=-.599;require(!m710::processSupportSeparated(a,b,.0002),"DEEP_SUPPORT_REJECTED");
  b.pose.translation()=Eigen::Vector3d(.2,0,0);require(!m710::processSupportSeparated(a,b,.0002),"SIDE_CONTACT_NOT_SUPPORT");
  auto p=sealPolicy();auto flange=Eigen::Isometry3d::Identity();flange.translation().z()=.3;
  flange.linear()=Eigen::AngleAxisd(std::acos(-1.),Eigen::Vector3d::UnitX()).toRotationMatrix();
  auto ok=m710::processSealRings(p,a,flange);require(ok.value("status",std::string())=="PASS","ONE_VALID_CUP_SUFFICIENT");
  require(ok.at("computed_actual_contact_mask").size()==72 && ok.at("commanded_count")==1,"INDEPENDENT_72_MASK");
  p["cups"][0]["center_contact_frame_m"]={.09,0.,0.};
  require(m710::processSealRings(p,a,flange).at("reason")=="COMMANDED_CUP_SEAL_INVALID","FULL_RING_OVER_EDGE_REJECTED");
  p=sealPolicy();flange.translation().z()=.303;
  require(m710::processSealRings(p,a,flange).at("reason")=="COMMANDED_CUP_SEAL_INVALID","REMOTE_ATTACHMENT_REJECTED");
  p["commanded_active_mask"][0]=false;
  require(m710::processSealRings(p,a,flange).at("reason")=="NO_COMMANDED_CUP","ZERO_CUPS_REJECTED");
  bool invalid=false;try {auto t=identity();t[3][0]=1.;m710::processTransform(t);} catch(const std::exception&) {invalid=true;}
  require(invalid,"NON_SE3_REJECTED");
  J motion={{"stage","extraction"},{"pipeline_id","pilz_industrial_motion_planner"},{"planner_id","PTP"},{"q_goal",{0.,0.,0.,0.,0.,0.}}};
  invalid=false;try {m710::validateNativeMotionScope(motion);} catch(const std::exception&) {invalid=true;}
  require(invalid,"PROCESS_PTP_SHORTCUT_REJECTED");
  motion["planner_id"]="LIN";invalid=false;try {m710::validateNativeMotionScope(motion);} catch(const std::exception&) {invalid=true;}
  require(invalid,"PROCESS_JOINT_GOAL_WITHOUT_TCP_REJECTED");
  motion["goal_pose"]=identity();motion.erase("q_goal");m710::validateNativeMotionScope(motion);
  // Source bellows extend 10 mm beyond the nominally compressed contact
  // plane. Production URDF import must use the core's compressed boxes;
  // the additional-compression threshold remains exactly 5 mm.
  m710::ProcessBox raw_cup,nominal_cup;raw_cup.half=Eigen::Vector3d(.02,.02,.015);raw_cup.pose.translation().z()=.305;
  nominal_cup.half=Eigen::Vector3d(.02,.02,.01);nominal_cup.pose.translation().z()=.31;
  require(std::abs(m710::processSignedGap(raw_cup,a)+.01)<1e-12,"SOURCE_BELLOWS_NOMINAL_COMPRESSION");
  require(std::abs(m710::processSignedGap(nominal_cup,a))<1e-12,"COMPRESSED_BELLOWS_ZERO_ADDITIONAL_COMPRESSION");
  nominal_cup.pose.translation().z()=.298;
  require(std::abs(m710::processSignedGap(nominal_cup,a)+.012)<1e-12 &&
    m710::processSignedGap(nominal_cup,a)<-.005,"ADDITIONAL_12MM_COMPRESSION_REJECTED");
  std::cout<<J({{"status","PASS"},{"scope","synthetic_process_geometry_only"},{"cases",15}}).dump()<<std::endl;
}
