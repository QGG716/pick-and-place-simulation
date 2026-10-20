// Exact-volume containment regression for the new same-source end relief.
#define main original_clearance_main
#include "clearance_test.cpp"
#undef main
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);auto s=scene(.02);
  J vertices=J::array();
  for(double x:{-.5,.5})for(double y:{-.5,.5})for(double z:{-.5,.5})
    vertices.push_back({x<0&&z<0?-.2:x,y,z});
  J spec={{"id","conveyor_transverse"},{"size",{1.,1.,1.}},
    {"pose",{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}},
    {"solid",{{"kind","convex_concept_solid_v1"},{"vertices_m",vertices},{"faces",m710::solidFaces()}}}};
  m710::validateSolid(spec);
  moveit_msgs::msg::CollisionObject obj;obj.id="conveyor_transverse";obj.header.frame_id="world";obj.operation=obj.ADD;
  shape_msgs::msg::Mesh mesh;
  for(const auto& v:m710::solidVertices(spec)){geometry_msgs::msg::Point p;p.x=v.x();p.y=v.y();p.z=v.z();mesh.vertices.push_back(p);}
  for(const auto& f:m710::solidFaces())for(int i:{1,2}){shape_msgs::msg::MeshTriangle t;t.vertex_indices={
    static_cast<uint32_t>(f[0]),static_cast<uint32_t>(f[i]),static_cast<uint32_t>(f[i+1])};mesh.triangles.push_back(t);}
  geometry_msgs::msg::Pose pose;pose.orientation.w=1;obj.meshes.push_back(mesh);obj.mesh_poses.push_back(pose);
  require(s->processCollisionObjectMsg(obj),"MESH_IMPORT");
  collision_detection::CollisionRequest cr;collision_detection::CollisionResult raw;
  s->getCollisionEnvUnpadded()->checkRobotCollision(cr,raw,s->getCurrentState(),s->getAllowedCollisionMatrix());
  auto p=policy();p["concept_solids"]={spec};
  m710::Clearance gate(s,p,"optimized");
  require(!gate.check(s->getCurrentState()),"FULLY_ENCLOSED_SMALL_BODY_MUST_REJECT");
  auto outside=s->getCurrentState();outside.setVariablePosition("J1",-.8);outside.update();
  require(gate.check(outside),"OUTSIDE_SOLID_CLEARANCE");
  bool rejected=false;try{m710::Clearance missing(s,policy(),"optimized");}catch(const std::exception&){rejected=true;}
  require(rejected,"MISSING_CONVEX_BINDING_MUST_FAIL");
  p["concept_solids"][0]["solid"]["vertices_m"][0][0]=-.19;
  rejected=false;try{m710::Clearance corrupt(s,p,"optimized");}catch(const std::exception&){rejected=true;}
  require(rejected,"CHANGED_SOLID_OR_NONPLANAR_FACE_MUST_FAIL");
  std::cout<<J({{"status","PASS"},{"surface_mesh_enclosure_collision",raw.collision},
    {"exact_convex_enclosure_rejected",true},{"scope","SYNTHETIC_CONTAINMENT_AND_IDENTITY"}}).dump()<<std::endl;
  rclcpp::shutdown();
}
