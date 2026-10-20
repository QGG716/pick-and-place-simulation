#pragma once
#include "process_geometry.h"
#include <fcl/fcl.h>
namespace m710 {
inline std::vector<Eigen::Vector3d> solidVertices(const ProcessJson& box) {
  const auto b=processBox(box);std::vector<Eigen::Vector3d> v;
  if(box.contains("solid")) {
    const auto& s=box.at("solid");
    if(s.at("kind")!="convex_concept_solid_v1" || s.at("vertices_m").size()!=8)
      throw std::runtime_error("INVALID_CONCEPT_SOLID");
    for(const auto& p:s.at("vertices_m")) {
      if(p.size()!=3)throw std::runtime_error("INVALID_CONCEPT_SOLID");
      Eigen::Vector3d x(p[0],p[1],p[2]);
      if(!x.allFinite() || (x.cwiseAbs()-b.half).maxCoeff()>1e-12)
        throw std::runtime_error("INVALID_CONCEPT_SOLID");
      v.push_back(x);
    }
  } else for(int x:{-1,1})for(int y:{-1,1})for(int z:{-1,1})
    v.push_back(Eigen::Vector3d(x*b.half.x(),y*b.half.y(),z*b.half.z()));
  return v;
}
inline const std::vector<std::array<int,4>>& solidFaces() {
  static const std::vector<std::array<int,4>> f={{0,1,3,2},{4,6,7,5},{0,4,5,1},{2,3,7,6},{0,2,6,4},{1,5,7,3}};
  return f;
}
inline void validateSolid(const ProcessJson& box) {
  if(!box.contains("solid"))return;
  const auto v=solidVertices(box);
  if(box.at("solid").at("faces")!=ProcessJson(solidFaces()))throw std::runtime_error("INVALID_CONCEPT_FACES");
  for(const auto& f:solidFaces()) {
    Eigen::Vector3d n=(v[f[1]]-v[f[0]]).cross(v[f[2]]-v[f[1]]);
    if(n.norm()<1e-12)throw std::runtime_error("DEGENERATE_CONCEPT_FACE");n.normalize();
    if(std::abs(n.dot(v[f[3]]-v[f[0]]))>1e-9)throw std::runtime_error("NONPLANAR_CONCEPT_FACE");
    for(const auto& p:v)if(n.dot(p-v[f[0]])>1e-9)throw std::runtime_error("NONCONVEX_CONCEPT_SOLID");
  }
}
inline double solidSignedGap(const ProcessJson& first,const ProcessJson& second) {
  auto a=solidVertices(first),b=solidVertices(second);
  const auto pa=processBox(first).pose,pb=processBox(second).pose;
  for(auto& v:a)v=pa*v;for(auto& v:b)v=pb*v;
  std::vector<Eigen::Vector3d> axes,ea,eb;
  for(auto pair:{std::make_pair(&a,&ea),std::make_pair(&b,&eb)})for(const auto& f:solidFaces()) {
    const auto& v=*pair.first;
    axes.push_back((v[f[1]]-v[f[0]]).cross(v[f[2]]-v[f[1]]));
    for(int i=0;i<4;++i)pair.second->push_back(v[f[(i+1)%4]]-v[f[i]]);
  }
  for(const auto& x:ea)for(const auto& y:eb)axes.push_back(x.cross(y));
  double result=-std::numeric_limits<double>::infinity();
  for(auto axis:axes) {if(axis.norm()<1e-12)continue;axis.normalize();
    double amin=1e100,amax=-1e100,bmin=1e100,bmax=-1e100;
    for(const auto& p:a){amin=std::min(amin,axis.dot(p));amax=std::max(amax,axis.dot(p));}
    for(const auto& p:b){bmin=std::min(bmin,axis.dot(p));bmax=std::max(bmax,axis.dot(p));}
    result=std::max(result,std::max(bmin-amax,amin-bmax));
  }
  if(!std::isfinite(result))throw std::runtime_error("UNKNOWN_CONCEPT_GAP");
  return result;
}
inline std::shared_ptr<fcl::CollisionGeometryd> solidFcl(const ProcessJson& b) {
  const auto parsed=processBox(b);
  if(!b.contains("solid"))return std::make_shared<fcl::Boxd>(2*parsed.half.x(),2*parsed.half.y(),2*parsed.half.z());
  validateSolid(b);
  auto vertices=std::make_shared<std::vector<Eigen::Vector3d>>(solidVertices(b));
  auto faces=std::make_shared<std::vector<int>>();
  for(const auto& f:solidFaces()){faces->push_back(4);faces->insert(faces->end(),f.begin(),f.end());}
  return std::make_shared<fcl::Convexd>(vertices,6,faces,true);
}
}
