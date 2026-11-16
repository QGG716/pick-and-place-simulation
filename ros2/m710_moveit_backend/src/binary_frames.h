#pragma once
// Binary worker frames, protocol version 2 (docs/native_boundary.md section 0;
// Python counterpart src/unloading_sim/workbench/binary_frames.py). Same
// canonical block rule on both sides: an array of >= 64 JSON floats (1-D), or
// an array of equal-length nonempty arrays of JSON floats with >= 64 values
// (2-D), becomes a float64 block {"$block": i}, numbered in sorted-key
// depth-first order. merge() restores identical float64 values.
#include <nlohmann/json.hpp>
#include <cstdint>
#include <cstring>
#include <istream>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace m710 {
struct FrameBlock {std::vector<uint32_t> shape;std::vector<double> data;};
struct Frame {uint16_t kind=0;uint64_t id=0;nlohmann::json value;size_t bytes=0;};
constexpr uint16_t kFrameRequest=1,kFrameReply=2,kFrameEvent=3,kFrameCancel=4;
constexpr size_t kMinBlockValues=64;

inline bool floatArray(const nlohmann::json& v) {
  if(!v.is_array()||v.empty())return false;
  for(const auto& x:v)if(!x.is_number_float())return false;
  return true;
}
inline void splitBlocks(nlohmann::json& v,std::vector<FrameBlock>& blocks) {
  if(v.is_object()) {
    if(v.contains("$block"))throw std::runtime_error("BINARY_FRAME_RESERVED_KEY");
    for(auto it=v.begin();it!=v.end();++it)splitBlocks(it.value(),blocks); // std::map: sorted keys
    return;
  }
  if(!v.is_array())return;
  if(floatArray(v)&&v.size()>=kMinBlockValues) {
    FrameBlock b;b.shape={uint32_t(v.size())};b.data.reserve(v.size());for(const auto& x:v)b.data.push_back(x.get<double>());
    blocks.push_back(std::move(b));v={{"$block",blocks.size()-1}};return;
  }
  bool matrix=!v.empty()&&v[0].is_array()&&!v[0].empty();
  const size_t width=matrix?v[0].size():0;
  if(matrix)for(const auto& r:v)if(!r.is_array()||r.size()!=width||!floatArray(r)) {matrix=false;break;}
  if(matrix&&v.size()*width>=kMinBlockValues) {
    FrameBlock b;b.shape={uint32_t(v.size()),uint32_t(width)};b.data.reserve(v.size()*width);
    for(const auto& r:v)for(const auto& x:r)b.data.push_back(x.get<double>());
    blocks.push_back(std::move(b));v={{"$block",blocks.size()-1}};return;
  }
  for(auto& x:v)splitBlocks(x,blocks);
}
inline void mergeBlocks(nlohmann::json& v,const std::vector<FrameBlock>& blocks) {
  if(v.is_object()) {
    if(v.size()==1&&v.contains("$block")) {
      const auto& b=blocks.at(v.at("$block").get<size_t>());
      if(b.shape.size()==1)v=b.data;
      else {nlohmann::json rows=nlohmann::json::array();
        for(size_t i=0;i<b.shape[0];++i)rows.push_back(std::vector<double>(b.data.begin()+i*b.shape[1],b.data.begin()+(i+1)*b.shape[1]));
        v=std::move(rows);}
      return;
    }
    for(auto it=v.begin();it!=v.end();++it)mergeBlocks(it.value(),blocks);
    return;
  }
  if(v.is_array())for(auto& x:v)mergeBlocks(x,blocks);
}
template<class T> inline void putRaw(std::string& out,T x) {out.append(reinterpret_cast<const char*>(&x),sizeof(T));}
inline std::string encodeFrame(uint16_t kind,uint64_t id,nlohmann::json value) {
  std::vector<FrameBlock> blocks;splitBlocks(value,blocks);
  const std::string header=value.dump();std::string out;out.reserve(64+header.size());
  out.append("E65B",4);putRaw<uint16_t>(out,2);putRaw<uint16_t>(out,kind);putRaw<uint64_t>(out,id);
  putRaw<uint32_t>(out,uint32_t(header.size()));putRaw<uint32_t>(out,uint32_t(blocks.size()));out+=header;
  for(const auto& b:blocks) {
    putRaw<uint8_t>(out,1);putRaw<uint8_t>(out,uint8_t(b.shape.size()));for(uint32_t s:b.shape)putRaw<uint32_t>(out,s);
    out.append(reinterpret_cast<const char*>(b.data.data()),b.data.size()*sizeof(double));
  }
  return out;
}
// Reads one frame whose 4-byte magic is still in the stream. Little-endian hosts only.
inline bool readFrame(std::istream& in,Frame& f) {
  auto exact=[&](void* p,size_t n) {in.read(static_cast<char*>(p),std::streamsize(n));if(size_t(in.gcount())!=n)throw std::runtime_error("BINARY_FRAME_TRUNCATED");};
  char magic[4];in.read(magic,4);if(in.gcount()==0)return false;
  if(in.gcount()!=4||std::memcmp(magic,"E65B",4)!=0)throw std::runtime_error("BINARY_FRAME_INVALID");
  uint16_t version,kind;uint64_t id;uint32_t header_length,count;
  exact(&version,2);exact(&kind,2);exact(&id,8);exact(&header_length,4);exact(&count,4);
  if(version!=2)throw std::runtime_error("BINARY_FRAME_VERSION");
  std::string header(header_length,'\0');exact(header.data(),header_length);
  std::vector<FrameBlock> blocks(count);size_t bytes=24+header_length;
  for(auto& b:blocks) {
    uint8_t dtype,ndim;exact(&dtype,1);exact(&ndim,1);
    if(dtype!=1||ndim<1||ndim>2)throw std::runtime_error("BINARY_FRAME_BLOCK_INVALID");
    b.shape.resize(ndim);size_t n=1;for(auto& s:b.shape) {exact(&s,4);n*=s;}
    b.data.resize(n);exact(b.data.data(),n*sizeof(double));bytes+=2+4*ndim+8*n;
  }
  f.kind=kind;f.id=id;f.value=nlohmann::json::parse(header);mergeBlocks(f.value,blocks);f.bytes=bytes;
  return true;
}
}
