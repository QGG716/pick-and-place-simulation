#pragma once
// Realtime step 7 (planning_module NATIVE): controlled/trajectory.py prepare()
// in the worker. The arithmetic follows scipy 1.14 / numpy 1.26 step by step
// (BPoly.from_derivatives -> PPoly.from_bernstein_basis -> _ppoly.evaluate,
// PPoly.derivative, numpy arange / unique), so every output agrees with the
// Python implementation to rounding; tools/check_native_prepare.py compares
// both on recorded native results (<= 1e-12). Error strings are identical.
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace m710 {
struct PrepareLimits {std::vector<double> lower,upper,velocity,acceleration;};
struct PrepareInput {std::vector<double> t;std::vector<double> q,v,a;}; // q,v,a: n x 6 row major
struct PreparedTrajectory {
  double period=0,time_scale=0,source_duration=0,scaled_duration=0,send_duration=0,terminal_hold=0,reach_bound=0;
  size_t subdivisions=0;
  std::vector<double> speed_bounds;
  std::vector<double> t,q,v,a;     // send grid: N, N x 6 (row major)
  std::vector<double> collision;   // M x 6
};

class PiecewisePolynomial {
 public:
  // c[(k*m+i)*J+j]: power-basis coefficient k (k=0 highest power) of interval i, joint j.
  int K=0;size_t m=0;int J=6;std::vector<double> x,c;
  double coef(int k,size_t i,int j) const {return c[(size_t(k)*m+i)*J+j];}
  double& coef(int k,size_t i,int j) {return c[(size_t(k)*m+i)*J+j];}
  // scipy.special.poch for a nonnegative integer m (cephes recurrence).
  static double poch(double a,double mm) {double r=1.;while(mm>=1.) {if(a+mm==1.)break;mm-=1.;r*=(a+mm);}return r;}
  // scipy.special.binom for small nonnegative integers (multiplication formula).
  static double comb(double n,double k) {
    double kx=std::floor(k);const double nx=std::floor(n);
    if(nx==n && kx>nx/2 && nx>0)kx=nx-kx;
    double num=1.,den=1.;for(int i=1;i<1+int(kx);++i) {num*=i+n-kx;den*=i;}
    return num/den;
  }
  static double sign(int p) {return (p%2)?-1.:1.;}
  // BPoly.from_derivatives(x, [[q_i, v_i, a_i], ...]) then PPoly.from_bernstein_basis.
  static PiecewisePolynomial quintic(const std::vector<double>& x,const std::vector<double>& q,
                                     const std::vector<double>& v,const std::vector<double>& a) {
    PiecewisePolynomial p;p.K=6;p.m=x.size()-1;p.x=x;const int J=p.J;const int n=6;
    std::vector<double> bern(size_t(n)*p.m*J);
    auto bc=[&](int k,size_t i,int j)->double& {return bern[(size_t(k)*p.m+i)*J+j];};
    for(size_t i=0;i<p.m;++i) {
      const double xa=x[i],xb=x[i+1];
      const double* ya[3]={&q[i*J],&v[i*J],&a[i*J]};const double* yb[3]={&q[(i+1)*J],&v[(i+1)*J],&a[(i+1)*J]};
      for(int j=0;j<J;++j) {
        double c[6];
        for(int qq=0;qq<3;++qq) {
          c[qq]=ya[qq][j]/poch(n-qq,qq)*std::pow(xb-xa,qq);
          for(int k=0;k<qq;++k)c[qq]-=(sign(k+qq)*comb(qq,k))*c[k];
        }
        for(int qq=0;qq<3;++qq) {
          c[n-qq-1]=yb[qq][j]/poch(n-qq,qq)*sign(qq)*std::pow(xb-xa,qq);
          for(int k=0;k<qq;++k)c[n-qq-1]-=(sign(k+1)*comb(qq,k+1))*c[n-qq+k];
        }
        for(int k=0;k<n;++k)bc(k,i,j)=c[k];
      }
    }
    p.c.assign(bern.size(),0.);const int k=n-1;
    for(int aa=0;aa<=k;++aa) {
      const double fs=sign(aa)*comb(k,aa);
      for(int s=aa;s<=k;++s) {
        const double val=comb(k-aa,s-aa)*sign(s);
        for(size_t i=0;i<p.m;++i) {
          const double dxs=std::pow(x[i+1]-x[i],s);
          for(int j=0;j<J;++j)p.coef(k-s,i,j)+=(fs*bc(aa,i,j))*val/dxs;
        }
      }
    }
    return p;
  }
  PiecewisePolynomial derivative(int nu) const {
    PiecewisePolynomial d;d.K=std::max(1,K-nu);d.m=m;d.J=J;d.x=x;d.c.assign(size_t(d.K)*m*J,0.);
    if(K-nu<=0)return d;
    for(int r=0;r<d.K;++r) {
      const double factor=poch(double(d.K-r),nu);
      for(size_t i=0;i<m;++i)for(int j=0;j<J;++j)d.coef(r,i,j)=coef(r,i,j)*factor;
    }
    return d;
  }
  // _ppoly.find_interval_ascending without extrapolation (-1 outside).
  long interval(double xv,long previous) const {
    const size_t nx=x.size();const double a=x[0],b=x[nx-1];
    long iv=previous;if(iv<0||size_t(iv)>=nx)iv=0;
    if(!(a<=xv&&xv<=b))return -1;
    if(xv==b)return long(nx-2);
    long low,high;
    if(xv>=x[iv]) {low=iv;high=long(nx-2);} else {low=0;high=iv;}
    if(xv<x[low+1])high=low;
    while(low<high) {
      const long mid=(high+low)/2;
      if(xv<x[mid])high=mid;else if(xv>=x[mid+1])low=mid+1;else {low=mid;break;}
    }
    return low;
  }
  // _ppoly.evaluate_poly1 (derivative order dx >= 0).
  double local(double s,size_t i,int j,int dx) const {
    double res=0.,z=1.;
    for(int kp=0;kp<K;++kp) {
      double prefactor=1.;
      if(dx>0) {if(kp<dx)continue;for(int k=kp;k>kp-dx;--k)prefactor*=k;}
      res=res+coef(K-kp-1,i,j)*z*prefactor;
      if(kp<K-1&&kp>=dx)z*=s;
    }
    return res;
  }
  void evaluate(const std::vector<double>& xs,int dx,std::vector<double>& out) const {
    out.assign(xs.size()*J,std::numeric_limits<double>::quiet_NaN());long iv=0;
    for(size_t p=0;p<xs.size();++p) {
      const long i=interval(xs[p],iv);if(i<0)continue;iv=i;
      for(int j=0;j<J;++j)out[p*J+j]=local(xs[p]-x[i],size_t(i),j,dx);
    }
  }
  double evaluateOne(double xv,int j) const {const long i=interval(xv,0);return i<0?std::numeric_limits<double>::quiet_NaN():local(xv-x[i],size_t(i),j,0);}
  // _ppoly.real_roots(y=0, discontinuity=True, extrapolate=False) of joint j:
  // per interval, exact real roots (quadratic formula, else companion matrix
  // eigenvalues) inside [x_i, x_{i+1}]; an identically zero interval adds x_i.
  std::vector<double> roots(int j) const {
    std::vector<double> out;double last=std::numeric_limits<double>::quiet_NaN();
    for(size_t i=0;i<m;++i) {
      int order=-1;for(int k=0;k<K;++k)if(coef(k,i,j)!=0.) {order=K-1-k;break;}
      std::vector<double> wr,wi;
      if(order<0) {out.push_back(x[i]);last=std::numeric_limits<double>::quiet_NaN();continue;}
      if(order==0)continue;
      if(order==1) {wr={-coef(K-1,i,j)/coef(K-2,i,j)};wi={0.};}
      else if(order==2) {
        const double a0=coef(K-3,i,j),a1=coef(K-2,i,j),a2=coef(K-1,i,j);double d=a1*a1-4*a0*a2;
        if(d<0) {d=std::sqrt(-d);wr={-a1/(2*a0),-a1/(2*a0)};wi={-d/(2*a0),d/(2*a0)};}
        else {
          d=std::sqrt(d);
          if(d==0) {wr={-a1/(2*a0),-a1/(2*a0)};wi={0.,0.};}
          else if(a1<0) {wr={(2*a2)/(-a1+d),(-a1+d)/(2*a0)};wi={0.,0.};}
          else {wr={(-a1-d)/(2*a0),(2*a2)/(-a1-d)};wi={0.,0.};}
        }
      } else {
        Eigen::MatrixXd companion=Eigen::MatrixXd::Zero(order,order);
        const double lead=coef(K-1-order,i,j);
        for(int k=0;k<order;++k) {
          companion(k,order-1)=-coef(K-1-k,i,j)/lead;
          if(k+1<order)companion(k+1,k)=1.;
        }
        Eigen::EigenSolver<Eigen::MatrixXd> solver(companion,false);
        if(solver.info()!=Eigen::Success)throw std::runtime_error("PREPARE_ROOT_FINDING_FAILED");
        std::vector<std::pair<double,double>> e;
        for(int k=0;k<order;++k)e.push_back({solver.eigenvalues()[k].real(),solver.eigenvalues()[k].imag()});
        std::sort(e.begin(),e.end(),[](const auto& p,const auto& q){return p.first<q.first;});
        for(const auto& r:e) {wr.push_back(r.first);wi.push_back(r.second);}
      }
      for(size_t k=0;k<wr.size();++k) {
        if(wi[k]!=0.)continue;
        const double r=wr[k]+x[i];
        if(!(x[i]<=r&&r<=x[i+1]))continue;
        if(r==last)continue;
        out.push_back(r);last=r;
      }
    }
    return out;
  }
};

// prepare(points, limits, time_scale, period, reach_bound_m); throws the Python error strings.
inline PreparedTrajectory nativePrepare(const PrepareInput& in,const PrepareLimits& limits,double time_scale,double period,double reach_bound) {
  const int J=6;
  if(!std::isfinite(time_scale)||!(1<=time_scale&&time_scale<=100)||!(.005<=period&&period<=.009))throw std::runtime_error("INVALID_TIMING");
  const size_t n=in.t.size();
  bool finite=true;
  for(double x:in.t)finite&=std::isfinite(x);for(double x:in.q)finite&=std::isfinite(x);
  for(double x:in.v)finite&=std::isfinite(x);for(double x:in.a)finite&=std::isfinite(x);
  if(n<2||in.q.size()!=n*J||in.v.size()!=n*J||in.a.size()!=n*J||!finite)throw std::runtime_error("INVALID_TIMED_TRAJECTORY");
  bool increasing=true;for(size_t i=1;i<n;++i)increasing&=!(in.t[i]-in.t[i-1]<=0);
  if(std::abs(in.t[0])>1e-9||!increasing||in.t[n-1]*time_scale>600)throw std::runtime_error("INVALID_TIME_AXIS");
  double vmax_end=0,amax_end=0;
  for(int j=0;j<J;++j) {vmax_end=std::max({vmax_end,std::abs(in.v[j]),std::abs(in.v[(n-1)*J+j])});amax_end=std::max({amax_end,std::abs(in.a[j]),std::abs(in.a[(n-1)*J+j])});}
  if(vmax_end>1e-6)throw std::runtime_error("NONZERO_ENDPOINT_VELOCITY");
  if(amax_end>1e-6)throw std::runtime_error("NONZERO_ENDPOINT_ACCELERATION");
  if(limits.lower.size()!=size_t(J)||limits.upper.size()!=size_t(J)||limits.velocity.size()!=size_t(J)||limits.acceleration.size()!=size_t(J))
    throw std::runtime_error("INVALID_LIMITS");
  for(int j=0;j<J;++j)
    if(!std::isfinite(limits.lower[j])||!std::isfinite(limits.upper[j])||
       !std::isfinite(limits.velocity[j])||!std::isfinite(limits.acceleration[j])||
       !(limits.lower[j]<limits.upper[j])||!(limits.velocity[j]>0)||!(limits.acceleration[j]>0))
      throw std::runtime_error("INVALID_LIMITS");
  std::vector<double> t(n),v(in.v),a(in.a);const double s2=std::pow(time_scale,2);
  for(size_t i=0;i<n;++i)t[i]=in.t[i]*time_scale;
  for(auto& x:v)x=x/time_scale;for(auto& x:a)x=x/s2;
  const auto poly=PiecewisePolynomial::quintic(t,in.q,v,a);
  PreparedTrajectory out;out.period=period;out.time_scale=time_scale;out.reach_bound=reach_bound;
  for(int j=0;j<J;++j) {
    const double bounds[3][2]={{limits.lower[j],limits.upper[j]},{-limits.velocity[j],limits.velocity[j]},{-limits.acceleration[j],limits.acceleration[j]}};
    for(int order=0;order<3;++order) {
      const auto f=poly.derivative(order);const auto g=f.derivative(1);
      std::vector<double> xs=t;for(double r:g.roots(j))if(std::isfinite(r))xs.push_back(r);
      double peak=0.;bool bad=false;
      for(double xv:xs) {
        const double y=f.evaluateOne(xv,j);
        if(y<bounds[order][0]-1e-8||y>bounds[order][1]+1e-8)bad=true;
        peak=std::max(peak,std::abs(y));
      }
      if(bad)throw std::runtime_error("INTERPOLATION_LIMIT:"+std::to_string(j+1)+":"+std::to_string(order));
      if(order==1)out.speed_bounds.push_back(peak);
    }
  }
  const double tend=t[n-1];
  const size_t count=size_t(std::ceil(tend/period))+1;
  if(count>120001)throw std::runtime_error("TRAJECTORY_TOO_LARGE");
  std::vector<double> ts(count),clipped(count);
  for(size_t i=0;i<count;++i) {ts[i]=double(i)*period;clipped[i]=std::min(ts[i],tend);}
  std::vector<double> qs,vs,acc;poly.evaluate(clipped,0,qs);poly.evaluate(clipped,1,vs);poly.evaluate(clipped,2,acc);
  bool discrete=false,jump=false;
  std::vector<double> dv((count-1)*J);
  for(size_t i=0;i+1<count;++i)for(int j=0;j<J;++j) {
    dv[i*J+j]=(qs[(i+1)*J+j]-qs[i*J+j])/period;
    if(std::abs(dv[i*J+j])>limits.velocity[j]+1e-8)discrete=true;
    if(std::abs(qs[(i+1)*J+j]-qs[i*J+j])>.002)jump=true;
  }
  if(count-1>1)for(size_t i=0;i+2<count;++i)for(int j=0;j<J;++j)
    if(std::abs((dv[(i+1)*J+j]-dv[i*J+j])/period)>limits.acceleration[j]+1e-8)discrete=true;
  if(discrete)throw std::runtime_error("DISCRETE_COMMAND_LIMIT");
  if(jump)throw std::runtime_error("COMMAND_DISCONTINUITY");
  if(!std::isfinite(reach_bound)||reach_bound<=0)throw std::runtime_error("INVALID_REACH_BOUND");
  double speed_sum=0.;for(double x:out.speed_bounds)speed_sum+=x;
  const size_t subdivisions=std::max<size_t>(1,size_t(std::ceil(4*reach_bound*speed_sum*period/.001)));
  if(subdivisions*count>500000)throw std::runtime_error("COLLISION_PATH_RESOURCE_LIMIT");
  const double step=period/double(subdivisions);
  const size_t dense_count=size_t(std::ceil((tend-0.)/step));
  std::vector<double> dense;dense.reserve(dense_count+n+count);
  for(size_t i=0;i<dense_count;++i)dense.push_back(double(i)*step);
  dense.insert(dense.end(),t.begin(),t.end());dense.insert(dense.end(),clipped.begin(),clipped.end());
  std::sort(dense.begin(),dense.end());dense.erase(std::unique(dense.begin(),dense.end()),dense.end());
  poly.evaluate(dense,0,out.collision);
  out.t=ts;out.q=qs;out.v=vs;out.a=acc;out.subdivisions=subdivisions;
  out.scaled_duration=tend;out.source_duration=tend/time_scale;out.send_duration=ts[count-1];out.terminal_hold=ts[count-1]-tend;
  return out;
}
}
