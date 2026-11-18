// Diagnostic only: inspect the actual native interpolation coefficients.
#include "native_prepare.h"
#include <nlohmann/json.hpp>
#include <iostream>
using J=nlohmann::json;
int main() {
  std::string line;std::getline(std::cin,line);const auto r=J::parse(line);
  const double s=r.at("time_scale");std::vector<double> t,q,v,a;
  for(const auto& p:r.at("points")) {
    t.push_back(p.at("t").get<double>()*s);
    for(int j=0;j<6;++j) {
      q.push_back(p.at("q")[j]);v.push_back(p.at("v")[j].get<double>()/s);
      a.push_back(p.at("a")[j].get<double>()/std::pow(s,2));
    }
  }
  const auto poly=m710::PiecewisePolynomial::quintic(t,q,v,a);
  std::cout<<J{{"c",poly.c},{"x",poly.x}}.dump()<<std::endl;
}
