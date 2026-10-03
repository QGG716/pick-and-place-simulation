#include "native_zero_motion.h"
#include <limits>
#include <iostream>
#include <stdexcept>
void require(bool value) {if(!value) throw std::runtime_error("stationary event regression");}
int main() {
  std::vector<double> start{.1,-.2,.3,-.4,.5,-.6};
  trajectory_msgs::msg::JointTrajectory original;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions=start;point.velocities=point.accelerations=std::vector<double>(6,0.);
  original.points.push_back(point);
  require(m710::stationaryPointFailure(original,start,true).empty());
  require(!m710::stationaryPointFailure(original,start,false).empty());
  auto x=original;x.points[0].positions[0]+=1e-12;
  require(!m710::stationaryPointFailure(x,start,true).empty());
  x=original;x.points[0].time_from_start.nanosec=1;
  require(!m710::stationaryPointFailure(x,start,true).empty());
  x=original;x.points[0].velocities[1]=1e-12;
  require(!m710::stationaryPointFailure(x,start,true).empty());
  x=original;x.points[0].accelerations[2]=1e-12;
  require(!m710::stationaryPointFailure(x,start,true).empty());
  x=original;x.points[0].positions[3]=std::numeric_limits<double>::quiet_NaN();
  require(!m710::stationaryPointFailure(x,start,true).empty());
  x=original;x.points[0].accelerations.clear();
  require(!m710::stationaryPointFailure(x,start,true).empty());
  x=original;x.points.push_back(point);
  require(!m710::stationaryPointFailure(x,start,true).empty());
  std::cout << "PASS: 9 stationary native point cases; no planning evidence\n";
}
