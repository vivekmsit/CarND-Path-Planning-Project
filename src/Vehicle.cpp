#include "Vehicle.hpp"
#include <math.h>

Vehicle::Vehicle() {
}

Vehicle::Vehicle(double x, double y, double s, double d, double yaw, double speed): 
                      x_(x), y_(y), s_(s), d_(d), yaw_(yaw), speed_(speed) {
  vx_ = speed_*cos(yaw_);
  vy_ = speed_*sin(yaw_);
  speed_ = speed/2.237; // change from miles per hour to meters per second
}

void Vehicle::print() {
 std::cout<<"Vehicle: x=" << x_ << ", y="<<y_ << ", s="<<s_<<", d="<<d_<<", yaw="<<yaw_<<", speed="<<speed_<<std::endl;
}