#include "Trajectory.h"

Trajectory::Trajectory() {
}

Trajectory::~Trajectory() {
}
  
void Trajectory::add(double x, double y,
           double s, double s_dot, double s_dot_dot, double s_dot_dot_dot,
           double d, double d_dot, double d_dot_dot, double d_dot_dot_dot,
           double yaw) {
  xs_.push_back(x);
  ys_.push_back(y);
  ss_.push_back(s);
  s_vels_.push_back(s_dot);
  s_accs_.push_back(s_dot_dot);
  s_jks_.push_back(s_dot_dot_dot);

  ds_.push_back(d);
  d_vels_.push_back(d_dot);
  d_accs_.push_back(d_dot_dot);
  d_jks_.push_back(d_dot_dot_dot);
  
  yaws_.push_back(yaw);
}

void Trajectory::removeFirstNPoints(int n) {
  xs_.erase(xs_.begin(), xs_.begin() + n);
  ys_.erase(ys_.begin(), ys_.begin() + n);
  ss_.erase(ss_.begin(), ss_.begin() + n);
  s_vels_.erase(s_vels_.begin(), s_vels_.begin() + n);
  s_accs_.erase(s_accs_.begin(), s_accs_.begin() + n);
  s_jks_.erase(s_jks_.begin(), s_jks_.begin() + n);
  ds_.erase(ds_.begin(), ds_.begin() + n);
  d_vels_.erase(d_vels_.begin(), d_vels_.begin() + n);
  d_accs_.erase(d_accs_.begin(), d_accs_.begin() + n);
  d_jks_.erase(d_jks_.begin(), d_jks_.begin() + n);
  yaws_.erase(yaws_.begin(), yaws_.begin() + n);  
}

void Trajectory::keepFirstNPoints(int n) {
  xs_.erase(xs_.begin() + n, xs_.end());
  ys_.erase(ys_.begin() + n, ys_.end());
  ss_.erase(ss_.begin() + n, ss_.end());
  s_vels_.erase(s_vels_.begin() + n, s_vels_.end());
  s_accs_.erase(s_accs_.begin() + n, s_accs_.end());
  s_jks_.erase(s_jks_.begin() + n, s_jks_.end());
  ds_.erase(ds_.begin() + n, ds_.end());
  d_vels_.erase(d_vels_.begin() + n, d_vels_.end());
  d_accs_.erase(d_accs_.begin() + n, d_accs_.end());
  d_jks_.erase(d_jks_.begin() + n, d_jks_.end());
  yaws_.erase(yaws_.begin() + n, yaws_.end()); 
}

int Trajectory::size() {
  return xs_.size(); 
}