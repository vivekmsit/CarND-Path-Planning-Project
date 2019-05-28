#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <vector>

using namespace std;

class Trajectory {
public:
  Trajectory();
  virtual ~Trajectory();
  
  void add(double x, double y,
           double s, double s_dot, double s_dot_dot, double s_dot_dot_dot,
           double d, double d_dot, double d_dot_dot, double d_dot_dot_dot,
           double yaw);
  void removeFirstNPoints(int n);
  void keepFirstNPoints(int n);
  int size();

  vector<double> xs_;
  vector<double> ys_;
  vector<double> ss_;
  vector<double> s_vels_;
  vector<double> s_accs_;
  vector<double> s_jks_;
  vector<double> ds_;
  vector<double> d_vels_;
  vector<double> d_accs_;
  vector<double> d_jks_;
  vector<double> yaws_;
};

#endif // TRAJECTORY_HPP