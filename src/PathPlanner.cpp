#include "PathPlanner.hpp"
#include "helpers.h"

PathPlanner::PathPlanner(vector<double> map_waypoints_x,
                        vector<double> map_waypoints_y,
                        vector<double> map_waypoints_s,
                        vector<double> map_waypoints_dx,
                        vector<double> map_waypoints_dy): map_waypoints_x_(map_waypoints_x),
                                                          map_waypoints_y_(map_waypoints_y),
                                                          map_waypoints_s_(map_waypoints_s),
                                                          map_waypoints_dx_(map_waypoints_dx),
                                                          map_waypoints_dy_(map_waypoints_dy) {
}

std::vector<std::vector<double>> PathPlanner::computePath(Vehicle &vehicle, vector<SFVehicleInfo> sfInfo, vector<Eigen::VectorXd> previous_path) {
  std::vector<std::vector<double>> path;
  double dist_inc = 0.5;
  for (int i =0; i<50; i++) {
    double next_s = vehicle.s_ + (i+1)*dist_inc;
    double next_d = 6;
    vector<double> XY = getXY(next_s, next_d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
    path[i][0] = XY[0];
    path[i][1] = XY[1];
  }
  return path;
}