#include "PathPlanner.hpp"
#include <iostream>

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

std::vector<Point> PathPlanner::computePath(Vehicle &vehicle, vector<SFVehicleInfo> sfInfo, vector<Eigen::VectorXd> previous_path) {
  std::vector<Point> path;
  return path;
}

/*std::vector<Point> PathPlanner::computePath(Vehicle &vehicle, vector<SFVehicleInfo> sfInfo, vector<Eigen::VectorXd> previous_path) {
  std::vector<Point> path;
  double pos_x;
  double pos_y;
  double angle;
  int path_size = previous_path.size();
  for (auto &obj: previous_path) {
   path.push_back(Point(obj[0], obj[1]));
  }
  if (path_size == 0) {
     pos_x = vehicle.x_;
     pos_y = vehicle.y_;
     angle = deg2rad(vehicle.yaw_);
  } else {
     pos_x = previous_path[path_size-1][0];
     pos_y = previous_path[path_size-1][1];
     double pos_x2 = previous_path[path_size-2][0];
     double pos_y2 = previous_path[path_size-2][1];
     angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
  }
  double dist_inc = 0.5;
  for (int i = 0; i < 50-path_size; ++i) {
    Point p;
    p.x = pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100));
    p.y = pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100));
    pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
    pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
    path.push_back(p);
  }
  return path;
}*/


/*std::vector<Point> PathPlanner::computePath(Vehicle &vehicle, vector<SFVehicleInfo> sfInfo, vector<Eigen::VectorXd> previous_path) {
  std::vector<Point> path;
  double dist_inc = 0.5;
  for (int i =0; i<50; i++) {
    double next_s = vehicle.s_ + (i+1)*dist_inc;
    double next_d = 6;
    vector<double> XY = getXY(next_s, next_d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
    path.push_back(Point(XY[0], XY[1]));
  }
  return path;
}*/
