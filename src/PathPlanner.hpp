#ifndef PATHPLANNER_HPP
#define PATHPLANNER_HPP

#include <iostream>
#include <vector>

#include "helpers.h"
#include "Vehicle.hpp"
#include "SFVehicleInfo.hpp"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;

class PathPlanner {
public:
    PathPlanner(vector<double> map_waypoints_x,
                vector<double> map_waypoints_y,
                vector<double> map_waypoints_s,
                vector<double> map_waypoints_dx,
                vector<double> map_waypoints_dy);
                
    std::vector<Eigen::VectorXd> computePath(Vehicle &vehicle, vector<SFVehicleInfo> sfInfo, vector<Eigen::VectorXd> previous_path, double end_path_s, double end_path_d);

private:
    double getLane(double d_value);
    double getRoundOffD(double d_value);
    bool getClosestVehicle(Vehicle vehicle, vector<SFVehicleInfo> sfInfo, SFVehicleInfo &sFVehicle, double lane, bool front);
    
    vector<double> map_waypoints_x_;
    vector<double> map_waypoints_y_;
    vector<double> map_waypoints_s_;
    vector<double> map_waypoints_dx_;
    vector<double> map_waypoints_dy_;
    
    std::vector<Eigen::VectorXd> previous_path_;
};

#endif // PATHPLANNER_HPP