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
                
    std::vector<Eigen::VectorXd> computePath(Vehicle vehicle, vector<SFVehicleInfo> sfInfo, vector<Eigen::VectorXd> previous_path, double end_path_s, double end_path_d);

private:
    //bool laneChangeRequired();
    //bool getLaneChangeInfo(SFVehicleInfo &sfObj, double &targetLane, bool &vehicleToFollow);
    double getLaneChangePath(const SFVehicleInfo &sfObj, const double &targetLane, const bool &vehicleToFollow);
    
    Vehicle vehicle_;
    vector<SFVehicleInfo> sensorFusionList_;
    vector<Eigen::VectorXd> previousPath_;
    double end_path_s_;
    double end_path_d_;
    
    // Fixed way points
    vector<double> map_waypoints_x_;
    vector<double> map_waypoints_y_;
    vector<double> map_waypoints_s_;
    vector<double> map_waypoints_dx_;
    vector<double> map_waypoints_dy_;
};

#endif // PATHPLANNER_HPP