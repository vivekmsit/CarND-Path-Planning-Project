#ifndef PATHPLANNER_HPP
#define PATHPLANNER_HPP

#include <iostream>
#include <vector>

#include "helpers.h"
#include "Vehicle.hpp"
#include "SFVehicleInfo.hpp"
#include "StateMachine.hpp"
#include "Trajectory.h"
#include "spline.h"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"

using namespace std;
using namespace tk;

class PathPlanner {
public:
    PathPlanner(vector<double> map_waypoints_x,
                vector<double> map_waypoints_y,
                vector<double> map_waypoints_s,
                vector<double> map_waypoints_dx,
                vector<double> map_waypoints_dy);
                
    Trajectory computePath(Vehicle vehicle, vector<SFVehicleInfo> sfInfo, vector<Eigen::VectorXd> previous_path, double end_path_s, double end_path_d);

private:
    // Private Member Functions
    double getStateCost(State state, StateInfo &stInfo, bool &stInfoAvailable);
    double getNormalFutureSpeed();
    double getCurrentLaneStateCost(StateInfo &stInfo, bool &stInfoAvailable);
    double getLaneChangeCost(StateInfo &stInfo, bool &stInfoAvailable, const State state);
    bool getClosestVehicle(SFVehicleInfo &sFVehicle, double lane, bool front);
    Eigen::VectorXd polyfit(vector<double> xvals, vector<double> yvals,	int order);
    vector<double> JMT(const vector<double> &start, const vector<double> &end, const double T);
    void smoothPolynomial(vector<double> inXValues, vector<double> inYValues, vector<double> &xValues, vector<double> &yValues);
    Trajectory getTrajectory(const StateInfo &nextStateInfo);
    Trajectory getJMTTrajectory(const StateInfo &nextStateInfo);
    bool getSplineTrajectory(const StateInfo &nextStateInfo, vector<double> &xValues, vector<double> &yValues);
    void buildSplines();
    vector<double> toRealWorldXY(double s, double d);
    
    // Private Member Variables
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
    StateMachine stateMachine_;
    bool laneChangeInProgress_;

    spline sp_x_s_;
    spline sp_y_s_;
    spline sp_dx_s_;
    spline sp_dy_s_;
    
    Trajectory currentTrajectory_;
    int laneChangePoints_;
};

#endif // PATHPLANNER_HPP