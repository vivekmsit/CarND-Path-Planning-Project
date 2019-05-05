#include "PathPlanner.hpp"
#include <iostream>
#include <cstdlib>
#include "helpers.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define DISTANCE_THRESHOLD 10
#define PATH_DURATION 2

#define CURRENT_NEXT_DIST_THRESHOLD 5
#define FUTURE_NEXT_DIST_THRESHOLD 5
#define CURRENT_PREV_DIST_THRESHOLD 5
#define FUTURE_PREV_DIST_THRESHOLD 5

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

std::vector<Eigen::VectorXd> PathPlanner::computePath(Vehicle vehicle, vector<SFVehicleInfo> sfInfo, vector<Eigen::VectorXd> previous_path, double end_path_s, double end_path_d) {
  
  // store data into private member variables
  vehicle_ = vehicle;
  sensorFusionList_ = sfInfo;
  previousPath_ = previous_path;
  end_path_s_ = end_path_s;
  end_path_d_ = end_path_d;
  
  std::vector<Eigen::VectorXd> path;
  double dist_inc = 0.4;
  vehicle_.print();
  int path_size = previousPath_.size();
  std::cout<<"size of previous path is: " << path_size << std::endl;
  std::cout<<"end_path_s is: " << end_path_s_ << ", end_path_d: " << end_path_d_ << std::endl;
  
  double first_x;
  double first_y;
  double last_x;
  double last_y;
  
  if (path_size != 0) {
    first_x = previousPath_[0][0];
    first_y = previousPath_[0][1];
    last_x = previousPath_[path_size-1][0];
    last_y = previousPath_[path_size-1][1];
    std::cout<<"first x position is: " << first_x << " and first y position is: " << first_y << std::endl;
    std::cout<<"last x position is: " << last_x << " and last y position is: " << last_y << std::endl;
  } else {
    last_x = vehicle.x_;
    last_y = vehicle.y_;
    end_path_s = vehicle.s_;
    end_path_d = vehicle.d_;
  }
  
  for (auto &obj: previousPath_) {
    Eigen::VectorXd point(2);
    point[0] = obj[0];
    point[1] = obj[1];
    path.push_back(point);
  }
  
  State nextState;
  StateInfo nextStateInfo;
  bool nextStateAvailable = false;
  double minCost = std::numeric_limits<double>::max();
  vector<State> nextStates = stateMachine_.getNextStates(getLane(vehicle_.d_));
  for (auto state: nextStates) {
    StateInfo stInfo;
    bool stInfoAvailable;
    double stateCost = getStateCost(state, stInfo, stInfoAvailable);
    if (stInfoAvailable && stateCost <= 0.5) {
      if (stateCost < minCost) {
       minCost = stateCost;
       nextStateInfo = stInfo;
       nextStateAvailable = true;
      }
    }
    std::cout<<"state cost is: " << stateCost << std::endl;
  }
  
  if (nextStateAvailable) {
    vector<double> xValues;
    vector<double> yValues;
    bool status = getPathCoordinates(nextStateInfo, xValues, yValues);
  } else {
    // TODO
  }
  
  double next_d = getRoundOffD(end_path_d);
  for (int i =0; i<50 - path_size; i++) {
    double next_s = end_path_s + (i+1)*dist_inc;
    vector<double> XY = getXY(next_s, next_d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
    Eigen::VectorXd point(2);
    point[0] = XY[0];
    point[1] = XY[1];
    path.push_back(point);
    std::cout<<"new point x value is: " << point[0] << " and y value is: " << point[1] << std::endl;
  }
  
  int final_size = path.size();
  auto obj = path.back();
  std::cout<<"final path size is: " << final_size << std::endl;
  std::cout<<"last co-ordinates of new path are: x: " << obj[0];
  std::cout<<" and y: " << obj[1] << std::endl;
  std::cout<<std::endl;
  return path;
}

double PathPlanner::getLaneChangePath(const SFVehicleInfo &sfObj, const double &targetLane, const bool &vehicleToFollow) {
  return 0;
}

double PathPlanner::getStateCost(State state, StateInfo &stInfo, bool &stInfoAvailable) {
  double cost = std::numeric_limits<double>::max();;
  switch(state) {
    case State::CURRENT:
      cost = getCurrentLaneStateCost(stInfo, stInfoAvailable);
      break;
    case State::LEFT_CHANGE:
      cost = getLaneChangeCost(stInfo, stInfoAvailable, state);
      break;
    case State::RIGHT_CHANGE:
      cost = getLaneChangeCost(stInfo, stInfoAvailable, state);
      break;
    default:
      break;
  }
  return cost;
}

double PathPlanner::getCurrentLaneStateCost(StateInfo &stInfo, bool &stInfoAvailable) {
  double cost = 0;
  SFVehicleInfo nextSF;
  double currentLane = getLane(vehicle_.d_);
  double nextFound = getClosestVehicle(nextSF, currentLane, true);
  if (nextFound) {
    // check if we can get too close to the car in the lane
    double vehicleFuturePosX = vehicle_.vx_*PATH_DURATION;
    double vehicleFuturePosY = vehicle_.vy_*PATH_DURATION;
    double nextSFFuturePosX = nextSF.vx_*PATH_DURATION;
    double nextSFFuturePosY = nextSF.vy_*PATH_DURATION;
    
    double nextSFCurrentDist = distance(vehicle_.x_, vehicle_.y_, nextSF.x_, nextSF.y_);
    double nextSFFutureDist = distance(vehicleFuturePosX, vehicleFuturePosY, nextSFFuturePosX, nextSFFuturePosY);
    
    if (nextSFCurrentDist > CURRENT_NEXT_DIST_THRESHOLD &&
        nextSFFutureDist > FUTURE_NEXT_DIST_THRESHOLD) {
      cost += 0.1;
      // continue with same speed
      // calculate future location here
      stInfo.d_ = vehicle_.d_;
      stInfo.s_ = vehicle_.s_*PATH_DURATION;
      stInfo.speed_ = vehicle_.speed_;
    } else if (nextSFCurrentDist < DISTANCE_THRESHOLD) {
      // current distance gap is low and thus cost is very high
      cost += 0.8;
    } else if (nextSFFutureDist < DISTANCE_THRESHOLD) {
      // future distance gap is low and thus cost is high
      cost += 0.6;
    }
  } else {
    cost = 0;
  }
  return cost;
}

double PathPlanner::getLaneChangeCost(StateInfo &stInfo, bool &stInfoAvailable, const State state) {
  double cost = 0;
  SFVehicleInfo nextSF;
  SFVehicleInfo prevSF;
  stInfoAvailable = false;
  double nextLane;
  std::string laneName;
  double currentLane = getLane(vehicle_.d_);
  if (state == State::LEFT_CHANGE) {
    nextLane = currentLane -1;
    laneName = "left";
  } else if (state == State::RIGHT_CHANGE) {
    nextLane = currentLane +1;
    laneName = "right";
  } else {
    // Invalid state
    cost = 0.9;
    return cost;
  }
  double nextFound = getClosestVehicle(nextSF, nextLane, true);
  double prevFound = getClosestVehicle(prevSF, nextLane, false);
  
  // calculation of next x,y coordinates of vehicle in left lane after 2 seconds
  double angle = 60; // in degrees
  double next_s = vehicle_.s_ + 4*tan(rad2deg(angle));
  double next_d;
  if (state == State::LEFT_CHANGE) {
    next_d = 2; // mid of left lane
  } else {
    next_d = 6; // mid of right lane
  }
  vector<double> XY = getXY(next_s, next_d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  double vehicleFuturePosX = XY[0];
  double vehicleFuturePosY = XY[1];
  
  // Calculate equivalent X,Y coordinates in left lane if vehicle would have been there.
  XY = getXY(vehicle_.s_, next_d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  double vehicleCurPosX = XY[0];
  double vehicleCurPosY = XY[1];
  
  if (nextFound && prevFound) {
    std::cout<<"In the " << laneName << " lane, vehicle is in front as well as in back"<<std::endl;
    double nextSFFuturePosX = nextSF.vx_*PATH_DURATION;
    double nextSFFuturePosY = nextSF.vy_*PATH_DURATION;
    double prevSFFuturePosX = prevSF.vx_*PATH_DURATION;
    double prevSFFuturePosY = prevSF.vy_*PATH_DURATION;
    
    double nextSFCurrentDist = distance(vehicleCurPosX, vehicleCurPosY, nextSF.x_, nextSF.y_);
    double nextSFFutureDist = distance(vehicleFuturePosX, vehicleFuturePosY, nextSFFuturePosX, nextSFFuturePosY);
    double previousSFCurrentDist = distance(vehicleCurPosX, vehicleCurPosY, prevSF.x_, prevSF.y_);
    double prevSFFutureDist = distance(vehicleFuturePosX, vehicleFuturePosY, prevSFFuturePosX, prevSFFuturePosX);
    
    // check if we can go to a comfortable distance between two cars
    if (nextSFCurrentDist > CURRENT_NEXT_DIST_THRESHOLD &&
        nextSFFutureDist > FUTURE_NEXT_DIST_THRESHOLD &&
        previousSFCurrentDist > CURRENT_PREV_DIST_THRESHOLD &&
        prevSFFutureDist > FUTURE_PREV_DIST_THRESHOLD) {
      cost += 0.1;
      stInfoAvailable = true;
      // calculate future location here
      stInfo.x_ = vehicleFuturePosX;
      stInfo.y_ = vehicleFuturePosY;
      double nextVehicleVel = std::sqrt(nextSF.vx_*nextSF.vx_ + nextSF.vy_*nextSF.vy_);
      stInfo.speed_ = nextVehicleVel; // keep the speed as speed of next vehicle
    } else {
      cost += 0.6;
    }
  } else if (nextFound) {
    std::cout<<"In the " << laneName << " lane, vehicle is only in the front"<<std::endl;
    double nextSFFuturePosX = nextSF.vx_*PATH_DURATION;
    double nextSFFuturePosY = nextSF.vy_*PATH_DURATION;
  
    double nextSFCurrentDist = distance(vehicleCurPosX, vehicleCurPosY, nextSF.x_, nextSF.y_);
    double nextSFFutureDist = distance(vehicleFuturePosX, vehicleFuturePosY, nextSFFuturePosX, nextSFFuturePosY);
   
    if (nextSFCurrentDist > CURRENT_NEXT_DIST_THRESHOLD &&
        nextSFFutureDist > FUTURE_NEXT_DIST_THRESHOLD) {
      cost += 0.1;
      stInfoAvailable = true;
      // calculate future location here
      stInfo.x_ = vehicleFuturePosX;
      stInfo.y_ = vehicleFuturePosY;
      double nextVehicleVel = std::sqrt(nextSF.vx_*nextSF.vx_ + nextSF.vy_*nextSF.vy_);
      stInfo.speed_ = nextVehicleVel; // keep the speed as speed of next vehicle
    } else {
        cost += 0.6;
    }
  } else if (prevFound) {
    std::cout<<"In the " << laneName << " lane, vehicle is only in the back"<<std::endl;
    double prevSFFuturePosX = prevSF.vx_*PATH_DURATION;
    double prevSFFuturePosY = prevSF.vy_*PATH_DURATION;
    
    double previousSFCurrentDist = distance(vehicleCurPosX, vehicleCurPosY, prevSF.x_, prevSF.y_);
    double prevSFFutureDist = distance(vehicleFuturePosX, vehicleFuturePosY, prevSFFuturePosX, prevSFFuturePosX);
    
    if (previousSFCurrentDist > CURRENT_PREV_DIST_THRESHOLD &&
        prevSFFutureDist > FUTURE_NEXT_DIST_THRESHOLD) {
      cost += 0.1;
      stInfoAvailable = true;
      // calculate future location here
      stInfo.x_ = vehicleFuturePosX;
      stInfo.y_ = vehicleFuturePosY;
      double nextVehicleVel = std::sqrt(prevSF.vx_*prevSF.vx_ + prevSF.vy_*prevSF.vy_);
      stInfo.speed_ = nextVehicleVel; // keep the speed as speed of next vehicle
    } else {
        cost += 0.6;
    }
  } else {
    // No vehicle in the left lane nearby, so lane change can happen easily
    std::cout<<"In the " << laneName << " lane, there are no vehicles"<<std::endl;
    cost += 0.1;
    stInfoAvailable = true;
    // calculate future location here
    stInfo.x_ = vehicleFuturePosX;
    stInfo.y_ = vehicleFuturePosY;
    stInfo.speed_ = 24; // keep the speed as maximum speed within limits
  }
  return cost;
}

bool PathPlanner::getClosestVehicle(SFVehicleInfo &sFVehicle, double lane, bool front) {
  bool found = false;
  vector<SFVehicleInfo> currentLaneVehicles;
  for (auto &sFObj: sensorFusionList_) {
    sFObj.lane_ = getLane(sFObj.d_);
    if (sFObj.lane_ == lane) {
      currentLaneVehicles.push_back(sFObj);
    }
  }
  if (currentLaneVehicles.size() == 0) {
    return found;
  }
  double minDistance = std::numeric_limits<double>::max();
  for (auto &obj: currentLaneVehicles) {
    double diff = obj.s_ - vehicle_.s_;
    double modDiff;// = abs(diff);
    if (front) {
      if ((diff > 0) && (modDiff < minDistance)) {
        minDistance = modDiff;
        sFVehicle = obj;
        found = true;
      }
    } else {
      if ((diff <= 0) && (modDiff < minDistance)) {
        minDistance = modDiff;
        sFVehicle = obj;
        found = true;
      }
    }
  }
  return found;
}

// Fit a polynomial.
// Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd PathPlanner::polyfit(vector<double> xvals, vector<double> yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);

  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (unsigned int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (unsigned int j = 0; j < xvals.size(); j++) {
    for (unsigned int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals[j];
    }
  }

  auto Q = A.householderQr();
  Eigen::VectorXd Yvals = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(yvals.data(), yvals.size());
  auto result = Q.solve(Yvals);
  return result;
}

vector<double> PathPlanner::JMT(const vector<double> &start, const vector<double> &end, const double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T,5*T*T*T*T,
       6*T, 12*T*T, 20*T*T*T;
    
  MatrixXd B = MatrixXd(3,1);     
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];
          
  MatrixXd Ai = A.inverse();
  
  MatrixXd C = Ai*B;
  
  vector <double> result = {start[0], start[1], .5*start[2]};

  for(int i = 0; i < C.size(); ++i) {
    result.push_back(C.data()[i]);
  }

  return result;
}

bool PathPlanner::getPathCoordinates(const StateInfo &nextStateInfo, vector<double> &xValues, vector<double> &yValues) {
 vector<double> startFrenetS;
 vector<double> endFrenetS;
 vector<double> startFrenetD;
 vector<double> endFrenetD;
 
 startFrenetS.push_back(vehicle_.s_);
 startFrenetS.push_back(vehicle_.speed_);
 startFrenetS.push_back(0);
 endFrenetS.push_back(nextStateInfo.s_);
 endFrenetS.push_back(vehicle_.speed_);
 endFrenetS.push_back(0);
   
 startFrenetD.push_back(vehicle_.d_);
 startFrenetD.push_back(0);
 startFrenetD.push_back(0);
 endFrenetD.push_back(nextStateInfo.d_);
 endFrenetD.push_back(0);
 endFrenetD.push_back(0);
    
 double time = 2; // 2 seconds
 vector<double> sFrenetCoeffs = JMT(startFrenetS, endFrenetS, 2);
 vector<double> dFrenetCoeffs = JMT(startFrenetD, endFrenetD, 2);
 
 double numOfPoints = (time*1000)/20; // as points need to be captured for every 20ms
 double t = 0;
 for (int i = 0; i < numOfPoints; i++) {
   double s = sFrenetCoeffs[0] + sFrenetCoeffs[1]*t + sFrenetCoeffs[2]*t*t + 
     sFrenetCoeffs[3]*t*t*t + sFrenetCoeffs[4]*t*t*t*t + sFrenetCoeffs[5]*t*t*t*t*t;
   double d = dFrenetCoeffs[0] + dFrenetCoeffs[1]*t + dFrenetCoeffs[2]*t*t + 
     dFrenetCoeffs[3]*t*t*t + dFrenetCoeffs[4]*t*t*t*t + dFrenetCoeffs[5]*t*t*t*t*t;
   vector<double> XY = getXY(s, d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
   xValues.push_back(XY[0]);
   yValues.push_back(XY[1]); 
  }
  return true;
}
