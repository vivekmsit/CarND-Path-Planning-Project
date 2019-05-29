#include "PathPlanner.hpp"
#include <iostream>
#include <cstdlib>
#include "helpers.h"
#include "spline.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define DISTANCE_THRESHOLD 5
#define PATH_DURATION 2
#define MAX_ACCELERATION 10
#define MAX_VELOCITY 22
#define MIN_PATH_SIZE_SAME_LANE 30
#define MIN_PATH_SIZE_LANE_CHANGE 6
#define PLANNING_DISTANCE 30
#define MAX_FRENET_S 6945.554
#define CONTROLLER_UPDATE_RATE_SECONDS 0.02

#define CURRENT_NEXT_DIST_THRESHOLD 30
#define CURRENT_PREV_DIST_THRESHOLD 30
#define FUTURE_NEXT_DIST_THRESHOLD 3
#define FUTURE_PREV_DIST_THRESHOLD 3
#define DIFF_LANE_DIST_THRESHOLD 14

PathPlanner::PathPlanner(vector<double> map_waypoints_x,
                        vector<double> map_waypoints_y,
                        vector<double> map_waypoints_s,
                        vector<double> map_waypoints_dx,
                        vector<double> map_waypoints_dy): map_waypoints_x_(map_waypoints_x),
                                                          map_waypoints_y_(map_waypoints_y),
                                                          map_waypoints_s_(map_waypoints_s),
                                                          map_waypoints_dx_(map_waypoints_dx),
                                                          map_waypoints_dy_(map_waypoints_dy) {
  laneChangeInProgress_ = false;
  buildSplines();
}

Trajectory PathPlanner::computePath(Vehicle vehicle, vector<SFVehicleInfo> sfInfo, vector<Eigen::VectorXd> previous_path, double end_path_s, double end_path_d) {
  // store data into private member variables
  vehicle_ = vehicle;
  sensorFusionList_ = sfInfo;
  previousPath_ = previous_path;
  end_path_s_ = end_path_s;
  end_path_d_ = end_path_d;
  
  std::vector<Eigen::VectorXd> path;
  double dist_inc = 0.4;
  vehicle_.print();
  int previousPathSize = previousPath_.size();
  std::cout<<"size of previous path is: " << previousPathSize << std::endl;
  std::cout<<"end_path_s is: " << end_path_s_ << ", end_path_d: " << end_path_d_ << std::endl;
  
  double first_x;
  double first_y;
  double last_x;
  double last_y;
  double next_d;
  
  int pointsConsumed = currentTrajectory_.size() - previousPathSize;
  if (pointsConsumed > 0 ) {
    currentTrajectory_.removeFirstNPoints(pointsConsumed);
  }
  
  if (previousPathSize == 0) {
    StateInfo nextStateInfo;
    nextStateInfo.d_ = vehicle_.d_;
    nextStateInfo.s_ = vehicle.s_ + 0.5* PATH_DURATION*PATH_DURATION*(MAX_ACCELERATION-2); // S = ut + 0.5*a*t*t
    nextStateInfo.speed_ = vehicle.speed_ + (MAX_ACCELERATION-2)*PATH_DURATION; // v = u + at
    vector<double> xValues;
    vector<double> yValues;
    getTrajectory(nextStateInfo, xValues, yValues);
    return currentTrajectory_;
  }
  
  first_x = previousPath_[0][0];
  first_y = previousPath_[0][1];
  last_x = previousPath_[previousPathSize-1][0];
  last_y = previousPath_[previousPathSize-1][1];
  std::cout<<"first x position is: " << first_x << " and first y position is: " << first_y << std::endl;
  std::cout<<"last x position is: " << last_x << " and last y position is: " << last_y << std::endl; 
  
  // If lane change is in progress, then use already computed path
  if (laneChangeInProgress_ && previousPathSize > MIN_PATH_SIZE_LANE_CHANGE) {
    std::cout<<"lane change in progress, path size is: " << previousPathSize << std::endl;
    return currentTrajectory_;
  } else if(laneChangeInProgress_ && previousPathSize <= MIN_PATH_SIZE_LANE_CHANGE) {
    laneChangeInProgress_ = false;
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
       nextState = state;
      }
      std::cout<<"cost of state " << state << " is: " << stateCost << std::endl;
    }
  }
  std::cout<<"next state is: "<< nextState << std::endl;
  
  if (nextStateAvailable && (nextState == LEFT_CHANGE || nextState == RIGHT_CHANGE)) {
    vector<double> xValues;
    vector<double> yValues;
    //path.clear();
    currentTrajectory_.keepFirstNPoints(5);
    getTrajectory(nextStateInfo, xValues, yValues);
    laneChangeInProgress_ = true;
  } else if(nextStateAvailable) { // same lane
    if (previousPathSize > MIN_PATH_SIZE_SAME_LANE) {
      // use previous path only
    } else {
      // Add remaining points here
      vector<double> xValues;
      vector<double> yValues;
      getTrajectory(nextStateInfo, xValues, yValues);
    }
  } else {
   // need to stop car smoothly here.. 
    std::cout<<"No next state available"<<std::endl;
  }
  
  /*int final_size = path.size();
  std::cout<<"final path size is: " << final_size << std::endl;
  
  if (final_size != 0) {
    auto obj = path.back();
    std::cout<<"last co-ordinates of new path are: x: " << obj[0];
    std::cout<<" and y: " << obj[1] << std::endl;
  }
  std::cout<<std::endl;*/
  
  return currentTrajectory_;
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
  int trajSize = currentTrajectory_.size();
  int remainingPoints = 100 - trajSize;
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
    std::cout<<"In current lane, next vehicle is " << nextSFCurrentDist << " distance away" << std::endl;    
    if (nextSFCurrentDist > CURRENT_NEXT_DIST_THRESHOLD) {
      // drive with maximum allowed speed
      stInfo.d_ = vehicle_.d_;
      stInfo.s_ = currentTrajectory_.ss_[trajSize-1] + vehicle_.speed_*remainingPoints*0.02;
      stInfo.speed_ = MAX_VELOCITY-1;
      stInfoAvailable = true;
    } else {
      // current distance gap is low and thus cost is very high, drive with next vehicle's speed
      stInfo.d_ = vehicle_.d_;
      double nextVehicleVel = std::sqrt(nextSF.vx_*nextSF.vx_ + nextSF.vy_*nextSF.vy_);
      stInfo.s_ = currentTrajectory_.ss_[trajSize-1] + nextVehicleVel*remainingPoints*0.02;
      stInfo.speed_ = nextVehicleVel;
      stInfoAvailable = true;
      cost += 0.4;
    }
  } else {
    // No vehicles in the front, so drive with maximum allowed speed
    cost = 0;
    stInfo.d_ = vehicle_.d_;
    stInfo.s_ = currentTrajectory_.ss_[trajSize-1] + vehicle_.speed_*remainingPoints*0.02;
    stInfo.speed_ = MAX_VELOCITY-1;
    stInfoAvailable = true;
  }
  return cost;
}

double PathPlanner::getLaneChangeCost(StateInfo &stInfo, bool &stInfoAvailable, const State state) {
  double cost = 0;
  SFVehicleInfo nextSF;
  SFVehicleInfo prevSF;
  stInfoAvailable = false;
  double nextLane;
  double next_d;
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
  
  next_d = nextLane*4 + 2;
  // calculation of next x,y coordinates of vehicle in left lane after 2 seconds
  double angle = 60; // in degrees
  //double next_s = vehicle_.s_ + 4*tan(rad2deg(angle));
  //double next_s = vehicle_.s_ + 60;
  double next_s = vehicle_.s_ + vehicle_.speed_ * PATH_DURATION * tan(rad2deg(angle));

  vector<double> XY = getXY(next_s, next_d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  double vehicleFuturePosX = XY[0];
  double vehicleFuturePosY = XY[1];
  
  // Calculate equivalent X,Y coordinates in left/right lane if vehicle would have been there.
  XY = getXY(vehicle_.s_, next_d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  double vehicleCurPosX = XY[0];
  double vehicleCurPosY = XY[1];
  
  double nextFound = getClosestVehicle(nextSF, nextLane, true);
  double prevFound = getClosestVehicle(prevSF, nextLane, false);
  
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
    
    std::cout<<"In " << laneName << " lane, next vehicle is " << nextSFCurrentDist << " distance away"<<std::endl;
    std::cout<<"In " << laneName << " lane, previous vehicle is " << previousSFCurrentDist << " distance away"<<std::endl;
    // check if we can go to a comfortable distance between two cars
    if (nextSFCurrentDist > DIFF_LANE_DIST_THRESHOLD &&
        nextSFFutureDist > DIFF_LANE_DIST_THRESHOLD &&
        previousSFCurrentDist > DIFF_LANE_DIST_THRESHOLD &&
        prevSFFutureDist > DIFF_LANE_DIST_THRESHOLD) {
      cost += 0.1;
      stInfoAvailable = true;
      // calculate future location here
      stInfo.x_ = vehicleFuturePosX;
      stInfo.y_ = vehicleFuturePosY;
      stInfo.s_ = next_s;
      stInfo.d_ = next_d;
      double nextVehicleVel = std::sqrt(nextSF.vx_*nextSF.vx_ + nextSF.vy_*nextSF.vy_);
      // stInfo.speed_ = nextVehicleVel; // keep the speed as speed of next vehicle
      stInfo.speed_ = vehicle_.speed_; // keep the speed same
    } else {
      cost += 0.6;
    }
  } else if (nextFound) {
    std::cout<<"In the " << laneName << " lane, vehicle is only in the front"<<std::endl;
    double nextSFFuturePosX = nextSF.vx_*PATH_DURATION;
    double nextSFFuturePosY = nextSF.vy_*PATH_DURATION;
  
    double nextSFCurrentDist = distance(vehicleCurPosX, vehicleCurPosY, nextSF.x_, nextSF.y_);
    double nextSFFutureDist = distance(vehicleFuturePosX, vehicleFuturePosY, nextSFFuturePosX, nextSFFuturePosY);
   
    std::cout<<"In " << laneName << " lane, next vehicle is " << nextSFCurrentDist << " distance away"<<std::endl;
    if (nextSFCurrentDist > DIFF_LANE_DIST_THRESHOLD &&
        nextSFFutureDist > FUTURE_NEXT_DIST_THRESHOLD) {
      cost += 0.1;
      stInfoAvailable = true;
      // calculate future location here
      stInfo.x_ = vehicleFuturePosX;
      stInfo.y_ = vehicleFuturePosY;
      stInfo.s_ = next_s;
      stInfo.d_ = next_d;
      double nextVehicleVel = std::sqrt(nextSF.vx_*nextSF.vx_ + nextSF.vy_*nextSF.vy_);
      //stInfo.speed_ = nextVehicleVel; // keep the speed as speed of next vehicle
      stInfo.speed_ = vehicle_.speed_; // keep the speed same
    } else {
        cost += 0.6;
    }
  } else if (prevFound) {
    std::cout<<"In the " << laneName << " lane, vehicle is only in the back"<<std::endl;
    double prevSFFuturePosX = prevSF.vx_*PATH_DURATION;
    double prevSFFuturePosY = prevSF.vy_*PATH_DURATION;
    
    double previousSFCurrentDist = distance(vehicleCurPosX, vehicleCurPosY, prevSF.x_, prevSF.y_);
    double prevSFFutureDist = distance(vehicleFuturePosX, vehicleFuturePosY, prevSFFuturePosX, prevSFFuturePosX);
    
    std::cout<<"In " << laneName << " lane, previous vehicle is " << previousSFCurrentDist << " distance away"<<std::endl;
    if (previousSFCurrentDist > CURRENT_PREV_DIST_THRESHOLD &&
        prevSFFutureDist > FUTURE_NEXT_DIST_THRESHOLD) {
      cost += 0.1;
      stInfoAvailable = true;
      // calculate future location here
      stInfo.x_ = vehicleFuturePosX;
      stInfo.y_ = vehicleFuturePosY;
      stInfo.s_ = next_s;
      stInfo.d_ = next_d;
      double nextVehicleVel = std::sqrt(prevSF.vx_*prevSF.vx_ + prevSF.vy_*prevSF.vy_);
      //stInfo.speed_ = nextVehicleVel; // keep the speed as speed of next vehicle
      stInfo.speed_ = vehicle_.speed_; // keep the speed same
    } else {
        cost += 0.6;
    }
  } else {
    // No vehicle in the left/right lane nearby, so lane change can happen easily
    std::cout<<"In the " << laneName << " lane, there are no vehicles"<<std::endl;
    cost += 0.1;
    stInfoAvailable = true;
    // calculate future location here
    stInfo.x_ = vehicleFuturePosX;
    stInfo.y_ = vehicleFuturePosY;
    stInfo.s_ = next_s;
    stInfo.d_ = next_d;
    stInfo.speed_ = vehicle_.speed_; // keep the speed as maximum speed within limits
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
    double modDiff = abs(diff);
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

void PathPlanner::smoothPolynomial(vector<double> inXValues, vector<double> inYValues, vector<double> &xValues, vector<double> &yValues) {
  Eigen::VectorXd coeffs = polyfit(inXValues, inYValues, 2);
  for (int i = 0; i < inXValues.size(); i++) {
    double xValue = inXValues[i];
    xValues.push_back(xValue);
    double yValue = coeffs[0] + coeffs[1]*xValue + coeffs[2]*xValue*xValue;
    yValues.push_back(yValue);
  }
}

Trajectory PathPlanner::getTrajectory(const StateInfo &nextStateInfo, vector<double> &xValues, vector<double> &yValues) {
  std::cout<<"PathPlanner::getTrajectory()"<<std::endl;
  return getJMTTrajectory(nextStateInfo, xValues, yValues);
  
  //if ((vehicle_.d_ != nextStateInfo.d_) || (vehicle_.speed_ != nextStateInfo.speed_)) {
    // Use JMT trajectory here
    //status = getJMTTrajectory(nextStateInfo, xValues, yValues);
    /*vector<double> localXValues;
    vector<double> localYValues;
    status = getJMTTrajectory(nextStateInfo, localXValues, localYValues);
    smoothPolynomial(localXValues, localYValues, xValues, yValues);*/
  //} else {
    // Use spline trajectory here as we have to continue driving in the same lane with same speed
    //status = getSplineTrajectory(nextStateInfo, xValues, yValues);
  //}
  //return status;
}

Trajectory PathPlanner::getJMTTrajectory(const StateInfo &nextStateInfo, vector<double> &xValues, vector<double> &yValues) {
  double last_x = vehicle_.x_;
  double last_y = vehicle_.y_;
  double last_s = vehicle_.s_;
  double last_d = vehicle_.d_;
  double last_s_vel = 0.0;
  double last_d_vel = 0.0;
  double last_s_acc = 0.0;
  double last_d_acc = 0.0;
  
  int trajSize = currentTrajectory_.size();
  
  if (trajSize > 0) {
    last_x = currentTrajectory_.xs_[trajSize - 1];
    last_y = currentTrajectory_.ys_[trajSize - 1];
    last_s = currentTrajectory_.ss_[trajSize - 1];
    last_d = currentTrajectory_.ds_[trajSize - 1];
    last_s_vel = currentTrajectory_.s_vels_[trajSize - 1];
    last_d_vel = currentTrajectory_.d_vels_[trajSize - 1];
    last_s_acc = currentTrajectory_.s_accs_[trajSize - 1];
    last_d_acc = currentTrajectory_.d_accs_[trajSize - 1];
  }
  
  std::cout<<"next state info-> s: " << nextStateInfo.s_ << ", d: " << nextStateInfo.d_ << std::endl;
  
  vector<double> startFrenetS = {last_s, last_s_vel, last_s_acc};
  vector<double> endFrenetS = {nextStateInfo.s_, nextStateInfo.speed_, 0};
  vector<double> startFrenetD = {last_d, last_d_vel, last_d_acc};
  vector<double> endFrenetD = {nextStateInfo.d_, 0, 0};

  double totalPoints = (PATH_DURATION*1000)/20; // as points need to be captured for every 20ms
  double tIncrement = 0.020;
  int remainingPoints = totalPoints - trajSize;
  vector<double> coeffs_s = JMT(startFrenetS, endFrenetS, remainingPoints*tIncrement);
  vector<double> coeffs_d = JMT(startFrenetD, endFrenetD, remainingPoints*tIncrement);

  for (int i = 0; i < remainingPoints; i++) {
    double t = CONTROLLER_UPDATE_RATE_SECONDS * (i + 1);
    double t_2 = pow(t, 2);
    double t_3 = pow(t, 3);
    double t_4 = pow(t, 4);
    double t_5 = pow(t, 5);

    double s_t = startFrenetS[0] + startFrenetS[1] * t + 0.5 * startFrenetS[2] * t_2 + coeffs_s[3] * t_3 + coeffs_s[4] * t_4 + coeffs_s[5] * t_5;
    double s_t_dot = startFrenetS[1] + startFrenetS[2] * t + 3 * coeffs_s[3] * t_2 + 4 * coeffs_s[4] * t_3 + 5 * coeffs_s[5] * t_4;
    double s_t_dot_dot = startFrenetS[2] + 6 * coeffs_s[3] * t + 12 * coeffs_s[4] * t_2 + 20 * coeffs_s[5] * t_3;
    double s_jerk = 6 * coeffs_s[3] + 24 * coeffs_s[4] * t + 60 * coeffs_s[5] * t_2;

    double d_t = startFrenetD[0] + startFrenetD[1] * t + startFrenetD[2] * 0.5 * t_2 + coeffs_d[3] * t_3 + coeffs_d[4] * t_4 + coeffs_d[5] * t_5;
    double d_t_dot = startFrenetD[1] + startFrenetD[2] * t + 3 * coeffs_d[3] * t_2 + 4 * coeffs_d[4] * t_3 + 5 * coeffs_d[5] * t_4;
    double d_t_dot_dot = startFrenetD[2] + 6 * coeffs_d[3] * t + 12 * coeffs_d[4] * t_2 + 20 * coeffs_d[5] * t_3;
    double d_jerk = 6 * coeffs_d[3] + 24 * coeffs_d[4] * t + 60 * coeffs_d[5] * t_2;

    vector<double> x_y = toRealWorldXY(s_t, d_t);
    double x = x_y[0];
    double y = x_y[1];
    double theta = atan2(y - last_y, x - last_x);
    // TODO fix the theta angle
    currentTrajectory_.add(x_y[0], x_y[1], s_t, s_t_dot, s_t_dot_dot, s_jerk, d_t, d_t_dot, d_t_dot_dot, d_jerk, theta);
    double dist = distance(last_x, last_y, x, y);
    last_x = x;
    last_y = y;
    std::cout<<"For " << i << " point, s is: " << s_t << ", d is: " << d_t << std::endl;
  }
  std::cout<<"PathPlanner::getJMTTrajectory(), size is: "<< currentTrajectory_.size() << std::endl;
  return currentTrajectory_;
}

bool PathPlanner::getSplineTrajectory(const StateInfo &nextStateInfo, vector<double> &xValues, vector<double> &yValues) {
  bool status = false;
  const double MAX_SPEED = 49.5;
  const double MAX_ACC = .224;
  double pre_size = previousPath_.size();
  // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
  // Later we will interoplate these waypoints with a spline and fill it in with more points that control sp ...
  vector<double> ptsx;
  vector<double> ptsy;

  // reference x,y, yaw states
  // either we will reference the starting point as where the car is or at the previous path end point
  double ref_x = vehicle_.x_;
  double ref_y = vehicle_.y_;
  double ref_yaw = deg2rad(vehicle_.yaw_);
  double car_yaw_rad = deg2rad(vehicle_.yaw_);

  // If previous path is almost empty, use the car as starting reference
  if (pre_size < 2) {
    // use two points that make the path tangent to the car
    double prev_car_x = vehicle_.x_ - cos(vehicle_.yaw_);
    double prev_car_y = vehicle_.y_ - sin(vehicle_.yaw_);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(vehicle_.x_);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(vehicle_.y_);
  } else { // use the previous path's end point as starting reference
    // redefine reference state as previous path end point
    ref_x = previousPath_[pre_size - 1][0];
    ref_y = previousPath_[pre_size - 1][1];

    double ref_x_prev = previousPath_[pre_size - 2][0];
    double ref_y_prev = previousPath_[pre_size - 2][1];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // use two points that make the path tangent to the previous path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // In Frenet add evenly 30m spaced points ahead of the starting reference.
  for (int i = 0; i < 3; i++){
    double target_s = vehicle_.s_ + PLANNING_DISTANCE * (i + 1);
    if (target_s > MAX_FRENET_S) {
      target_s -= MAX_FRENET_S;
    }
    double target_d = vehicle_.d_;
    vector<double> next_wp = getXY(target_s, target_d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
    ptsx.push_back(next_wp[0]);
    ptsy.push_back(next_wp[1]);
  }
  
  // Making coordinates to local car coordinates.
  for ( int i = 0; i < ptsx.size(); i++ ) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }

  // create a spline
  tk::spline s;
  // set(x, y) points to the spline
  s.set_points(ptsx, ptsy);
  
  // Start with all of the previous path points from last time
  for (int i = 0; i < pre_size; i++){
    xValues.push_back(previousPath_[i][0]);
    yValues.push_back(previousPath_[i][1]);
  }
  
  // Calculate distance y position on 30 m ahead.
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);
  double x_add_on = 0;
  double ref_vel = vehicle_.speed_;

  for( int i = 1; i < 50 - pre_size; i++ ) {
    /*ref_vel += speed_diff;
    if ( ref_vel > MAX_SPEED ) {
      ref_vel = MAX_SPEED;
    } else if ( ref_vel < MAX_ACC ) {
      ref_vel = MAX_ACC;
    }*/
    double N = target_dist/(0.02*ref_vel/2.24);
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    xValues.push_back(x_point);
    yValues.push_back(y_point);
    std::cout<<"PathPlanner::getSplineTrajectory(), new x is:" << x_point << ", new y is: " << y_point <<std::endl;
  }
  return true;
}

void PathPlanner::buildSplines() {
  sp_x_s_.set_points(map_waypoints_s_, map_waypoints_x_);
  sp_y_s_.set_points(map_waypoints_s_, map_waypoints_y_);
  sp_dx_s_.set_points(map_waypoints_s_, map_waypoints_dx_);
  sp_dy_s_.set_points(map_waypoints_s_, map_waypoints_dy_);
}

vector<double> PathPlanner::toRealWorldXY(double s, double d) {
  s = fmod(s, MAX_TRACK_S);
  // Use the spline we have created to get a smoother path
  double x = sp_x_s_(s) + d * sp_dx_s_(s);
  double y = sp_y_s_(s) + d * sp_dy_s_(s);
  return {x, y};
}
