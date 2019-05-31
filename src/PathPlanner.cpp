#include "PathPlanner.hpp"
#include <iostream>
#include <cstdlib>
#include "helpers.h"
#include "spline.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define DISTANCE_THRESHOLD 5
#define PATH_DURATION 2
#define LANE_CHANGE_DURATION 2.5
#define MAX_ACCELERATION 10
#define MAX_VELOCITY 21
#define MIN_PATH_SIZE_SAME_LANE 30
#define MIN_PATH_SIZE_LANE_CHANGE 6
#define OLD_PATH_SIZE_LANE_CHANGE 6
#define PLANNING_DISTANCE 30
#define MAX_FRENET_S 6945.554
#define CONTROLLER_UPDATE_RATE_SECONDS 0.02

#define CURRENT_NEXT_DIST_THRESHOLD 20
#define CURRENT_PREV_DIST_THRESHOLD 20
#define FUTURE_NEXT_DIST_THRESHOLD 3
#define FUTURE_PREV_DIST_THRESHOLD 3
#define DIFF_LANE_DIST_THRESHOLD 25

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
  laneChangePoints_ = 0;
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
    if (laneChangeInProgress_ && (laneChangePoints_ > 0)) {
      laneChangePoints_ = laneChangePoints_ -  pointsConsumed;
      if (laneChangePoints_ <=0) {
        laneChangeInProgress_ = false;
        laneChangePoints_ = 0;
      }
    }
  }
  
  if (previousPathSize == 0) {
    StateInfo nextStateInfo;
    nextStateInfo.d_ = vehicle_.d_;
    nextStateInfo.s_ = vehicle.s_ + 0.5* PATH_DURATION*PATH_DURATION*(MAX_ACCELERATION-6); // S = ut + 0.5*a*t*t
    nextStateInfo.speed_ = vehicle.speed_ + (MAX_ACCELERATION-6)*PATH_DURATION; // v = u + at
    nextStateInfo.numOldPoints_ = 0;
    nextStateInfo.numNewPoints_ = PATH_DURATION/0.02;
    getTrajectory(nextStateInfo);
    return currentTrajectory_;
  }
  
  // If lane change is in progress, then use already computed path
  if (laneChangeInProgress_ && previousPathSize > OLD_PATH_SIZE_LANE_CHANGE) {
    std::cout<<"lane change in progress, path size is: " << previousPathSize << std::endl;
    return currentTrajectory_;
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
    getTrajectory(nextStateInfo);
    laneChangeInProgress_ = true;
    laneChangePoints_ = currentTrajectory_.size();
  } else if(nextStateAvailable) { // same lane
    if (previousPathSize > MIN_PATH_SIZE_SAME_LANE) {
      // use previous path only
    } else {
      // Add remaining points here
      getTrajectory(nextStateInfo);
    }
  } else {
   // need to stop car smoothly here.. 
    std::cout<<"No next state available"<<std::endl;
  }
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

// approximate speed after PATH_DURATION time
double PathPlanner::getNormalFutureSpeed() {
  double nextSpeed = 0;
  double lastPointSpeed = 0;
  int trajSize = currentTrajectory_.size();
  if (trajSize == 0) {
   lastPointSpeed = vehicle_.speed_;
  } else {
    lastPointSpeed = currentTrajectory_.s_vels_[trajSize-1];
  }
  if (lastPointSpeed == 0) {
    nextSpeed = 8;
  } else{
    double mult = 1.05;
    if (lastPointSpeed < 10) {
      mult = 1.1;
    }
    // keep on increasing speed until 19
    nextSpeed = lastPointSpeed*mult;
  }
  if (nextSpeed > 19) {
   nextSpeed = 19; 
  }
  return nextSpeed;
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
      stInfo.d_ = getRoundOffD(vehicle_.d_);
      stInfo.speed_ = getNormalFutureSpeed();
      stInfo.s_ = currentTrajectory_.ss_[trajSize-1] + stInfo.speed_*remainingPoints*0.02;
      stInfo.numOldPoints_ = trajSize;
      stInfo.numNewPoints_ = remainingPoints;
      stInfoAvailable = true;
    } else {
      // current distance gap is low and thus cost is very high, drive with next vehicle's speed
      stInfo.d_ = getRoundOffD(vehicle_.d_);
      double nextVehicleVel = std::sqrt(nextSF.vx_*nextSF.vx_ + nextSF.vy_*nextSF.vy_);
      if (nextVehicleVel < vehicle_.speed_) {
        stInfo.speed_ = nextVehicleVel;
      } else {
         stInfo.speed_ = getNormalFutureSpeed();
      }
      stInfo.s_ = currentTrajectory_.ss_[trajSize-1] + stInfo.speed_*remainingPoints*0.02;
      stInfo.numOldPoints_ = trajSize;
      stInfo.numNewPoints_ = remainingPoints;
      stInfoAvailable = true;
      cost += 0.4;
    }
  } else {
    // No vehicles in the front, so drive with maximum allowed speed
    cost = 0;
    stInfo.d_ = getRoundOffD(vehicle_.d_);
    stInfo.speed_ = getNormalFutureSpeed();
    stInfo.s_ = currentTrajectory_.ss_[trajSize-1] + stInfo.speed_*remainingPoints*0.02;
    stInfo.numOldPoints_ = trajSize;
    stInfo.numNewPoints_ = remainingPoints;
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
  double next_s = 0;
  int trajSize = currentTrajectory_.size();
  stInfo.numNewPoints_ = LANE_CHANGE_DURATION/0.02;
  
  if (trajSize == 0) {
    next_s = vehicle_.s_ + vehicle_.speed_ * PATH_DURATION * tan(rad2deg(angle));
    stInfo.numOldPoints_ = 0;
  } else if (trajSize < OLD_PATH_SIZE_LANE_CHANGE) {
    next_s = currentTrajectory_.ss_[trajSize-1] + vehicle_.speed_ * PATH_DURATION * tan(rad2deg(angle));
    stInfo.numOldPoints_ = trajSize;
  } else {
    next_s = currentTrajectory_.ss_[OLD_PATH_SIZE_LANE_CHANGE-1] + vehicle_.speed_ * PATH_DURATION * tan(rad2deg(angle));
    stInfo.numOldPoints_ = OLD_PATH_SIZE_LANE_CHANGE;
  }
  
  vector<double> XY = toRealWorldXY(next_s, next_d);
  double vehicleFuturePosX = XY[0];
  double vehicleFuturePosY = XY[1];
  
  // Calculate equivalent X,Y coordinates in left/right lane if vehicle would have been there.
  XY = toRealWorldXY(vehicle_.s_, next_d);
  double vehicleCurPosX = XY[0];
  double vehicleCurPosY = XY[1];
  
  double nextFound = getClosestVehicle(nextSF, nextLane, true);
  double prevFound = getClosestVehicle(prevSF, nextLane, false);
  
  if (nextFound && prevFound) {
    std::cout<<"In the " << laneName << " lane, vehicle is in front as well as in back"<<std::endl;
    double nextSFFuturePosX = nextSF.vx_*LANE_CHANGE_DURATION;
    double nextSFFuturePosY = nextSF.vy_*LANE_CHANGE_DURATION;
    double prevSFFuturePosX = prevSF.vx_*LANE_CHANGE_DURATION;
    double prevSFFuturePosY = prevSF.vy_*LANE_CHANGE_DURATION;
    
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
      //stInfo.speed_ = vehicle_.speed_; // keep the speed same
      stInfo.speed_ = getNormalFutureSpeed();
    } else {
      cost += 0.6;
    }
  } else if (nextFound) {
    std::cout<<"In the " << laneName << " lane, vehicle is only in the front"<<std::endl;
    double nextSFFuturePosX = nextSF.vx_*LANE_CHANGE_DURATION;
    double nextSFFuturePosY = nextSF.vy_*LANE_CHANGE_DURATION;
  
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
      //stInfo.speed_ = vehicle_.speed_; // keep the speed same
      stInfo.speed_ = getNormalFutureSpeed();
    } else {
        cost += 0.6;
    }
  } else if (prevFound) {
    std::cout<<"In the " << laneName << " lane, vehicle is only in the back"<<std::endl;
    double prevSFFuturePosX = prevSF.vx_*LANE_CHANGE_DURATION;
    double prevSFFuturePosY = prevSF.vy_*LANE_CHANGE_DURATION;
    
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
      //stInfo.speed_ = vehicle_.speed_; // keep the speed same
      stInfo.speed_ = getNormalFutureSpeed();
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
    //stInfo.speed_ = vehicle_.speed_; // keep the speed as maximum speed within limits
    stInfo.speed_ = getNormalFutureSpeed();
  }
  if (laneChangeInProgress_) {
    // If lane Change is already in progress, do not do one more lane change
    // Instead reduce the speed of the vehicle
    cost = 0.9; 
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

Trajectory PathPlanner::getTrajectory(const StateInfo &nextStateInfo) {
  double last_x = vehicle_.x_;
  double last_y = vehicle_.y_;
  double last_s = vehicle_.s_;
  double last_d = vehicle_.d_;
  double last_s_vel = 0.0;
  double last_d_vel = 0.0;
  double last_s_acc = 0.0;
  double last_d_acc = 0.0;
  
  std::cout<<"PathPlanner::getTrajectory()"<<std::endl;
  std::cout<<"next state info-> s: " << nextStateInfo.s_ << ", d: " << nextStateInfo.d_ << std::endl;
  std::cout<<"next state info-> speed: " << nextStateInfo.speed_ << std::endl;
  std::cout<<"nextStateInfo-> old points: " << nextStateInfo.numOldPoints_ << ", new points: " << nextStateInfo.numNewPoints_ << std::endl;
  
  int trajSize = currentTrajectory_.size();
  
  if (nextStateInfo.numOldPoints_ < trajSize) {
    // some points needs to be discarded
    currentTrajectory_.keepFirstNPoints(nextStateInfo.numOldPoints_);
  } else if (nextStateInfo.numOldPoints_ > trajSize) {
    // Should not reach here, return previous trajectory here..
    return currentTrajectory_;
  }
  
  trajSize = currentTrajectory_.size();
  
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
   
  vector<double> startFrenetS = {last_s, last_s_vel, last_s_acc};
  vector<double> endFrenetS = {nextStateInfo.s_, nextStateInfo.speed_, 0};
  vector<double> startFrenetD = {last_d, last_d_vel, last_d_acc};
  vector<double> endFrenetD = {nextStateInfo.d_, 0, 0};

  vector<double> coeffs_s = JMT(startFrenetS, endFrenetS, nextStateInfo.numNewPoints_*CONTROLLER_UPDATE_RATE_SECONDS);
  vector<double> coeffs_d = JMT(startFrenetD, endFrenetD, nextStateInfo.numNewPoints_*CONTROLLER_UPDATE_RATE_SECONDS);

  for (int i = 0; i < nextStateInfo.numNewPoints_; i++) {
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

    currentTrajectory_.add(x_y[0], x_y[1], s_t, s_t_dot, s_t_dot_dot, s_jerk, d_t, d_t_dot, d_t_dot_dot, d_jerk, theta);
    double dist = distance(last_x, last_y, x, y);
    last_x = x;
    last_y = y;
    std::cout<<"For " << i << " point, s is: " << s_t << ", d is: " << d_t << std::endl;
  }
  std::cout<<"PathPlanner::getTrajectory(), size is: "<< currentTrajectory_.size() << std::endl;
  return currentTrajectory_;
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
