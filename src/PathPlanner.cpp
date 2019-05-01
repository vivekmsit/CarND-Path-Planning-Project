#include "PathPlanner.hpp"
#include <iostream>
#include <cstdlib>
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

std::vector<Eigen::VectorXd> PathPlanner::computePath(Vehicle vehicle, vector<SFVehicleInfo> sfInfo, vector<Eigen::VectorXd> previous_path, double end_path_s, double end_path_d) {
  
  // store data into private member variables
  vehicle_ = vehicle;
  sensorFusionList_ = sfInfo;
  previousPath_ = previous_path;
  end_path_s_ = end_path_s;
  end_path_d_ = end_path_d;
  
  std::vector<Eigen::VectorXd> path;
  double dist_inc = 0.5;
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
  
  /*if (path_size >= 10) {
   return previous_path;
  }*/
  
  /*if (vehicle.speed_ == 0) {
    dist_inc = (24/1000)*20;
  } else {
    dist_inc = (vehicle.speed_/1000)*20;
  }*/
  
  /*double lChangeRequired = laneChangeRequired();
  if (lChangeRequired) {
    dist_inc = 0;
    SFVehicleInfo sfObj;
    double targetLane;
    bool vehicleToFollow = false;
    bool laneFound = getLaneChangeInfo(sfObj, targetLane, vehicleToFollow);
    if (laneFound) {
      double path = getLaneChangePath(sfObj, targetLane, vehicleToFollow);
    } else {
      // stop the vehicle
      dist_inc = 0;
    }
  }*/
  
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
  
  //previous_path_ = path;
  return path;
}

/*bool PathPlanner::laneChangeRequired() {
  SFVehicleInfo nextSF;
  bool lChangeRequired = false;
  double currentLane = getLane(vehicle_.d_);
  // get closest vehicle in front of the vehicle in current lane
  double found = getClosestVehicle(nextSF, currentLane, true);
  if (found) {
    double distance = nextSF.s_ - vehicle_.s_;
    //double speed_diff = nextSF.speed_ - vehicle_.speed_;
    if (distance > 50) { // or if speed of next vehicle is less 
      // continue in same lane
    } else {
      // either stop or reduce speed or change lane
      lChangeRequired = true;
    }
  } else {
      // continue in same lane with good speed
  }
  return lChangeRequired;
}*/


/*bool PathPlanner::getLaneChangeInfo(SFVehicleInfo &sfObj, double &targetLane, bool &vehicleToFollow){
  bool laneFound = false;
  double minDiff = 10;
  double currentLane = getLane(vehicle_.d_);
  vehicleToFollow = false;
  if (currentLane == 0) { // For left lane vehicle, middle lane is the only option
    SFVehicleInfo nextSF;
    double localFound = getClosestVehicle(nextSF, 1, true);
    if (localFound) {
      double diff = nextSF.s_ - vehicle_.s_;
      if (diff > minDiff) {
        sfObj = nextSF;
        targetLane = 0;
        laneFound = true;
        vehicleToFollow = true;
      }
    }
  } else if (currentLane == 1) { // for middle lane vehicle, left and right lane both are options
    SFVehicleInfo nextLeftSF;
    SFVehicleInfo nextrightSF;
    double leftFound = getClosestVehicle(nextLeftSF, 0, true);
    double rightFound = getClosestVehicle(nextrightSF, 2, true);
    if (leftFound && rightFound) {
      if (nextLeftSF.s_ > nextrightSF.s_) {
        double diff = nextLeftSF.s_ - vehicle_.s_;
        if (diff > minDiff) {
          sfObj = nextLeftSF;
          targetLane = 0;
          laneFound = true;
          vehicleToFollow = true;
        }
      } else {
        double diff = nextrightSF.s_ - vehicle_.s_;
        if (diff > minDiff) {
          sfObj = nextrightSF;
          targetLane = 2;
          laneFound = true;
          vehicleToFollow = true;
        }
      }
    } else if (leftFound) {
      double diff = nextLeftSF.s_ - vehicle_.s_;
      if (diff > minDiff) {
        sfObj = nextLeftSF;
        targetLane = 0;
        laneFound = true;
        vehicleToFollow = true;
      }
    } else if (rightFound) {
      double diff = nextrightSF.s_ - vehicle_.s_;
      if (diff > minDiff) {
        sfObj = nextrightSF;
        targetLane = 2;
        laneFound = true;
        vehicleToFollow = true;
      }
    } else {
      // Here we can switch to either left lane or right lane, we will choose right lane (2) here
      sfObj = nextrightSF;
      targetLane = 2;
      laneFound = true;
      vehicleToFollow = true;
    }
  } else { // For right lane vehicle, middle lane is the only option
    SFVehicleInfo nextSF;
    double localFound = getClosestVehicle(nextSF, 1, true);
    if (localFound) {
      double diff = nextSF.s_ - vehicle_.s_;
      if (diff > minDiff) {
        sfObj = nextSF;
        targetLane = 1;
        laneFound = true;
        vehicleToFollow = true;
      }
    } else { // no vehicles on middle lane, so can easily change lane
      targetLane = 1;
      laneFound = true;
    }
  }
  return laneFound;
}*/

double PathPlanner::getLaneChangePath(const SFVehicleInfo &sfObj, const double &targetLane, const bool &vehicleToFollow) {
  return 0;
}
