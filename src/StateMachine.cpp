#include "StateMachine.hpp"

#include <limits>
#include <cstdlib>

#include "helpers.h"

StateMachine::StateMachine(Vehicle vehicle, vector<SFVehicleInfo> sfInfo): vehicle_(vehicle), sensorFusionList_(sfInfo){
  
}

vector<State> StateMachine::getNextStates(double lane) {
  vector<State> nextStates;
  if (lane == 0) {
    nextStates.push_back(State::CURRENT);
    nextStates.push_back(State::RIGHT_CHANGE);
  } else if (lane == 1) {
    nextStates.push_back(State::CURRENT);
    nextStates.push_back(State::LEFT_CHANGE);
    nextStates.push_back(State::RIGHT_CHANGE);
  } else {
    nextStates.push_back(State::CURRENT);
    nextStates.push_back(State::LEFT_CHANGE);
  }
  return nextStates;
}

double StateMachine::getStateCost(State state, StateInfo &stInfo) {
  double cost = std::numeric_limits<double>::max();;
  switch(state) {
    case State::CURRENT:
      cost = getCLStateCost(stInfo);
      break;
    case State::LEFT_CHANGE:
      cost = getLLCStateCost(stInfo);
      break;
    case State::RIGHT_CHANGE:
      cost = getRLCStateCost(stInfo);
      break;
    default:
      break;
  }
  return cost;
}

double StateMachine::getCLStateCost(StateInfo &stInfo) {
  SFVehicleInfo nextSF;
  double cost = 0;
  double currentLane = getLane(vehicle_.d_);
  double localFound = getClosestVehicle(nextSF, currentLane, true);
  if (localFound) {
    cost += 0.1;
  } else {
    cost = 0;
  }
  return cost;
}

double StateMachine::getLLCStateCost(StateInfo &stInfo) {
  double cost = 1;
  return cost;
}

double StateMachine::getRLCStateCost(StateInfo &stInfo) {
  double cost = 1;
  return cost;
}

bool StateMachine::getClosestVehicle(SFVehicleInfo &sFVehicle, double lane, bool front) {
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