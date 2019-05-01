#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include <vector>

#include "Vehicle.hpp"
#include "SFVehicleInfo.hpp"

using namespace std;

enum State {
  CURRENT,
  LEFT_CHANGE,
  RIGHT_CHANGE
};

typedef struct {
  double x_;
  double y_;
  double s_;
  double d_;
  double speed_;
  double yaw_;
  double duration_;
} StateInfo;

class StateMachine {
public:
  StateMachine(Vehicle vehicle, vector<SFVehicleInfo> sfInfo);
  
  vector<State> getNextStates(double lane);
  double getStateCost(State state, StateInfo &stInfo);
  
private:
  double getLLCStateCost(StateInfo &stInfo);
  double getRLCStateCost(StateInfo &stInfo);
  double getCLStateCost(StateInfo &stInfo);
  
  bool getClosestVehicle(SFVehicleInfo &sFVehicle, double lane, bool front);
  
  Vehicle vehicle_;
  vector<SFVehicleInfo> sensorFusionList_;
};

#endif // STATE_MACHINE_HPP