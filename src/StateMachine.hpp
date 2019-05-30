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
  int numOldPoints_;
  int numNewPoints_;
} StateInfo;

class StateMachine {
public:
  StateMachine();
  
  vector<State> getNextStates(double lane);
};

#endif // STATE_MACHINE_HPP