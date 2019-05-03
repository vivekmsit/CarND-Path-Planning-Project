#include "StateMachine.hpp"

#include <limits>
#include <cstdlib>

#include "helpers.h"

StateMachine::StateMachine(){
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