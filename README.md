# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Introduction

Path-Planning project is about using sensor fusion data to determine the trajectory of the vehicle which is having minimum jerk, no collisions with other vehicles, follows optimal speed and does not cross the lane boundaries except in case of lane change.

#### Input Data

For each cycle (20ms), we are provided sensor fusion data as well as information about the vehicle's location, frenet coordinates, velocity. In sensor fusion data, information (e.g. speed, location coordinates) is provided about all the nearby vehicles.

### Processing of Input Data

On each iteration, input data is stored in C++ objects and then processing is done using various path-planning algorithms. 

Following principles are used while doing path-planning:

* State Machine: state machine is used to determine the next available states of the vehicle. For a given information about the vehicle, not all states can be possible always. It depends on the current location/data about the vehicle to know about the next available states. 
* For each state machine, trajectory is determined and cost is allocated for each state. State having minimum cost among all available states will be selected.
* For the next state, trajectory calculation is done using five-degree polynomial equation for s and d frenet coordinates to minimize the average jerk. Also, spline library is used to correctly map s,d coordinates to real x,y coordinates.

Three states are used in the path-planning project:
1. Current Lane (CL)
2. Left Lane Change (LLC)
3. Right Lane Change (RLC)

For calculation of the cost of each state mentioned above, sensor fusion data is used to determine how easy it will be to move to the state.
e.g. For a LLC state, for all vehicles which are ahead of our vehicle in the target lane, distance is measured. If distance is more than a given threshold, then cost is low. Also, after LANE_CHANGE_DURATION, state of previous vehicle in the target lane is also determined to avoid any collision.

### Source Code Structure

* StateMachine - This class gives next available states for a given state.
* Trajectory - This class stores information about a given trajectory, which is collection of information about each point in the trajectory.
* PathPlanner - This class has all the logic about trajectory generation. This class uses spline libraries, matrix multiplication libraries to generate next trajectory.f

### Testing

Source code was tested rigorously with various set of parameters. e.g. Distance threshold between our vehicle and next vehicle in current lane and also in case of lane change. Testing was done to make sure there are no collisions while vehicle is running in autonomous mode and vehicle runs at a safe distance from other vehicles.

### github link

https://github.com/vivekmsit/CarND-Path-Planning-Project 

