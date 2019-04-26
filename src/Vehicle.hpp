#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include <iostream>

using namespace std;

class Vehicle {
public:
    Vehicle(int x, int y, int s, int d, int yaw, int speed);
    int x_;
    int y_;
    int s_;
    int d_;
    int yaw_;
    int speed_;
};

#endif // VEHICLE_HPP