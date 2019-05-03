#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include <iostream>

using namespace std;

class Vehicle {
public:
    Vehicle();
    Vehicle(double x, double y, double s, double d, double yaw, double speed);
    void print();
    double x_;
    double y_;
    double s_;
    double d_;
    double yaw_;
    double speed_;
    double vx_;
    double vy_;
};

#endif // VEHICLE_HPP