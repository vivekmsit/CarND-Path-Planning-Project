#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include <iostream>

using namespace std;

class Vehicle {
public:
    Vehicle(double x, double y, double s, double d, double yaw, double speed);
    void print();
    double x_;
    double y_;
    double s_;
    double d_;
    double yaw_;
    double speed_;
};

#endif // VEHICLE_HPP