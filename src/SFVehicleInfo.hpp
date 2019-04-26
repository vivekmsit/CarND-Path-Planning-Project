#ifndef SFVEHICLE_INFO_HPP
#define SFVEHICLE_INFO_HPP

#include <iostream>

using namespace std;

class SFVehicleInfo {
public:
    SFVehicleInfo(int id, int x, int y, int vx, int vy, int s, int d);
    int id_;
    int x_;
    int y_;
    int vx_;
    int vy_;
    int s_;
    int d_;
};

#endif // SFVEHICLE_INFO_HPP