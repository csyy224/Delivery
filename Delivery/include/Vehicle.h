#ifndef VEHICLE_H
#define VEHICLE_H

struct Vehicle
{
	double speed,car_weight,capacity;
	Vehicle() = default;
    Vehicle(double s, double cw, double cap)
        : speed(s), car_weight(cw), capacity(cap) {}
};
#endif