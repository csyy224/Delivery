#ifndef TRIP_H
#define TRIP_H

#include <vector>
#include "Parcel.h"
#include "Vehicle.h"

struct Trip 
{
    std::vector<Parcel> parcels;
    std::vector<int> route;
    double departTime,returnTime,totalCost;
    int overtimeCount;
};

#endif