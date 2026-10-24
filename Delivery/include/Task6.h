#ifndef TASK6_H
#define TASK6_H

#include <string>
#include <vector>
#include "Graph.h"
#include "Parcel.h"
#include "Vehicle.h"
#include "Trip.h"

namespace Task6 {

enum class Strategy {
    BASELINE_NEAREST_FIRST,
    BASELINE_EDD,
    IMPROVED_SCORE
};

struct ScheduleResult {
    std::vector<Trip> trips;
    double totalDissatisfaction = 0.0;
    double totalCost = 0.0;
    int overtimeCount = 0;
};

// 策略对比接口（直接使用给定的包裹和车辆数据）
void runT3Comparison(Graph& graph, const std::vector<Parcel>& parcels, const Vehicle& vehicle);
void runT2Comparison(Graph& graph, const std::vector<Parcel>& parcels, const Vehicle& vehicle);

} // namespace Task6

#endif