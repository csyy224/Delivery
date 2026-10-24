#include "Task2.h"
#include "DeliveryScheduler.h"
#include "Visualizer.h"         
#include <iostream>
#include <iomanip>

void Task2::run(Graph& graph, const std::vector<Parcel>& parcels, const Vehicle& vehicle) {
    Visualizer::printTitle("Task 2: Minimize Dissatisfaction");

    // 先展示地图
    Visualizer::printMap(graph);

    // 调用调度器
    auto result = DeliveryScheduler::planT2(graph, parcels, vehicle);

    // 逐趟展示
    for (size_t t = 0; t < result.trips.size(); ++t) {
        Visualizer::printTrip(result.trips[t], static_cast<int>(t + 1));
    }

    // 汇总表格
    Visualizer::printT2Summary(result.trips, result.totalDissatisfaction);
}