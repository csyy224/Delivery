#ifndef TASK2_H
#define TASK2_H

#include "Graph.h"
#include "Parcel.h"
#include "Vehicle.h"
#include <vector>

namespace Task2 {
    /**
     * 执行 T2 任务：最小化不满意度之和
     * 将配送方案打印到标准输出，并返回不满意度总和
     */
    void run(Graph& graph, const std::vector<Parcel>& parcels, const Vehicle& vehicle);
}

#endif // TASK2_H