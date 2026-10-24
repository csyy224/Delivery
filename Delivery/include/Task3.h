#ifndef TASK3_H
#define TASK3_H

#include "Graph.h"
#include "Parcel.h"
#include "Vehicle.h"
#include <vector>

namespace Task3 {
    /**
     * 执行 T3 任务：带容量的运送成本
     * 输出每趟配送方案、总成本、超时包裹数量
     */
    void run(Graph& graph, const std::vector<Parcel>& parcels, const Vehicle& vehicle);
}

#endif // TASK3_H