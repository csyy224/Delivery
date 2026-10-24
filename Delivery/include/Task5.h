#ifndef TASK5_H
#define TASK5_H

#include "Graph.h"
#include "Parcel.h"
#include "Vehicle.h"
#include <vector>

namespace Task5 {
    /**
     * 执行 T5 任务：双车协同配送
     * 将包裹分配给两辆车，各自规划路线，输出总成本和超时数
     */
    void run(Graph& graph, const std::vector<Parcel>& parcels, const Vehicle& vehicle);
}

#endif // TASK5_H