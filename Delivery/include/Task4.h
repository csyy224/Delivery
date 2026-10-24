#ifndef TASK4_H
#define TASK4_H

#include "Graph.h"
#include <vector>

namespace Task4 {
    /**
     * 执行 T4 任务：退货回收
     * @param graph  地图（内部会多次调用 Dijkstra 构造距离矩阵）
     * @param returnNodes  需要取回的退货地点编号列表（不包含驿站 0）
     */
    void run(Graph& graph, const std::vector<int>& returnNodes);
}

#endif // TASK4_H