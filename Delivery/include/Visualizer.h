#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "Graph.h"
#include "Trip.h"
#include <vector>
#include <string>

namespace Visualizer {

    // 打印地图的整体拓扑结构（邻接表 + 节点坐标）
    void printMap(const Graph& graph);

    // 打印一条路径（节点序列），如 0 → 3 → 1 → 0
    void printRoute(const std::vector<int>& route, const std::string& label = "Route");

    // 打印单趟配送的详细信息
    void printTrip(const Trip& trip, int tripIndex);

    // 打印所有趟次的汇总表格（T2 用）
    void printT2Summary(const std::vector<Trip>& trips, double totalDissatisfaction);

    // 打印所有趟次的汇总表格（T3 用）
    void printT3Summary(const std::vector<Trip>& trips, double totalCost, int totalOvertime);

    // 打印分隔线
    void printSeparator(char ch = '=', int width = 70);

    // 打印标题
    void printTitle(const std::string& title);

} // namespace Visualizer

#endif // VISUALIZER_H