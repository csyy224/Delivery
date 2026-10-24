#include "Visualizer.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>

namespace Visualizer {

// ============ 打印地图 ============
void printMap(const Graph& graph) {
    printSeparator('=');
    std::cout << "  CAMPUS MAP\n";
    printSeparator('=');

    int n = graph.getVertexCount();
    const auto& adj = graph.getAdjList();

    // 节点信息
    std::cout << "\nNodes (" << n << " total):\n";
    std::cout << std::left << std::setw(6) << "ID"
              << std::setw(10) << "X"
              << std::setw(10) << "Y"
              << "Type\n";
    std::cout << "---------------------------------\n";

    // 由于节点坐标在 Graph 中可能没有公开访问接口，我们通过邻接表间接展示
    // 若 Graph 提供了获取节点信息的接口，可在此调用
    // 作为替代，我们主要展示边的关系
    // 如果你的 Graph 有公开的 nodes 成员或 getter，可以在这里迭代

    // 边信息（邻接表）
    std::cout << "\nEdges (adjacency list):\n";
    std::cout << "---------------------------------\n";
    for (int u = 0; u < n; ++u) {
        std::cout << "Node " << u;
        if (u == 0) std::cout << " [Station]";
        std::cout << " -> ";
        if (adj[u].empty()) {
            std::cout << "(none)";
        } else {
            for (size_t j = 0; j < adj[u].size(); ++j) {
                std::cout << adj[u][j].v << "(" << std::fixed << std::setprecision(1) << adj[u][j].weight << ")";
                if (j != adj[u].size() - 1) std::cout << ", ";
            }
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}

// ============ 打印路径 ============
void printRoute(const std::vector<int>& route, const std::string& label) {
    std::cout << label << ": ";
    for (size_t i = 0; i < route.size(); ++i) {
        std::cout << route[i];
        if (i != route.size() - 1) std::cout << " -> ";
    }
    std::cout << "\n";
}

// ============ 打印单趟行程 ============
void printTrip(const Trip& trip, int tripIndex) {
    std::cout << "\n";
    printSeparator('-', 60);
    std::cout << "  Trip " << tripIndex << "\n";
    printSeparator('-', 60);

    std::cout << "  Departure time : " << std::fixed << std::setprecision(2) << trip.departTime << "\n";
    std::cout << "  Return time    : " << std::fixed << std::setprecision(2) << trip.returnTime << "\n";

    // 包裹列表
    std::cout << "  Parcels (" << trip.parcels.size() << "): ";
    for (size_t i = 0; i < trip.parcels.size(); ++i) {
        std::cout << "#" << trip.parcels[i].id
                  << "(dest:" << trip.parcels[i].dest
                  << ",w:" << std::fixed << std::setprecision(1) << trip.parcels[i].weight << ")";
        if (i != trip.parcels.size() - 1) std::cout << ", ";
    }
    std::cout << "\n";

    // 路线
    printRoute(trip.route, "  Route");

    // 成本（如果有）
    if (trip.totalCost > 0.0) {
        std::cout << "  Cost           : " << std::fixed << std::setprecision(2) << trip.totalCost << "\n";
    }
    if (trip.overtimeCount > 0) {
        std::cout << "  Overtime count : " << trip.overtimeCount << "\n";
    }
}

// ============ T2 汇总 ============
void printT2Summary(const std::vector<Trip>& trips, double totalDissatisfaction) {
    printSeparator('=');
    std::cout << "  T2 SUMMARY\n";
    printSeparator('=');

    std::cout << std::left << std::setw(8) << "Trip"
              << std::setw(12) << "Depart"
              << std::setw(12) << "Return"
              << std::setw(8) << "Stops"
              << "Route\n";
    std::cout << "----------------------------------------------------------------------\n";

    for (size_t t = 0; t < trips.size(); ++t) {
        const Trip& trip = trips[t];
        std::cout << std::left << std::setw(8) << (t + 1)
                  << std::setw(12) << std::fixed << std::setprecision(2) << trip.departTime
                  << std::setw(12) << trip.returnTime
                  << std::setw(8) << (trip.route.size() - 2);  // 不含首尾 0

        for (size_t i = 0; i < trip.route.size(); ++i) {
            std::cout << trip.route[i];
            if (i != trip.route.size() - 1) std::cout << "->";
        }
        std::cout << "\n";
    }
    std::cout << "----------------------------------------------------------------------\n";
    std::cout << "Total dissatisfaction: " << std::fixed << std::setprecision(2) << totalDissatisfaction << "\n";
    printSeparator('=');
}

// ============ T3 汇总 ============
void printT3Summary(const std::vector<Trip>& trips, double totalCost, int totalOvertime) {
    printSeparator('=');
    std::cout << "  T3 SUMMARY\n";
    printSeparator('=');

    std::cout << std::left << std::setw(8) << "Trip"
              << std::setw(14) << "Cost"
              << std::setw(10) << "Overtime"
              << std::setw(10) << "Stops"
              << "Route\n";
    std::cout << "--------------------------------------------------------------------------\n";

    for (size_t t = 0; t < trips.size(); ++t) {
        const Trip& trip = trips[t];
        std::cout << std::left << std::setw(8) << (t + 1)
                  << std::setw(14) << std::fixed << std::setprecision(2) << trip.totalCost
                  << std::setw(10) << trip.overtimeCount
                  << std::setw(10) << (trip.route.size() - 2);

        for (size_t i = 0; i < trip.route.size(); ++i) {
            std::cout << trip.route[i];
            if (i != trip.route.size() - 1) std::cout << "->";
        }
        std::cout << "\n";
    }
    std::cout << "--------------------------------------------------------------------------\n";
    std::cout << "Total cost      : " << std::fixed << std::setprecision(2) << totalCost << "\n";
    std::cout << "Total overtime  : " << totalOvertime << "\n";
    printSeparator('=');
}

// ============ 辅助函数 ============
void printSeparator(char ch, int width) {
    std::cout << std::string(width, ch) << "\n";
}

void printTitle(const std::string& title) {
    printSeparator('=');
    std::cout << "  " << title << "\n";
    printSeparator('=');
}

} // namespace Visualizer