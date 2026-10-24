#include "Task4.h"
#include <iostream>
#include <iomanip>
#include <limits>
#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace {

const double INF = std::numeric_limits<double>::max();

// 基于多次 Dijkstra 构建指定节点集合之间的距离矩阵
// 返回的矩阵索引对应 nodes 的顺序，nodes[0] 固定为 0（驿站）
std::vector<std::vector<double>> buildDistMatrix(Graph& graph, const std::vector<int>& nodes) {
    int n = static_cast<int>(nodes.size());
    std::vector<std::vector<double>> dist(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i) {
        graph.dijkstra(nodes[i]);
        for (int j = 0; j < n; ++j) {
            dist[i][j] = graph.getDistance(nodes[i], nodes[j]);
        }
    }
    return dist;
}

// 状态压缩 DP 精确求解 TSP（节点数 ≤ 20）
// 返回最短路径的长度以及访问顺序（不含终点 0，因为终点自动回到 0）
std::pair<double, std::vector<int>> tspExact(const std::vector<std::vector<double>>& dist) {
    int n = static_cast<int>(dist.size());
    if (n == 1) return {0.0, {}};  // 只有驿站
    // dp[mask][i]：已访问节点集合 mask，当前在节点 i 的最短距离
    std::vector<std::vector<double>> dp(1 << n, std::vector<double>(n, INF));
    std::vector<std::vector<int>> parent(1 << n, std::vector<int>(n, -1));
    dp[1][0] = 0.0;  // 从 0 出发

    for (int mask = 1; mask < (1 << n); ++mask) {
        for (int i = 0; i < n; ++i) {
            if (!(mask & (1 << i))) continue;
            if (dp[mask][i] >= INF) continue;
            for (int j = 0; j < n; ++j) {
                if (mask & (1 << j)) continue;
                double newDist = dp[mask][i] + dist[i][j];
                if (newDist < dp[mask | (1 << j)][j]) {
                    dp[mask | (1 << j)][j] = newDist;
                    parent[mask | (1 << j)][j] = i;
                }
            }
        }
    }

    // 找最短回路（最后回到 0）
    int fullMask = (1 << n) - 1;
    double bestDist = INF;
    int lastNode = -1;
    for (int i = 0; i < n; ++i) {
        if (dp[fullMask][i] >= INF) continue;
        double total = dp[fullMask][i] + dist[i][0];
        if (total < bestDist) {
            bestDist = total;
            lastNode = i;
        }
    }

    // 回溯路径
    std::vector<int> order;
    int mask = fullMask;
    int cur = lastNode;
    while (cur != -1) {
        order.push_back(cur);
        int prev = parent[mask][cur];
        mask ^= (1 << cur);
        cur = prev;
    }
    std::reverse(order.begin(), order.end());  // 现在以 0 开头，以 lastNode 结尾
    // 去掉开头的 0（行程中会显式加上）
    if (!order.empty() && order[0] == 0) order.erase(order.begin());

    return {bestDist, order};
}

// 2-opt 改进（作用于访问序列，不含起点 0，但计算中考虑与 0 的连接）
void twoOpt(const std::vector<std::vector<double>>& dist, std::vector<int>& order) {
    int n = static_cast<int>(order.size());
    if (n < 2) return;
    // 为了计算方便，构造扩展数组包含起点 0（虚拟）
    std::vector<int> path;
    path.push_back(0);  // 驿站
    path.insert(path.end(), order.begin(), order.end());
    path.push_back(0);  // 回到驿站

    bool improved = true;
    while (improved) {
        improved = false;
        for (int i = 1; i <= n - 1; ++i) {
            for (int j = i + 1; j <= n; ++j) {
                double oldCost = dist[path[i-1]][path[i]] + dist[path[j]][path[j+1]];
                double newCost = dist[path[i-1]][path[j]] + dist[path[i]][path[j+1]];
                if (newCost + 1e-9 < oldCost) {
                    std::reverse(path.begin() + i, path.begin() + j + 1);
                    improved = true;
                }
            }
        }
    }
    // 更新 order
    order.assign(path.begin() + 1, path.end() - 1);
}

// 最近邻构造初始解
std::vector<int> nearestNeighbor(const std::vector<std::vector<double>>& dist, int n) {
    std::vector<int> order;
    std::unordered_set<int> unvisited;
    for (int i = 1; i < n; ++i) unvisited.insert(i);  // 跳过 0
    int cur = 0;
    while (!unvisited.empty()) {
        int next = -1;
        double minDist = INF;
        for (int v : unvisited) {
            if (dist[cur][v] < minDist) {
                minDist = dist[cur][v];
                next = v;
            }
        }
        order.push_back(next);
        unvisited.erase(next);
        cur = next;
    }
    return order;
}

// 求解 TSP：自动选择精确算法或启发式
std::pair<double, std::vector<int>> solveTSP(const std::vector<std::vector<double>>& dist) {
    int n = static_cast<int>(dist.size());
    if (n <= 20) {
        return tspExact(dist);
    } else {
        // 启发式：最近邻 + 2-opt
        std::vector<int> order = nearestNeighbor(dist, n);
        twoOpt(dist, order);
        // 计算总距离
        double total = dist[0][order[0]];
        for (size_t i = 1; i < order.size(); ++i)
            total += dist[order[i-1]][order[i]];
        total += dist[order.back()][0];
        return {total, order};
    }
}

} // anonymous namespace

// ------------------------------------------------
//  Task4 对外接口
// ------------------------------------------------
void Task4::run(Graph& graph, const std::vector<int>& returnNodes) {
    std::cout << "==================== Task 4: Return Pickup ====================\n";

    // 构造完整节点集：0 号驿站 + 所有退货点
    std::vector<int> nodes = {0};
    nodes.insert(nodes.end(), returnNodes.begin(), returnNodes.end());

    // 计算距离矩阵（索引与 nodes 对应）
    auto dist = buildDistMatrix(graph, nodes);

    // 求解 TSP
    auto [totalDist, order] = solveTSP(dist);

    // order 是访问的退货点顺序（不含 0），需要映射回原始节点编号
    // 映射到原始节点ID：order[i] 是 nodes 的索引，实际节点为 nodes[order[i]]
    std::vector<int> pickupOrder;
    for (int idx : order) pickupOrder.push_back(nodes[idx]);

    // 计算总耗时（距离 / 速度）
    // 速度从 Vehicle 获取？这里我们仅输出总距离，用户可根据小车的速度自行换算
    // 为保持通用，这里只输出距离；若要输出时间，需要传入 Vehicle 参数。
    // 我们假定调用者可以自行换算。但为了报告清晰，我们打印总距离。

    std::cout << "Pickup order (node IDs): ";
    for (size_t i = 0; i < pickupOrder.size(); ++i) {
        std::cout << pickupOrder[i];
        if (i != pickupOrder.size() - 1) std::cout << " -> ";
    }
    std::cout << "\nTotal travel distance: " << std::fixed << std::setprecision(2) << totalDist << "\n";
    std::cout << "================================================================\n";
}