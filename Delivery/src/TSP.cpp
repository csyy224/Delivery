#include "TSP.h"
#include <limits>
#include <algorithm>
#include <unordered_set>

namespace TSP {

const double INF = std::numeric_limits<double>::max();

// ---------- 内部辅助函数 ----------

// 状态压缩 DP 精确求解（节点数 ≤ 20）
static std::pair<double, std::vector<int>> tspExact(const std::vector<std::vector<double>>& dist) {
    int n = static_cast<int>(dist.size());
    if (n == 1) return {0.0, {}};  // 只有驿站

    // dp[mask][i]：已访问节点集合 mask，当前在节点 i 的最短距离
    std::vector<std::vector<double>> dp(1 << n, std::vector<double>(n, INF));
    std::vector<std::vector<int>> parent(1 << n, std::vector<int>(n, -1));
    dp[1][0] = 0.0;  // 从 0 出发（位 0 为 1 表示访问过起点）

    for (int mask = 1; mask < (1 << n); ++mask) {
        for (int i = 0; i < n; ++i) {
            if (!(mask & (1 << i))) continue;
            if (dp[mask][i] >= INF) continue;
            for (int j = 0; j < n; ++j) {
                if (mask & (1 << j)) continue;         // 已经访问过
                if (dist[i][j] >= INF) continue;       // 不可达
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
    if (!order.empty() && order[0] == 0)
        order.erase(order.begin());

    return {bestDist, order};
}

// 2-opt 改进（作用于访问序列，不含起点 0，但计算中考虑与 0 的连接）
static void twoOpt(const std::vector<std::vector<double>>& dist, std::vector<int>& order) {
    int n = static_cast<int>(order.size());
    if (n < 2) return;

    // 构造包含起点 0 的完整路径，方便处理
    std::vector<int> path;
    path.push_back(0);
    path.insert(path.end(), order.begin(), order.end());
    path.push_back(0);

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

// 最近邻构造初始解（从 0 出发，贪心选择最近的未访问节点）
static std::vector<int> nearestNeighbor(const std::vector<std::vector<double>>& dist, int n) {
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

// ---------- 公共接口 ----------

std::pair<double, std::vector<int>> solveTSP(const std::vector<std::vector<double>>& dist) {
    int n = static_cast<int>(dist.size());
    if (n <= 20) {
        // 精确算法（状态压缩 DP）
        return tspExact(dist);
    } else {
        // 启发式：最近邻 + 2-opt 改进
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

} // namespace TSP