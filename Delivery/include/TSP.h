#ifndef TSP_H
#define TSP_H

#include <vector>
#include <utility>

namespace TSP {

/**
 * 求解旅行商问题（TSP）
 * @param dist 距离矩阵，dist[i][j] 表示节点 i 到节点 j 的距离，
 *             节点集包含驿站（索引 0）和所有需要访问的节点。
 * @return pair<总最短路径长度, 访问顺序（不含起点 0 的节点索引列表）>
 *         例如 order = {2, 1, 3} 表示从 0 出发依次访问节点 2, 1, 3 后返回 0。
 *         自动选择算法：当节点数 ≤ 20 时使用状态压缩 DP 求最优解，否则使用最近邻 + 2-opt 启发式。
 */
std::pair<double, std::vector<int>> solveTSP(const std::vector<std::vector<double>>& dist);

} // namespace TSP

#endif // TSP_H