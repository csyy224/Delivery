#include "DeliveryScheduler.h"
#include <algorithm>
#include <numeric>
#include <limits>
#include <unordered_set>
#include <map>
#include <iostream>
#include <cmath>

// 工具函数：从包裹列表中提取唯一目的地集合，并返回每个目的地对应的包裹列表（用于后续时间计算）
static std::vector<int> extractUniqueDests(const std::vector<Parcel>& parcels,
                                           std::map<int, std::vector<Parcel>>& destToParcels) {
    std::vector<int> dests;
    std::unordered_set<int> seen;
    for (const auto& p : parcels) {
        if (seen.find(p.dest) == seen.end()) {
            seen.insert(p.dest);
            dests.push_back(p.dest);
        }
        destToParcels[p.dest].push_back(p);
    }
    return dests;
}

// 计算给定访问顺序下，从 0 出发依次访问 dests 再返回 0 的总行驶距离
static double computeRouteDistance(const std::vector<int>& destOrder,
                                   const std::vector<std::vector<double>>& dist) {
    if (destOrder.empty()) return 0.0;
    double total = dist[0][destOrder[0]];
    for (size_t i = 1; i < destOrder.size(); ++i) {
        total += dist[destOrder[i-1]][destOrder[i]];
    }
    total += dist[destOrder.back()][0];
    return total;
}

// 根据访问顺序计算每个目的地的到达时间（出发时间为 depTime，速度 v）
static std::vector<double> computeArrivalTimes(const std::vector<int>& destOrder,
                                               double depTime,
                                               double speed,
                                               const std::vector<std::vector<double>>& dist) {
    std::vector<double> arrivals(destOrder.size(), 0.0);
    double curTime = depTime;
    if (!destOrder.empty()) {
        curTime += dist[0][destOrder[0]] / speed;
        arrivals[0] = curTime;
        for (size_t i = 1; i < destOrder.size(); ++i) {
            curTime += dist[destOrder[i-1]][destOrder[i]] / speed;
            arrivals[i] = curTime;
        }
    }
    return arrivals;
}

// ---------------------- Floyd‑Warshall 距离矩阵 -----------------------
std::vector<std::vector<double>> DeliveryScheduler::buildDistMatrix(const Graph& graph) {
    int n = graph.getVertexCount();
    const double INF = std::numeric_limits<double>::max();
    std::vector<std::vector<double>> dist(n, std::vector<double>(n, INF));
    
    // 对角线为 0
    for (int i = 0; i < n; ++i) dist[i][i] = 0.0;
    
    // 从邻接表读入边权重（假设 Graph 提供 getAdjList()）
    const auto& adj = graph.getAdjList();
    for (int u = 0; u < n; ++u) {
        for (const auto& e : adj[u]) {
            int v = e.v;          // e 中包含 u, v, weight
            dist[u][v] = std::min(dist[u][v], e.weight); // 防重复边
        }
    }
    
    // Floyd-Warshall
    for (int k = 0; k < n; ++k) {
        for (int i = 0; i < n; ++i) {
            if (dist[i][k] == INF) continue;
            for (int j = 0; j < n; ++j) {
                if (dist[k][j] == INF) continue;
                double newDist = dist[i][k] + dist[k][j];
                if (newDist < dist[i][j]) {
                    dist[i][j] = newDist;
                }
            }
        }
    }
    return dist;
}

// ---------------------- T2 批次路径规划 --------------------------------
Trip DeliveryScheduler::planBatchT2(const std::vector<Parcel>& batch,
                                    const std::vector<std::vector<double>>& dist,
                                    double departTime,
                                    const Vehicle& vehicle) {
    Trip trip;
    trip.parcels = batch;
    trip.departTime = departTime;
    trip.totalCost = 0.0;      // T2 不关心成本
    trip.overtimeCount = 0;    // T2 不统计超时

    // 提取唯一目的地及其包裹
    std::map<int, std::vector<Parcel>> destToParcels;
    std::vector<int> uniqueDests = extractUniqueDests(batch, destToParcels);
    
    if (uniqueDests.empty()) {
        trip.route = {0, 0};
        trip.returnTime = departTime;
        return trip;
    }

    // 如果目的地数量较少（≤10），全排列求最小不满意度
    if (uniqueDests.size() <= 10) {
        std::vector<int> bestOrder;
        double bestDissat = std::numeric_limits<double>::max();
        std::vector<int> perm = uniqueDests;
        std::sort(perm.begin(), perm.end());  // next_permutation 需要初始有序
        
        do {
            // 计算各目的地到达时间
            std::vector<double> arrivals = computeArrivalTimes(perm, departTime, vehicle.speed, dist);
            // 计算该顺序下的不满意度之和
            double dissat = 0.0;
            for (size_t i = 0; i < perm.size(); ++i) {
                int dest = perm[i];
                double arrival = arrivals[i];
                for (const auto& p : destToParcels[dest]) {
                    dissat += (arrival - p.S);  // 题目保证 S <= arrival
                }
            }
            if (dissat < bestDissat) {
                bestDissat = dissat;
                bestOrder = perm;
            }
        } while (std::next_permutation(perm.begin(), perm.end()));
        
        // 构建路径
        trip.route.push_back(0);
        for (int d : bestOrder) trip.route.push_back(d);
        trip.route.push_back(0);
        
        // 计算返回时间
        double totalDist = computeRouteDistance(bestOrder, dist);
        trip.returnTime = departTime + totalDist / vehicle.speed;
    } else {
        // 目的地过多，使用最近邻贪心
        std::vector<int> order;
        int cur = 0;
        std::unordered_set<int> remaining(uniqueDests.begin(), uniqueDests.end());
        while (!remaining.empty()) {
            int nearest = -1;
            double minDist = std::numeric_limits<double>::max();
            for (int d : remaining) {
                if (dist[cur][d] < minDist) {
                    minDist = dist[cur][d];
                    nearest = d;
                }
            }
            order.push_back(nearest);
            remaining.erase(nearest);
            cur = nearest;
        }
        // 按该顺序计算返回时间等
        trip.route.push_back(0);
        for (int d : order) trip.route.push_back(d);
        trip.route.push_back(0);
        double totalDist = computeRouteDistance(order, dist);
        trip.returnTime = departTime + totalDist / vehicle.speed;
        // 不满意度由后续调用者统一计算，这里不单独记录
    }
    return trip;
}

// ---------------------- T2 整体规划 --------------------------------
DeliveryScheduler::T2Result DeliveryScheduler::planT2(Graph& graph,
                                                       const std::vector<Parcel>& parcels,
                                                       const Vehicle& vehicle) {
    // 第一步：计算全源最短距离
    auto dist = buildDistMatrix(graph);
    
    // 第二步：按包裹的到站时间 S 升序排序（用于分批）
    std::vector<Parcel> sortedParcels = parcels;
    std::sort(sortedParcels.begin(), sortedParcels.end(),
              [](const Parcel& a, const Parcel& b) { return a.S < b.S; });
    
    // 第三步：贪心分批
    std::vector<std::vector<Parcel>> batches;
    std::vector<Parcel> currentBatch;
    double currentWeight = 0.0;
    double currentMaxS = 0.0;
    for (const auto& p : sortedParcels) {
        if (currentWeight + p.weight > vehicle.capacity + 1e-9) {
            // 当前批次已满，保存
            if (!currentBatch.empty()) {
                batches.push_back(currentBatch);
                currentBatch.clear();
                currentWeight = 0.0;
                currentMaxS = 0.0;
            }
        }
        currentBatch.push_back(p);
        currentWeight += p.weight;
        currentMaxS = std::max(currentMaxS, p.S);
    }
    if (!currentBatch.empty()) {
        batches.push_back(currentBatch);
    }
    
    // 第四步：顺序执行各批次，记录行程和总不满意度
    T2Result result;
    result.totalDissatisfaction = 0.0;
    double currentTime = 0.0;   // 小车可用时刻（刚返回驿站的时间）
    
    for (const auto& batch : batches) {
        // 出发时间不得早于本批包裹中最大的 S，也不得早于当前可用时间
        double maxS = 0.0;
        for (const auto& p : batch) maxS = std::max(maxS, p.S);
        double depart = std::max(currentTime, maxS);
        
        Trip trip = planBatchT2(batch, dist, depart, vehicle);
        
        // 计算本批各包裹的不满意度并累加（若 planBatchT2 未直接提供，这里补算）
        // planBatchT2 可能未记录每包裹送达时间，我们重新根据 trip 计算。
        // 为简单，我们从 trip 的路径和出发时间重新计算送达时间。
        // 注意：如果 planBatchT2 内部使用了最优（或贪心）路径，这里应基于该路径计算。
        // 所以 trip 中已经包含了 route，我们直接复用。
        // 本函要求 planBatchT2 返回的 trip 正确设置了 departTime 和 returnTime，路径 route。
        
        // 根据 route 重新计算每个目的地的到达时间（route[0]=0, route[1..k] 目的地, route[k+1]=0）
        // 我们可以提取除首尾 0 外的节点序列，然后计算到达时间。
        double dep = trip.departTime;
        double speed = vehicle.speed;
        // 构建目的地顺序（不含起止 0）
        std::vector<int> destOrder;
        for (size_t i = 1; i < trip.route.size() - 1; ++i) {
            destOrder.push_back(trip.route[i]);
        }
        std::vector<double> arrivals = computeArrivalTimes(destOrder, dep, speed, dist);
        // 匹配到批次中的每个包裹（同一个目的地可能有多个包裹）
        std::map<int, std::vector<Parcel>> destToParcels;
        extractUniqueDests(batch, destToParcels);
        for (size_t i = 0; i < destOrder.size(); ++i) {
            int dest = destOrder[i];
            double arr = arrivals[i];
            for (const auto& p : destToParcels[dest]) {
                result.totalDissatisfaction += (arr - p.S);
            }
        }
        
        result.trips.push_back(trip);
        currentTime = trip.returnTime;
    }
    return result;
}

// ---------------------- T3 批次路径规划 --------------------------------
Trip DeliveryScheduler::planBatchT3(const std::vector<Parcel>& batch,
                                    const std::vector<std::vector<double>>& dist,
                                    double departTime,
                                    const Vehicle& vehicle) {
    Trip trip;
    trip.parcels = batch;
    trip.departTime = departTime;
    trip.overtimeCount = 0;
    trip.totalCost = 0.0;

    // 提取唯一目的地及其包裹总重量
    std::map<int, std::vector<Parcel>> destToParcels;
    std::vector<int> uniqueDests = extractUniqueDests(batch, destToParcels);
    if (uniqueDests.empty()) {
        trip.route = {0, 0};
        trip.returnTime = departTime;
        return trip;
    }

    // 计算每个目的地的总重量
    std::map<int, double> destWeight;
    for (const auto& p : batch) destWeight[p.dest] += p.weight;

    // 贪心构造访问顺序：从 0 开始，每次选择下一个节点，使 (重量 / 距离) 最大
    std::vector<int> order;
    int cur = 0;
    double curLoad = 0.0;  // 车上当前总重（初始为全批总重）
    for (const auto& p : batch) curLoad += p.weight;
    double totalBatchWeight = curLoad;
    
    std::unordered_set<int> remaining(uniqueDests.begin(), uniqueDests.end());
    while (!remaining.empty()) {
        int bestDest = -1;
        double bestScore = -1.0;
        for (int d : remaining) {
            if (dist[cur][d] < 1e-9) continue; // 避免除零
            double score = destWeight[d] / dist[cur][d];
            if (score > bestScore) {
                bestScore = score;
                bestDest = d;
            }
        }
        // 若所有距离为0或没找到，选最近的一个
        if (bestDest == -1) {
            double minDist = std::numeric_limits<double>::max();
            for (int d : remaining) {
                if (dist[cur][d] < minDist) {
                    minDist = dist[cur][d];
                    bestDest = d;
                }
            }
        }
        order.push_back(bestDest);
        remaining.erase(bestDest);
        cur = bestDest;
    }

    // 根据 order 计算行驶成本、送达时间
    trip.route.push_back(0);
    for (int d : order) trip.route.push_back(d);
    trip.route.push_back(0);

    double currentTime = departTime;
    curLoad = totalBatchWeight;
    int lastNode = 0;
    for (size_t i = 0; i < order.size(); ++i) {
        int dest = order[i];
        double segDist = dist[lastNode][dest];
        // 该段成本
        trip.totalCost += segDist * (vehicle.car_weight + curLoad);
        // 行驶时间
        double travelTime = segDist / vehicle.speed;
        currentTime += travelTime;
        // 卸货
        curLoad -= destWeight[dest];
        lastNode = dest;

        // 记录送达时间，检查超时
        for (const auto& p : destToParcels[dest]) {
            if (currentTime > p.T + 1e-9) {
                trip.overtimeCount++;
            }
        }
    }
    // 返回驿站段
    double backDist = dist[lastNode][0];
    trip.totalCost += backDist * (vehicle.car_weight + curLoad); // curLoad 此时应为 0
    double backTime = backDist / vehicle.speed;
    currentTime += backTime;
    trip.returnTime = currentTime;

    return trip;
}

// ---------------------- T3 整体规划 --------------------------------
DeliveryScheduler::T3Result DeliveryScheduler::planT3(Graph& graph,
                                                       const std::vector<Parcel>& parcels,
                                                       const Vehicle& vehicle) {
    auto dist = buildDistMatrix(graph);

    // 按 deadline 升序排序（让紧急的先送，减少超时）
    std::vector<Parcel> sortedParcels = parcels;
    std::sort(sortedParcels.begin(), sortedParcels.end(),
              [](const Parcel& a, const Parcel& b) { return a.T < b.T; });

    // 贪心分批
    std::vector<std::vector<Parcel>> batches;
    std::vector<Parcel> currentBatch;
    double currentWeight = 0.0;
    for (const auto& p : sortedParcels) {
        if (currentWeight + p.weight > vehicle.capacity + 1e-9) {
            if (!currentBatch.empty()) {
                batches.push_back(currentBatch);
                currentBatch.clear();
                currentWeight = 0.0;
            }
        }
        currentBatch.push_back(p);
        currentWeight += p.weight;
    }
    if (!currentBatch.empty()) {
        batches.push_back(currentBatch);
    }

    // 顺序执行
    T3Result result;
    result.totalCost = 0.0;
    result.overtimeCount = 0;
    double currentTime = 0.0;

    for (const auto& batch : batches) {
        Trip trip = planBatchT3(batch, dist, currentTime, vehicle);
        result.totalCost += trip.totalCost;
        result.overtimeCount += trip.overtimeCount;
        result.trips.push_back(trip);
        currentTime = trip.returnTime; // 下一趟最早出发时间
    }

    return result;
}

DeliveryScheduler::T3Result DeliveryScheduler::planT3ForSingleCar(
        Graph& graph, const std::vector<Parcel>& parcels, const Vehicle& vehicle) {
    auto dist = buildDistMatrix(graph);

    // 按 deadline 升序排序（减少超时）
    std::vector<Parcel> sortedParcels = parcels;
    std::sort(sortedParcels.begin(), sortedParcels.end(),
              [](const Parcel& a, const Parcel& b) { return a.T < b.T; });

    // 贪心分批（与 planT3 完全一致）
    std::vector<std::vector<Parcel>> batches;
    std::vector<Parcel> currentBatch;
    double currentWeight = 0.0;
    for (const auto& p : sortedParcels) {
        if (currentWeight + p.weight > vehicle.capacity + 1e-9) {
            if (!currentBatch.empty()) {
                batches.push_back(currentBatch);
                currentBatch.clear();
                currentWeight = 0.0;
            }
        }
        currentBatch.push_back(p);
        currentWeight += p.weight;
    }
    if (!currentBatch.empty()) batches.push_back(currentBatch);

    T3Result result;
    result.totalCost = 0.0;
    result.overtimeCount = 0;
    double currentTime = 0.0;
    for (const auto& batch : batches) {
        Trip trip = planBatchT3(batch, dist, currentTime, vehicle);
        result.totalCost += trip.totalCost;
        result.overtimeCount += trip.overtimeCount;
        result.trips.push_back(trip);
        currentTime = trip.returnTime;
    }
    return result;
}