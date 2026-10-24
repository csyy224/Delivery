#include "Task6.h"
#include "DeliveryScheduler.h"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <limits>
#include <map>
#include <unordered_set>
#include <cmath>

// 调试开关：运行时如果还有 0，打开下面的 #define 查看中间变量
// #define TASK6_DEBUG

namespace {

const double INF = std::numeric_limits<double>::max();
const double EPS = 1e-9;


// ---------- 提取唯一目的地 ----------
std::vector<int> extractUniqueDests(const std::vector<Parcel>& parcels,
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

// ---------- 计算给定顺序的行驶距离 ----------
double computeRouteDist(const std::vector<int>& order,
                        const std::vector<std::vector<double>>& dist) {
    if (order.empty()) return 0.0;
    double total = dist[0][order[0]];
    for (size_t i = 1; i < order.size(); ++i)
        total += dist[order[i-1]][order[i]];
    total += dist[order.back()][0];
    return total;
}

// ---------- 计算到达时间 ----------
std::vector<double> computeArrivalTimes(const std::vector<int>& order,
                                        double depTime, double speed,
                                        const std::vector<std::vector<double>>& dist) {
    std::vector<double> arrivals(order.size(), 0.0);
    if (order.empty()) return arrivals;
    double t = depTime + dist[0][order[0]] / speed;
    arrivals[0] = t;
    for (size_t i = 1; i < order.size(); ++i) {
        t += dist[order[i-1]][order[i]] / speed;
        arrivals[i] = t;
    }
    return arrivals;
}

void twoOpt(std::vector<int>& order, const std::vector<std::vector<double>>& dist) {
    if (order.size() < 2) return;
    std::vector<int> path = {0};
    path.insert(path.end(), order.begin(), order.end());
    path.push_back(0);
    bool improved = true;
    while (improved) {
        improved = false;
        for (int i = 1; i <= (int)order.size() - 1; ++i) {
            for (int j = i + 1; j <= (int)order.size(); ++j) {
                double oldCost = dist[path[i-1]][path[i]] + dist[path[j]][path[j+1]];
                double newCost = dist[path[i-1]][path[j]] + dist[path[i]][path[j+1]];
                if (newCost + 1e-9 < oldCost) {
                    std::reverse(path.begin() + i, path.begin() + j + 1);
                    improved = true;
                }
            }
        }
    }
    order.assign(path.begin() + 1, path.end() - 1);
}

// ============ T3 批次路径规划（可配置策略） ============
// 参数 destToParcels 是从外部传入的、当前批次的目的地→包裹列表映射
// 本函数不再重复调用 extractUniqueDests，也不用 const_cast
std::vector<int> planBatchPathT3(const std::vector<Parcel>& batch,
                                 const std::vector<std::vector<double>>& dist,
                                 Task6::Strategy strategy,
                                 const std::map<int, std::vector<Parcel>>& destToParcels) {
    // 从 destToParcels 直接提取唯一目的地列表
    std::vector<int> uniqueDests;
    for (const auto& kv : destToParcels) {
        uniqueDests.push_back(kv.first);
    }
    if (uniqueDests.empty()) return {};

    std::vector<int> order;

    if (strategy == Task6::Strategy::BASELINE_NEAREST_FIRST) {
        // 最近邻贪心
        int cur = 0;
        std::unordered_set<int> remaining(uniqueDests.begin(), uniqueDests.end());
        while (!remaining.empty()) {
            int nearest = -1;
            double minDist = INF;
            for (int d : remaining) {
                if (dist[cur][d] < minDist) {
                    minDist = dist[cur][d];
                    nearest = d;
                }
            }
            if (nearest == -1) break; // 安全保护，但通常不会发生
            order.push_back(nearest);
            remaining.erase(nearest);
            cur = nearest;
        }
    } else if (strategy == Task6::Strategy::BASELINE_EDD) {
        // 按最早截止时间排序
        std::map<int, double> earliestDeadline;
        for (const auto& p : batch) {
            auto it = earliestDeadline.find(p.dest);
            if (it == earliestDeadline.end() || p.T < it->second)
                earliestDeadline[p.dest] = p.T;
        }
        std::sort(uniqueDests.begin(), uniqueDests.end(),
                  [&](int a, int b) { return earliestDeadline[a] < earliestDeadline[b]; });
        order = uniqueDests;
    } else if (strategy == Task6::Strategy::IMPROVED_SCORE) {
        // 综合评分贪心：优先处理重包裹和紧急包裹
        std::map<int, double> destWeight, destDeadline;
        for (const auto& p : batch) {
            destWeight[p.dest] += p.weight;
            auto it = destDeadline.find(p.dest);
            if (it == destDeadline.end() || p.T < it->second)
                destDeadline[p.dest] = p.T;
        }

        int cur = 0;
        std::unordered_set<int> remaining(uniqueDests.begin(), uniqueDests.end());
        while (!remaining.empty()) {
            int best = -1;
            double bestScore = -INF;
            for (int d : remaining) {
                double w = destWeight[d];
                double d_cur = dist[cur][d];
                if (d_cur < EPS) d_cur = EPS;          // 防止除零
                double deadlineFactor = std::max(1.0, destDeadline[d]);
                double score = (w / d_cur) * (1.0 / deadlineFactor);
                if (score > bestScore) {
                    bestScore = score;
                    best = d;
                }
            }
            if (best == -1) break;
            order.push_back(best);
            remaining.erase(best);
            cur = best;
        }

        // 2-opt 局部改进（需提前定义 twoOpt 函数）
        twoOpt(order, dist);
    }

    return order;
}

// ============ 执行一趟 T3 ============
Trip executeBatchT3(const std::vector<Parcel>& batch,
                    const std::vector<int>& order,
                    const std::vector<std::vector<double>>& dist,
                    double departTime,
                    const Vehicle& vehicle,
                    const std::map<int, std::vector<Parcel>>& destToParcels) {
    Trip trip;
    trip.parcels = batch;
    trip.departTime = departTime;
    trip.totalCost = 0.0;
    trip.overtimeCount = 0;

    std::map<int, double> destWeight;
    for (const auto& p : batch) destWeight[p.dest] += p.weight;
    double curLoad = 0.0;
    for (const auto& p : batch) curLoad += p.weight;

    trip.route.push_back(0);
    for (int d : order) trip.route.push_back(d);
    trip.route.push_back(0);

    double currentTime = departTime;
    int lastNode = 0;

    // 调试：输出车辆自重
    std::cout << "  DEBUG: car_weight = " << vehicle.car_weight << std::endl;

    for (size_t i = 0; i < order.size(); ++i) {
        int d = order[i];
        double segDist = dist[lastNode][d];
        double cost = segDist * (vehicle.car_weight + curLoad);

        std::cout << "  From " << lastNode << " to " << d
                  << " dist=" << segDist << " curLoad=" << curLoad
                  << " cost=" << cost << std::endl;

        trip.totalCost += cost;
        currentTime += segDist / vehicle.speed;
        curLoad -= destWeight[d];
        lastNode = d;

        auto it = destToParcels.find(d);
        if (it != destToParcels.end()) {
            for (const auto& p : it->second) {
                if (currentTime > p.T + 1e-9) trip.overtimeCount++;
            }
        }
    }
    // 返回驿站段
    double backDist = dist[lastNode][0];
    double backCost = backDist * (vehicle.car_weight + curLoad);
    std::cout << "  From " << lastNode << " to 0 dist=" << backDist
              << " curLoad=" << curLoad << " backCost=" << backCost << std::endl;

    trip.totalCost += backCost;
    currentTime += backDist / vehicle.speed;
    trip.returnTime = currentTime;

    std::cout << "  TotalCost after accumulation: " << trip.totalCost << std::endl;
    return trip;
}

// ============ T3 整体调度器 ============
Task6::ScheduleResult scheduleT3(Graph& graph,
                                 const std::vector<Parcel>& parcels,
                                 const Vehicle& vehicle,
                                 Task6::Strategy strategy) {
    using namespace Task6;
    ScheduleResult res;
    auto dist = DeliveryScheduler::buildDistMatrix(graph);
    // 根据策略对包裹排序
    std::vector<Parcel> sortedParcels;
    if (strategy == Strategy::BASELINE_NEAREST_FIRST) {
        sortedParcels = parcels;
        std::sort(sortedParcels.begin(), sortedParcels.end(),
                  [](const Parcel& a, const Parcel& b) { return a.weight > b.weight; });
    } else if (strategy == Strategy::BASELINE_EDD) {
        sortedParcels = parcels;
        std::sort(sortedParcels.begin(), sortedParcels.end(),
                  [](const Parcel& a, const Parcel& b) { return a.T < b.T; });
    } else if (strategy == Strategy::IMPROVED_SCORE) {
        sortedParcels = parcels;
        std::sort(sortedParcels.begin(), sortedParcels.end(),
                  [](const Parcel& a, const Parcel& b) {
                      double sa = a.weight / std::max(1.0, a.T);
                      double sb = b.weight / std::max(1.0, b.T);
                      return sa > sb;
                  });
    }

    // 按容量分批
    std::vector<std::vector<Parcel>> batches;
    std::vector<Parcel> curBatch;
    double curWeight = 0.0;
    for (const auto& p : sortedParcels) {
        if (curWeight + p.weight > vehicle.capacity + EPS) {
            if (!curBatch.empty()) {
                batches.push_back(curBatch);
                curBatch.clear();
                curWeight = 0.0;
            }
        }
        curBatch.push_back(p);
        curWeight += p.weight;
    }
    if (!curBatch.empty()) batches.push_back(curBatch);

#ifdef TASK6_DEBUG
    std::cerr << "[scheduleT3] batches.size()=" << batches.size() << "\n";
#endif

    double currentTime = 0.0;
    for (const auto& batch : batches) {
        // 构造当前批次的目的地→包裹映射
        std::map<int, std::vector<Parcel>> destMap;
        extractUniqueDests(batch, destMap);

        // 根据策略生成访问顺序
        std::vector<int> order = planBatchPathT3(batch, dist, strategy, destMap);

        // 执行此趟行程，计算成本和时间
        Trip trip = executeBatchT3(batch, order, dist, currentTime, vehicle, destMap);

        res.trips.push_back(trip);
        res.totalCost += trip.totalCost;
        res.overtimeCount += trip.overtimeCount;
        currentTime = trip.returnTime;
        std::cout << "Batch size: " << batch.size() << std::endl;
        std::cout << "Unique dests in batch: ";
        for (auto& kv : destMap) std::cout << kv.first << " ";
        std::cout << std::endl;
        std::cout << "Order size: " << order.size() << std::endl;
        for (int d : order) std::cout << d << " ";
        std::cout << std::endl;
    }

    return res;
}
// ============ T2 调度器（可配置策略） ============
Task6::ScheduleResult scheduleT2(Graph& graph,
                                 const std::vector<Parcel>& parcels,
                                 const Vehicle& vehicle,
                                 Task6::Strategy strategy) {
    using namespace Task6;
    ScheduleResult res;
    auto dist = DeliveryScheduler::buildDistMatrix(graph);

    // T2 按 S 排序
    std::vector<Parcel> sortedParcels = parcels;
    std::sort(sortedParcels.begin(), sortedParcels.end(),
              [](const Parcel& a, const Parcel& b) { return a.S < b.S; });

    // 分批
    std::vector<std::vector<Parcel>> batches;
    std::vector<Parcel> curBatch;
    double curWeight = 0.0;
    for (const auto& p : sortedParcels) {
        if (curWeight + p.weight > vehicle.capacity + EPS) {
            if (!curBatch.empty()) {
                batches.push_back(curBatch);
                curBatch.clear();
                curWeight = 0.0;
            }
        }
        curBatch.push_back(p);
        curWeight += p.weight;
    }
    if (!curBatch.empty()) batches.push_back(curBatch);

    double currentTime = 0.0;
    for (const auto& batch : batches) {
        double maxS = 0.0;
        for (const auto& p : batch) maxS = std::max(maxS, p.S);
        double depart = std::max(currentTime, maxS);

        std::map<int, std::vector<Parcel>> destMap;
        extractUniqueDests(batch, destMap);

        std::vector<int> order;
        if (strategy == Strategy::BASELINE_NEAREST_FIRST) {
            int cur = 0;
            std::unordered_set<int> remaining;
            for (const auto& p : batch) remaining.insert(p.dest);
            while (!remaining.empty()) {
                int nearest = -1;
                double minDist = INF;
                for (int d : remaining) {
                    if (dist[cur][d] < minDist) { minDist = dist[cur][d]; nearest = d; }
                }
                order.push_back(nearest);
                remaining.erase(nearest);
                cur = nearest;
            }
        } else if (strategy == Strategy::IMPROVED_SCORE) {
            std::map<int, double> destWeight;
            for (const auto& p : batch) destWeight[p.dest] += p.weight;
            int cur = 0;
            std::unordered_set<int> remaining;
            for (const auto& p : batch) remaining.insert(p.dest);
            while (!remaining.empty()) {
                int best = -1;
                double bestScore = -INF;
                for (int d : remaining) {
                    double w = destWeight[d];
                    double d_cur = dist[cur][d];
                    if (d_cur < EPS) d_cur = EPS;
                    double score = w / d_cur;
                    if (score > bestScore) { bestScore = score; best = d; }
                }
                order.push_back(best);
                remaining.erase(best);
                cur = best;
            }
        }

#ifdef TASK6_DEBUG
        std::cerr << "[scheduleT2] order.size()=" << order.size() << "\n";
#endif

        Trip trip;
        trip.parcels = batch;
        trip.departTime = depart;
        trip.totalCost = 0.0;
        trip.overtimeCount = 0;
        trip.route.push_back(0);
        for (int d : order) trip.route.push_back(d);
        trip.route.push_back(0);

        std::vector<double> arrivals = computeArrivalTimes(order, depart, vehicle.speed, dist);
        double batchDissat = 0.0;
        for (size_t i = 0; i < order.size(); ++i) {
            int d = order[i];
            double arr = arrivals[i];
            auto it = destMap.find(d);
            if (it != destMap.end()) {
                for (const auto& p : it->second) batchDissat += (arr - p.S);
            }
        }
        double totalDist = computeRouteDist(order, dist);
        trip.returnTime = depart + totalDist / vehicle.speed;

        res.trips.push_back(trip);
        res.totalDissatisfaction += batchDissat;
        currentTime = trip.returnTime;
    }
    return res;
}

} // anonymous namespace

// ==================== Task6 公共接口 ====================

void Task6::runT3Comparison(Graph& graph, const std::vector<Parcel>& parcels, const Vehicle& vehicle) {
    using namespace Task6;
    std::cout << "\n========== T3 Strategy Comparison ==========\n";
    std::cout << "Strategy        Total Cost      Overtime Parcels\n";
    std::cout << "------------------------------------------------\n";

    auto print = [](const std::string& name, const ScheduleResult& res) {
        std::cout << std::left << std::setw(15) << name
                  << std::setw(12) << std::fixed << std::setprecision(2) << res.totalCost
                  << res.overtimeCount << "\n";
    };

    ScheduleResult resNN = scheduleT3(graph, parcels, vehicle, Strategy::BASELINE_NEAREST_FIRST);
    print("Baseline NN", resNN);

    ScheduleResult resEDD = scheduleT3(graph, parcels, vehicle, Strategy::BASELINE_EDD);
    print("Baseline EDD", resEDD);

    ScheduleResult resImp = scheduleT3(graph, parcels, vehicle, Strategy::IMPROVED_SCORE);
    print("Improved", resImp);

    std::cout << "================================================\n";
}

void Task6::runT2Comparison(Graph& graph, const std::vector<Parcel>& parcels, const Vehicle& vehicle) {
    using namespace Task6;
    std::cout << "\n========== T2 Strategy Comparison ==========\n";
    std::cout << "Strategy        Total Dissatisfaction\n";
    std::cout << "----------------------------------------\n";

    auto print = [](const std::string& name, const ScheduleResult& res) {
        std::cout << std::left << std::setw(15) << name
                  << std::fixed << std::setprecision(2) << res.totalDissatisfaction << "\n";
    };

    ScheduleResult resNN = scheduleT2(graph, parcels, vehicle, Strategy::BASELINE_NEAREST_FIRST);
    print("Baseline NN", resNN);

    ScheduleResult resImp = scheduleT2(graph, parcels, vehicle, Strategy::IMPROVED_SCORE);
    print("Improved", resImp);

    std::cout << "========================================\n";
}