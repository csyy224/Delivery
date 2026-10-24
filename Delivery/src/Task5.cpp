#include "Task5.h"
#include "DeliveryScheduler.h"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <limits>

namespace {

// 将包裹按重量降序排序（辅助分配函数）
bool cmpByWeight(const Parcel& a, const Parcel& b) {
    return a.weight > b.weight;
}

/**
 * 简单的贪心分配策略：
 * 1. 将包裹按目的地到驿站的距离降序排序，优先处理距离远的（期待两车远近距离均衡）
 * 2. 依次将包裹分配给当前累计重量较小的车辆
 */
std::pair<std::vector<Parcel>, std::vector<Parcel>> assignParcels(
        Graph& graph, const std::vector<Parcel>& parcels) {
    // 计算每个包裹目的地到 0 的最短距离
    std::vector<std::pair<Parcel, double>> parcelDist;
    for (const auto& p : parcels) {
        graph.dijkstra(0);                     // 以 0 为源
        double d = graph.getDistance(0, p.dest);
        parcelDist.push_back({p, d});
    }

    // 按距离降序排序
    std::sort(parcelDist.begin(), parcelDist.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });

    std::vector<Parcel> car1, car2;
    double weight1 = 0.0, weight2 = 0.0;

    for (const auto& [p, dist] : parcelDist) {
        // 若总重量已超过车辆容量？实际上两车可以多趟，但这里只是分配初始负载，
        // 我们只做重量均衡，不限制单次装载（后续单车规划会自动分批）
        if (weight1 <= weight2) {
            car1.push_back(p);
            weight1 += p.weight;
        } else {
            car2.push_back(p);
            weight2 += p.weight;
        }
    }
    return {car1, car2};
}

} // anonymous namespace

void Task5::run(Graph& graph, const std::vector<Parcel>& parcels, const Vehicle& vehicle) {
    std::cout << "==================== Task 5: Dual-Vehicle Coordination ====================\n";

    // 1. 将包裹分配给两辆车
    auto [parcelsCar1, parcelsCar2] = assignParcels(graph, parcels);
    std::cout << "Car 1 gets " << parcelsCar1.size() << " parcels, "
              << "total weight: " << std::fixed << std::setprecision(2)
              << std::accumulate(parcelsCar1.begin(), parcelsCar1.end(), 0.0,
                                 [](double sum, const Parcel& p) { return sum + p.weight; })
              << "\n";
    std::cout << "Car 2 gets " << parcelsCar2.size() << " parcels, "
              << "total weight: " << std::fixed << std::setprecision(2)
              << std::accumulate(parcelsCar2.begin(), parcelsCar2.end(), 0.0,
                                 [](double sum, const Parcel& p) { return sum + p.weight; })
              << "\n\n";

    // 2. 分别规划单车配送（复用 T3 的单车调度逻辑）
    DeliveryScheduler::T3Result res1, res2;
    if (!parcelsCar1.empty()) {
        res1 = DeliveryScheduler::planT3ForSingleCar(graph, parcelsCar1, vehicle);
    }
    if (!parcelsCar2.empty()) {
        res2 = DeliveryScheduler::planT3ForSingleCar(graph, parcelsCar2, vehicle);
    }

    // 3. 输出 Car 1 的结果
    if (!parcelsCar1.empty()) {
        std::cout << "--- Car 1 Trips ---\n";
        for (size_t t = 0; t < res1.trips.size(); ++t) {
            const Trip& trip = res1.trips[t];
            std::cout << "  Trip " << (t+1) << ": ";
            for (size_t i = 0; i < trip.parcels.size(); ++i) {
                std::cout << trip.parcels[i].id << " ";
            }
            std::cout << "| Cost: " << std::fixed << std::setprecision(2) << trip.totalCost;
            std::cout << " | Overtime: " << trip.overtimeCount << "\n";
        }
    }

    // 4. 输出 Car 2 的结果
    if (!parcelsCar2.empty()) {
        std::cout << "\n--- Car 2 Trips ---\n";
        for (size_t t = 0; t < res2.trips.size(); ++t) {
            const Trip& trip = res2.trips[t];
            std::cout << "  Trip " << (t+1) << ": ";
            for (size_t i = 0; i < trip.parcels.size(); ++i) {
                std::cout << trip.parcels[i].id << " ";
            }
            std::cout << "| Cost: " << std::fixed << std::setprecision(2) << trip.totalCost;
            std::cout << " | Overtime: " << trip.overtimeCount << "\n";
        }
    }

    // 5. 汇总
    double totalCost = res1.totalCost + res2.totalCost;
    int totalOvertime = res1.overtimeCount + res2.overtimeCount;
    std::cout << "\nTotal cost (both cars): " << std::fixed << std::setprecision(2) << totalCost << "\n";
    std::cout << "Total overtime parcels: " << totalOvertime << "\n";
    std::cout << "============================================================================\n";
}