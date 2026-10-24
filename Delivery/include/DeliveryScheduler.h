/**
 * @file DeliveryScheduler.h
 * @brief 配送调度器头文件
 */

#ifndef DELIVERY_SCHEDULER_H
#define DELIVERY_SCHEDULER_H

#include "Parcel.h"
#include "Vehicle.h"
#include "Trip.h"
#include "Graph.h"
#include <vector>

class DeliveryScheduler {
public:
    // T2 结果
    struct T2Result {
        std::vector<Trip> trips;
        double totalDissatisfaction; // 总不满意度 = sum(送达时间 - S_i)
    };

    // T3 结果
    struct T3Result {
        std::vector<Trip> trips;
        double totalCost;      // 总运送成本
        int overtimeCount;     // 超时包裹数
    };

    /**
     * 规划 T2：最小化不满意度之和
     * @param graph 地图引用（非 const，内部会用于计算距离矩阵，可能修改内部状态，请在调用前保存必要信息）
     * @param parcels 所有包裹列表
     * @param vehicle 小车参数
     * @return T2Result 包含配送方案与总不满意度
     */
    static T2Result planT2(Graph& graph, const std::vector<Parcel>& parcels, const Vehicle& vehicle);

    /**
     * 规划 T3：带容量的运送成本
     * @param graph 地图引用
     * @param parcels 所有包裹（S 视为 0，但保留原始 deadline）
     * @param vehicle 小车参数
     * @return T3Result 包含配送方案、总成本与超时数
     */
    static T3Result planT3(Graph& graph, const std::vector<Parcel>& parcels, const Vehicle& vehicle);
        
    // 为一组包裹规划一趟路径（T3 用）
    // 返回的 Trip 中包含完整路径、成本、超时计数等信息
    static Trip planBatchT3(const std::vector<Parcel>& batch,
                            const std::vector<std::vector<double>>& dist,
                            double departTime,
                            const Vehicle& vehicle);

    static T3Result planT3ForSingleCar(Graph& graph,
                                       const std::vector<Parcel>& parcels,
                                       const Vehicle& vehicle);

	// 构建全源最短距离矩阵（利用 Floyd‑Warshall，需要 Graph 暴露邻接表）								   
	static std::vector<std::vector<double>> buildDistMatrix(const Graph& graph);


private:
    
    
    // 为一组包裹（可能含重复 destinations）规划一趟路径（T2 用）
    // 返回的 Trip 中 route 已经包含 0→…→0，totalCost 字段不使用，不满意度由调用者汇总
    static Trip planBatchT2(const std::vector<Parcel>& batch,
                            const std::vector<std::vector<double>>& dist,
                            double departTime,
                            const Vehicle& vehicle);
};
    

#endif // DELIVERY_SCHEDULER_H