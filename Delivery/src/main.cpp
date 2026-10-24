// main.cpp
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <stdexcept>

// 数据与模型
#include "DataReader.h"
#include "Parcel.h"
#include "Vehicle.h"
#include "Graph.h"

// 任务接口
#include "Task1.h"
#include "Task2.h"
#include "Task3.h"
#include "Task4.h"
#include "Task5.h"
#include "Task6.h"

// 辅助常量
const std::string MAP_FILE     = "map.txt";
const std::string PARCEL_FILE  = "packages.txt";
const std::string CAR_FILE     = "car.txt";
const std::string RETURNS_FILE = "returns.txt";  // 可选，用于 T4

// 从 dataDir 加载数据
Data loadData(const std::string& dataDir) {
    std::string carPath    = dataDir + "/" + CAR_FILE;
    std::string graphPath  = dataDir + "/" + MAP_FILE;
    std::string parcelPath = dataDir + "/" + PARCEL_FILE;
    return DataReader::loadData(carPath, graphPath, parcelPath);
}

// 加载退货节点列表（可选）
std::vector<int> loadReturnNodes(const std::string& dataDir) {
    std::vector<int> nodes;
    std::string returnsPath = dataDir + "/" + RETURNS_FILE;
    std::ifstream fin(returnsPath);
    if (!fin.is_open()) {
        std::cout << "[Info] No returns.txt found, skipping T4.\n";
        return nodes;
    }
    int node;
    while (fin >> node) {
        nodes.push_back(node);
    }
    fin.close();
    return nodes;
}

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " <task_id> <data_directory>\n";
    std::cout << "  task_id: 1 | 2 | 3 | 4 | 5 | 6 | custom\n";
    std::cout << "    custom  : run Task6 with custom generated test cases\n";
    std::cout << "  data_directory: path to folder containing map.txt, packages.txt, car.txt\n\n";
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }

    std::string taskArg = argv[1];
    std::string dataDir = argv[2];

    try {
        // 加载通用数据（除了 T6 custom 模式可能不需要外部数据）
        Data data = loadData(dataDir);

        // 根据任务 ID 分发
        if (taskArg == "1") {
            // T1: 默认查询 0 到所有节点的最短路
            std::cout << "Running Task 1: Shortest paths from station (node 0)\n";
            std::vector<std::pair<int,int>> queries;
            for (int i = 0; i < data.graph.getVertexCount(); ++i) {
                if (i != 0) queries.emplace_back(0, i);
            }
            Task1::runQueries(data.graph, queries);
        }
        else if (taskArg == "2") {
            std::cout << "Running Task 2: Minimize dissatisfaction\n";
            Task2::run(data.graph, data.parcels, data.vehicle);
        }
        else if (taskArg == "3") {
            std::cout << "Running Task 3: Delivery cost with capacity\n";
            Task3::run(data.graph, data.parcels, data.vehicle);
        }
        else if (taskArg == "4") {
            std::cout << "Running Task 4: Return pickup (TSP)\n";
            std::vector<int> returns = loadReturnNodes(dataDir);
            if (returns.empty()) {
                std::cout << "No return nodes to process. Exiting.\n";
            } else {
                Task4::run(data.graph, returns);
            }
        }
        else if (taskArg == "5") {
            std::cout << "Running Task 5: Dual-vehicle coordination\n";
            Task5::run(data.graph, data.parcels, data.vehicle);
        }
        else if (taskArg == "6") {
            std::cout << "Running Task 6: Strategy comparison on given data\n";
            // 同时对比 T2 和 T3
            Task6::runT3Comparison(data.graph, data.parcels, data.vehicle);
            Task6::runT2Comparison(data.graph, data.parcels, data.vehicle);
        }
        else {
            std::cerr << "Error: unknown task ID '" << taskArg << "'\n";
            printUsage(argv[0]);
            return 1;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Runtime error: " << e.what() << "\n";
        return 2;
    }
    return 0;
}