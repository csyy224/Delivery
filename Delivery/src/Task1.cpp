#include "Task1.h"
#include "Visualizer.h"
#include <iostream>
#include <iomanip>

void Task1::query(Graph & graph,int u,int v)
{
	graph.dijkstra(u);
	double distance = graph.getDistance(u,v);
	if(distance == INT_MAX)
	{
		std::cout << "No path from " << u << " to " << v << "\n";
		return;
	}
	std::vector<int> path = graph.getPath(u,v);
	std::cout << "Shortest path from " << u << " to " << v << ": ";
	for(size_t i = 0;i < path.size();i++)
	{
		std::cout << path[i];
		if(i != path.size() - 1) std::cout << " -> ";
	}
	std::cout << "\nDistance: " << std::fixed << std::setprecision(2) << distance << "\n";
}

void Task1::runQueries(Graph & graph, const std::vector<std::pair<int,int>> & queries)
{
    // 先展示地图
    Visualizer::printMap(graph);
    // 再逐条查询最短路径
    for (auto & [u, v] : queries)
        query(graph, u, v);
}