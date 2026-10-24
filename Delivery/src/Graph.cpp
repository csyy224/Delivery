#include "Graph.h"
#include"Utils.h"
#include <limits>
#include <algorithm>

Graph::Graph(int count)
{
	V = count;
	graph.resize(V);
	dist.resize(V, Utils::INF);
	prev.resize(V, -1);
	nodes.resize(V);
}

void Graph::addEdge(int u, int v, double weight) 
{
	graph[u].push_back({u, v, weight});
	graph[v].push_back({v, u, weight});
}

void Graph::addNode(int id, double x, double y, bool is_station) 
{
	if (id < 0 || id >= V) throw std::out_of_range("Node id out of range");
	nodes[id] = {id,x,y,is_station};
}

int Graph::getVertexCount() const { return V; }

void Graph::dijkstra(int source)
{
	dist.resize(V, Utils::INF);
	prev.resize(V, -1);
	dist[source] = 0;
	MinHeap minHeap;
	minHeap.push({source, 0});
	while (!minHeap.empty()) 
	{
		auto [u,d] = minHeap.top();
		minHeap.pop();
		if (d > dist[u]) continue;
		for(auto & edge : graph[u]) 
		{
			int v = edge.v;
			double weight = edge.weight;
			if (dist[u] + weight < dist[v]) 
			{
				dist[v] = dist[u] + weight;
				prev[v] = u;
				minHeap.push({v, dist[v]});
			}
		}
	}
}

double Graph::getDistance(int u, int v) const
{
	if (u < 0 || u >= V || v < 0 || v >= V) throw std::out_of_range("Vertex index out of range");
	return dist[v];
}

std::vector<int> Graph::getPath(int u, int v) const
{
	if (u < 0 || u >= V || v < 0 || v >= V) throw std::out_of_range("Vertex index out of range");
	std::vector<int> path;
	for(int i = v; i != -1; i = prev[i]) path.push_back(i);
	std::reverse(path.begin(),path.end());
	if (path.front() != u) return {};
	return path;
}
