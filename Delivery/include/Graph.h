#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include "MinHeap.h"
#include <algorithm>

class Graph
{
	private:
	struct Node
	{
		int id;
		double x,y;
		bool is_station;
	};
	int V;
	
	std::vector<Node> nodes;
	std::vector<double> dist;
	std::vector<int> prev;

	public:
	struct Edge
	{
		int u,v;
		double weight;
	};
	Graph(int count);
	std::vector<std::vector<Edge>> graph;
	const Node& getNode(int id) const { return nodes[id]; }
	const std::vector<std::vector<Edge>>& getAdjList() const { return graph; }
	void addNode(int id,double x,double y,bool is_station);
	void addEdge(int u,int v,double weight);
	int getVertexCount() const;
	void dijkstra(int source);
	double getDistance(int u,int v) const;
	std::vector<int> getPath(int u,int v) const;
};
#endif
