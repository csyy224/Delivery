#include "DataReader.h"
#include <fstream>
#include <stdexcept>

Data DataReader::loadData(const std::string& car_file,const std::string& graph_file,const std::string& parcel_file)
{
	Data data;
	std::fstream cfile(car_file),gfile(graph_file),pfile(parcel_file);
	if(!cfile.is_open() || !gfile.is_open() || !pfile.is_open()) throw std::runtime_error("Failed to open input files");
	double speed,car_weight,capacity;
	cfile >> speed >> car_weight >> capacity;
	data.vehicle = Vehicle(speed,car_weight,capacity);
	int k; pfile >> k;
	data.parcels.reserve(k);
	for(int i = 0;i < k;i++)
	{
		int id,dest; double weight,S,T;
		pfile >> id >> weight >> dest >> S >> T;
		data.parcels.push_back(Parcel{id,dest,weight,S,T});
	}
	int n,m; gfile >> n >> m;
	data.graph = Graph(n);
	for(int i = 0;i < n;i++)
	{
		int id; double x,y; bool is_station;
		gfile >> id >> x >> y >> is_station;
		data.graph.addNode(id,x,y,is_station);
	}
	for(int i = 0;i < m;i++)
	{
		int u,v; double weight;
		gfile >> u >> v >> weight;
		data.graph.addEdge(u,v,weight);
	}
	cfile.close(); gfile.close(); pfile.close();
	return data;
}