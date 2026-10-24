#ifndef DATAREADER_H
#define DATAREADER_H

#include "Parcel.h"
#include "Vehicle.h"
#include "Graph.h"
#include <string>

struct Data
{
	Graph graph{0};
	std::vector<Parcel> parcels;
	Vehicle vehicle;
};

class DataReader
{
	public:
	static Data loadData(const std::string& car_file,const std::string& graph_file,const std::string& parcel_file);
};
#endif
