#ifndef TASK1_H
#define TASK1_H

#include "Graph.h"
#include <vector>

namespace Task1
{
    void query( Graph& graph, int u, int v);
    void runQueries( Graph& graph, const std::vector<std::pair<int,int>>& queries);
};
#endif
