#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H
#include "grid_map_core/GridMap.hpp"
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <sys/time.h>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <math.h>    // for sqrt
#include <visualization_msgs/Marker.h>
#include "ros/ros.h"

using namespace std;
using namespace grid_map;
using namespace boost;
namespace move_control {
typedef float cost;
typedef adjacency_list<listS, vecS, undirectedS, no_property,
property<edge_weight_t, cost> > MyGraph;
typedef MyGraph::vertex_descriptor vertex;
typedef MyGraph::edge_descriptor edge_descriptor;
typedef graph_traits<MyGraph>::vertex_iterator vertex_iter;
typedef std::pair<int, int> edge;
typedef property_map<MyGraph, vertex_index_t>::type IndexMap;
class AStarPlanner {
public:
    AStarPlanner(ros::NodeHandle& nh);
    ~AStarPlanner(){}
    bool makePlan(Position& start,Position& target,vector<Position>& path);
private:
    ros::NodeHandle& nh_;
    ros::Publisher marker_pub_;
    vector<Position> locations_;
    MyGraph graph_;
    void init();
    vertex findClosedVertex(const Position& pos);
    void publishMarker();

};
}
#endif // ASTAR_PLANNER_H
