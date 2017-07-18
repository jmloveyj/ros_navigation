
#include "move_control/astar_planner.h"



using namespace boost;
using namespace std;

//We don't need map reference. just give a simple way to pass
// null reference to parent construction function
namespace move_control {



// euclidean distance heuristic
template <class Graph, class CostType, class LocMap>
class distance_heuristic : public astar_heuristic<Graph, CostType>
{
public:
    typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
    distance_heuristic(LocMap l, Vertex goal)
        : m_location(l), m_goal(goal) {}
    CostType operator()(Vertex u)
    {
        CostType dx = m_location[m_goal][0] - m_location[u][0];
        CostType dy = m_location[m_goal][1] - m_location[u][1];
        return ::sqrt(dx * dx + dy * dy);
    }
private:
    LocMap m_location;
    Vertex m_goal;
};


struct found_goal {}; // exception for termination

// visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
    astar_goal_visitor(Vertex goal) : m_goal(goal) {}
    template <class Graph>
    void examine_vertex(Vertex u, Graph& g) {
        if(u == m_goal)
            throw found_goal();
    }
private:
    Vertex m_goal;
};




AStarPlanner::AStarPlanner(ros::NodeHandle& nh):
    nh_(nh)
{
    init();
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

}

bool AStarPlanner::makePlan(Position& start,Position& target,vector<Position> &path)
{
    vertex entranceVertex = findClosedVertex(start);
    vertex exitVertex = findClosedVertex(target);

    vector<vertex> p(num_vertices(graph_));
    vector<cost> d(num_vertices(graph_));
    try {
        // call astar named parameter interface
        astar_search
                (graph_, entranceVertex,
                 distance_heuristic<MyGraph, cost, vector<Position>>
                 (locations_, exitVertex),
                 predecessor_map(&p[0]).distance_map(&d[0]).
                visitor(astar_goal_visitor<vertex>(exitVertex)));


    } catch(found_goal fg) { // found a path to the goal
        list<vertex> shortest_path;
        for(vertex v = exitVertex;; v = p[v]) {
            shortest_path.push_front(v);
            if(p[v] == v)
                break;
        }
        path.push_back(start);

        list<vertex>::iterator it;
        for(it=shortest_path.begin();it!=shortest_path.end();it++)
            path.push_back(locations_[*it]);

        path.push_back(target);
    }
    publishMarker();
}

void AStarPlanner::init()
{
    //TODO: here just add some default markup locations. In the future
    // The locations should be inserted dynamically
    int m =8;
    int n=5;
    locations_.push_back(Position(0.5*m,0*n)); //0
    locations_.push_back(Position(1.5*m,0*n)); //1
    locations_.push_back(Position(2.5*m,0*n)); //2
    locations_.push_back(Position(2.5*m,1*n)); //3
    locations_.push_back(Position(1.5*m,1*n)); //4
    locations_.push_back(Position(0.5*m,1*n)); //5
    locations_.push_back(Position(0.5*m,2*n)); //6
    locations_.push_back(Position(1.5*m,2*n)); //7
    locations_.push_back(Position(2.5*m,2*n)); //8


    graph_ = MyGraph(locations_.size());
    add_edge(0,1,graph_);
    add_edge(1,2,graph_);
    add_edge(2,3,graph_);
    add_edge(3,4,graph_);
    add_edge(4,5,graph_);
    add_edge(5,6,graph_);
    add_edge(6,7,graph_);
    add_edge(7,8,graph_);
    add_edge(0,5,graph_);
    add_edge(3,8,graph_);

}

vertex AStarPlanner::findClosedVertex(const Position &pos)
{
    float closedDistance =999.0;
    vertex closedVertex;

    std::pair<vertex_iter, vertex_iter> it;
    for (it = vertices(graph_); it.first != it.second; ++it.first){
        Position posIt= locations_[*it.first];
        float distance = hypot(pos[0]-posIt[0],pos[1]-posIt[1]);
        if(distance<closedDistance) {
            closedDistance = distance;
            closedVertex = *it.first;
        }
    }

    return closedVertex;
}

void AStarPlanner::publishMarker()
{
    visualization_msgs::Marker points, line_list;
    points.header.frame_id =  line_list.header.frame_id = "/odom";
    points.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns  = line_list.ns = "points_and_lines";
    points.action =  line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    // %EndTag(MARKER_INIT)%

    // %Tag(ID)%
    points.id = 0;
    line_list.id = 1;
    // %EndTag(ID)%

    // %Tag(TYPE)%
    points.type = visualization_msgs::Marker::POINTS;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    // %EndTag(TYPE)%

    // %Tag(SCALE)%
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.5;
    points.scale.y = 0.5;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list.scale.x = 0.1;
    // %EndTag(SCALE)%

    // %Tag(COLOR)%
    // Points are green
    points.color.r =1.0f;
    points.color.g = 0.447;
    points.color.b = 0.337;
    points.color.a = 1.0;

    // Line list is red
    line_list.color.b = 1.0;
    line_list.color.a = 1.0;
    // %EndTag(COLOR)%

    // %Tag(HELIX)%
    // Create the vertices for the points and lines

    vector<Position>::iterator it;
    for(it=locations_.begin();it!=locations_.end();it++){
        geometry_msgs::Point p;
        p.x = (*it)[0];
        p.y = (*it)[1];
        points.points.push_back(p);

    }

    MyGraph::edge_iterator eitr, eitr_end;
    for(tie(eitr, eitr_end) = edges(graph_);
        eitr != eitr_end;
        ++eitr){
        geometry_msgs::Point sourcePoint,targetPoint;
        int sourceId = source(*eitr,graph_);
        int targetId = target(*eitr,graph_);
        sourcePoint.x = locations_[sourceId][0];
        sourcePoint.y = locations_[sourceId][1];
        targetPoint.x = locations_[targetId][0];
        targetPoint.y = locations_[targetId][1];
        line_list.points.push_back(sourcePoint);
        line_list.points.push_back(targetPoint);

    }


    marker_pub_.publish(points);
    marker_pub_.publish(line_list);



}
}

