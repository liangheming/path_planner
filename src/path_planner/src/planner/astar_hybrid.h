#pragma once
#include "node_hybrid.h"
#include "motion_hybrid.h"
#include "collision_checker.h"
#include <chrono>

class AstarHybrid
{
public:
    using Node = NodeHybrid;
    using NodePtr = Node *;
    using NodePtrVector = std::vector<NodePtr>;
    using NodeElement = std::pair<float, NodePtr>;
    using NodeGraph = std::unordered_map<unsigned int, Node>;
    using ObstacleHeuristicElement = std::pair<float, unsigned int>;
    struct NodeComparator
    {
        bool operator()(const NodeElement &a, const NodeElement &b) const
        {
            return a.first > b.first;
        }
    };
    struct ObstacleHeuristicComparator
    {
        bool operator()(const ObstacleHeuristicElement &a, const ObstacleHeuristicElement &b) const
        {
            return a.first > b.first;
        }
    };
    using NodeQueue = std::priority_queue<NodeElement, std::vector<NodeElement>, NodeComparator>;
    using ObstacleHeuristicQueue = std::vector<ObstacleHeuristicElement>;

    AstarHybrid(SearchInfo *search_info, CollisionChecker *collision_checker);
    void resetIterations(unsigned int &max_iterations, unsigned int &max_on_approach_iterations);
    void clearGraph();
    void clearQueue();
    bool createPath(const Coordinate &start, const Coordinate &goal, Coordinates &path, int &iterations);

private:
    NodePtr addToGraph(unsigned int index);

    bool isInputValid();

    bool setStart(const float &x, const float &y, const float &theta);

    bool setGoal(const float &x, const float &y, const float &theta);

    bool isGoal(NodePtr &node);

    void addToQueue(const float cost, NodePtr &node);

    float distance2d(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);

    float getMotionHeuristicCost(const Coordinate &start_coord, const Coordinate &goal_coord);

    bool cacheObstacleHeuristic();

    float getObstacleHeuristic(const Coordinate &start_coord, const Coordinate &goal_coord);

    float getHeuristicCost(const Coordinate &start_coord, const Coordinate &goal_coord);

    bool backtracePath(NodePtr &node, Coordinates &path);

    void getNeighbors(NodePtr &node, NodePtrVector &neighbors);

    NodePtr getAnalyticExpansion(NodePtr &current_node);

    NodePtr tryAnalyticExpansion(NodePtr &current_node, int &analytic_iterations, int &closest_distance);

    float getTravelCost(NodePtr &from, NodePtr &to);

    void fromCellToWorld( Coordinates &path);

    NodePtr _start;
    NodePtr _goal;
    MotionHybrid _motion_table;
    CollisionChecker *_collision_checker;
    SearchInfo *_search_info;
    NodeQueue _queue;
    NodeGraph _graph;
    std::vector<float> _obstacle_heuristic_lookup_table;
    ObstacleHeuristicQueue _obstacle_heuristic_queue;
};