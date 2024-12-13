#pragma once
#include "commons.h"
#include "node_hybrid.h"
#include "motion_hybrid.h"
#include "collision_checker.h"
#include <queue>
#include <unordered_map>
#include <chrono>

class AstarAlgorithm
{
public:
    using Node = NodeHybrid;
    using NodePtr = Node *;
    using NodePtrVector = std::vector<NodePtr>;
    using NodeElement = std::pair<float, NodePtr>;
    using NodeGraph = std::unordered_map<unsigned int, Node>;
    using ObstacleHeuristicElement = std::pair<float, unsigned int>;
    using ObstacleHeuristicQueue = std::vector<ObstacleHeuristicElement>;

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

    AstarAlgorithm(SearchInfo &search_info, CollisionChecker &collision_checker);

    void resetIterations(unsigned int &max_iterations, unsigned int &max_on_approach_iterations)
    {
        _max_iterations = max_iterations;
        _max_on_approach_iterations = max_on_approach_iterations;
    }

    bool setStart(const unsigned int &x, const unsigned int &y, const unsigned int &theta);

    bool setGoal(const unsigned int &x, const unsigned int &y, const unsigned int &theta);

    bool isInputValid();

    void clearGraph();

    void clearQueue();

    bool createPath(Coordinates &path, int &iterations, const float &tolerance);

    NodePtr addToGraph(unsigned int index);

    bool isGoal(NodePtr &node);

    void addToQueue(const float cost, NodePtr &node);

    float getHeuristicCost(const Coordinate &start_coord, const Coordinate &goal_coord);

    float getMotionHeuristicCost(const Coordinate &start_coord, const Coordinate &goal_coord);

    float getObstacleHeuristic(const Coordinate &start_coord, const Coordinate &goal_coord);

    float getTravelCost(NodePtr &from, NodePtr &to);

    float distance2d(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);

    bool cacheObstacleHeuristic();

    bool backtracePath(NodePtr &node, Coordinates &path);

    void getNeighbors(NodePtr &node, NodePtrVector &neighbors);
    MotionHybrid &getMotionTable() { return _motion_table; }

    NodePtr tryAnalyticExpansion(NodePtr &current_node, int &analytic_iterations, int &closest_distance);

    NodePtr getAnalyticExpansion(NodePtr &current_node);

private:
    NodePtr _start;
    NodePtr _goal;
    MotionHybrid _motion_table;
    CollisionChecker _collision_checker;
    costmap_2d::Costmap2D *_costmap;
    SearchInfo _search_info;
    unsigned int _max_iterations;
    unsigned int _max_on_approach_iterations;

    NodeQueue _queue;
    NodeGraph _graph;

    std::vector<float> _obstacle_heuristic_lookup_table;
    ObstacleHeuristicQueue _obstacle_heuristic_queue;
};