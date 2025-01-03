#pragma once
#include "commons.h"
#include "robot_model.h"
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include "g2o_edges/edge_velocity.hpp"
#include "g2o_edges/edge_acceleration.hpp"
#include "g2o_edges/edge_obstacle.hpp"
#include "g2o_edges/edge_shortest_path.hpp"
#include "g2o_edges/edge_time_optimal.hpp"
#include "g2o_edges/edge_kinematics.hpp"
#include "g2o_edges/edge_via_point.hpp"

using TebBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
using TebLinearSolver = g2o::LinearSolverCSparse<TebBlockSolver::PoseMatrixType>;

class TebOptimizer
{
public:
    TebOptimizer(FollowerInfo *follower_info, RobotModel *robot_model);

    void initOptimizer();
    void clearObstacles() { _obstacles.clear(); }
    void addObstacle(const Point2D &point) { _obstacles.addPoint(point); }
    void addObstacle(const double &x, const double &y) { _obstacles.addPoint(x, y); }
    void buildObstacleKDTree() { _kdtree.buildIndex(); }
    void clearViaPoints() { _via_points.clear(); }
    void addViaPoint(const Point2D &point) { _via_points.push_back(point); }
    void addViaPoint(const double &x, const double &y) { _via_points.emplace_back(x, y); }
    std::pair<bool, Velocity> &mutableStartVelocity() { return _start_vel; }
    std::pair<bool, Velocity> &mutableGoalVelocity() { return _goal_vel; }
    std::vector<Point2D> &mutableCanidateObstacles() { return _obstacles.points(); }
    std::vector<Point2D *> &mutableAffectedObstacles() { return _affected_obstacles; }
    std::vector<Point2E> &mutableTrajectory() { return _cached_trajectory; }
    std::vector<double> &mutableTimediffs() { return _cached_timediffs; }

    bool smoothTrajectory(std::vector<Point2E> &trajectory);

    void clearCached()
    {
        _cached_trajectory.clear();
        _cached_timediffs.clear();
    }

private:
    void pruneTrajectory(const Point2E &start, const Point2E &end);
    void initTrajectory(std::vector<Point2E> &trajectory);
    bool optimizeTrajectory();
    double esitmateTimediff(const Point2E &p1, const Point2E &p2);

    void cacheToVertices();

    void verticesToCache();

    void buildGraph(const int &iteration);

    bool optimizeGraph();

    void autoResize();
    void clearGraph() { _optimizer->clear(); }

    void addVelocityEdges();
    void addAccelerationEdges();
    void addObstacleEdges();
    void addKineticEdges();
    void addShortestPathEdges();
    void addTimeOptimalEdges();
    void addViapointEdges();

    FollowerInfo *_info;
    RobotModel *_robot;
    std::vector<double> _cached_timediffs;
    std::vector<Point2E> _cached_trajectory;
    std::vector<Vertex2E *> _trajectory;
    std::vector<VertexTimeDiff *> _timediffs;
    std::shared_ptr<g2o::SparseOptimizer> _optimizer;
    PointCloud2D _obstacles;
    std::pair<bool, Velocity> _start_vel;
    std::pair<bool, Velocity> _goal_vel;
    std::vector<Point2D> _via_points;
    std::vector<Point2D *> _affected_obstacles;
    KDTree2D _kdtree;
};