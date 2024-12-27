#pragma once
#include "commons.h"
#include "robot_model.h"
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
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
    std::vector<Point2D> &mutableObstacles() { return _obstacles.points(); }
    std::vector<Point2E> &mutableTrajectory() { return _cached_trajectory; }

private:
    void cacheToVertices();

    void verticesToCache();

    void buildGraph();

    bool optimizeGraph();

    void autoResize();
    void clearGraph() { _optimizer->clear(); }

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
    KDTree2D _kdtree;
};