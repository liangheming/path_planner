#pragma once
#include "commons.h"
#include <g2o/core/base_vertex.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include "g2o_edges/edge_velocity.hpp"
#include "g2o_edges/edge_kinematics.hpp"
#include "g2o_edges/edge_acceleration.hpp"
#include "g2o_edges/edge_shortest_path.hpp"
#include "g2o_edges/edge_time_optimal.hpp"
#include "g2o_edges/edge_obstacles.hpp"
#include <g2o/core/factory.h>
#include "robot_model.h"

using TebBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
using TebLinearSolver = g2o::LinearSolverCSparse<TebBlockSolver::PoseMatrixType>;

class TebOptimizer
{
public:
    TebOptimizer(FollowerInfo *follower_info, RobotModel *robot_model);

    Pose2E plan(const Pose2Es &initial_trajectory, const Pose2E &start_vel);

    void initTrajectory();

    void pruneTrajectory();

    double estimateTimeDiff(const Pose2E &p1, const Pose2E &p2);

    bool optimize();

    void buildGraph(double weight_multiplier);

    void setVertices();

    void getVertices();

    void clearGraph();

    bool optimizeGraph();

    void autoResize();

    void addVelocityEdges();

    void addKinematicsEdges();

    void addAccelerationEdges();

    void addObstacleEdges();

    void addShortestPathEdges();

    void addTimeOptimalEdges();

    void clearObstacles();

    void addObstacles(const double &x, const double &y);

    void rebuildKDTree();

    PointCloud2D &mutableObstacles() { return _obstacles; }

    std::vector<Pose2E> &mutableTrajectory() { return _cached_trajectory; }

private:
    FollowerInfo *_follower_info;
    Pose2E _cached_start_vel;
    std::vector<Pose2E> _cached_trajectory;
    std::vector<double> _cached_time_diffs;
    std::vector<g2o::VertexSE2 *> _trajectory;
    std::vector<VertexDouble *> _time_diffs;
    std::shared_ptr<g2o::SparseOptimizer> _optimizer;
    PointCloud2D _obstacles;
    KDTree2D _kdtree;
    RobotModel *_robot_model;
};