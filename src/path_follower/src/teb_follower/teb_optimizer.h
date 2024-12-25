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
#include <g2o/core/factory.h>

using TebBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
using TebLinearSolver = g2o::LinearSolverCSparse<TebBlockSolver::PoseMatrixType>;

class TebOptimizer
{
public:
    TebOptimizer(FollowerInfo *follower_info);

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

private:
    FollowerInfo *_follower_info;
    Pose2E _cached_start_vel;
    std::vector<Pose2E> _cached_trajectory;
    std::vector<double> _cached_time_diffs;
    std::vector<g2o::VertexSE2 *> _trajectory;
    std::vector<VertexDouble *> _time_diffs;
    std::shared_ptr<g2o::SparseOptimizer> _optimizer;
};