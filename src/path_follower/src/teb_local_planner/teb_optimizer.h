#pragma once
#include "commons.h"
#include <g2o/core/base_vertex.h>
#include <g2o/types/slam2d/vertex_se2.h>

class VertexDouble : public g2o::BaseVertex<1, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;
    virtual void setToOriginImpl();
    virtual void oplusImpl(const double *update);
};

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

    void clearGraph();

    bool optimizeGraph();

    void autoResize();

private:
    FollowerInfo *_follower_info;
    Pose2E _cached_start_vel;
    std::vector<Pose2E> _cached_trajectory;
    std::vector<double> _cached_time_diffs;
    std::vector<g2o::VertexSE2> _trajectory;
    std::vector<VertexDouble> _time_diffs;
};