#pragma once
#include "commons.h"
#include <g2o/core/base_vertex.h>
#include <g2o/types/slam2d/vertex_se2.h>

class VertexDouble : public g2o::BaseVertex<1, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexDouble();
    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;
    virtual void setToOriginImpl();
    virtual void oplusImpl(const double *update);
};

class TebOptimizer
{
public:
    TebOptimizer(FollowerInfo *follower_info);

private:
    FollowerInfo *_follower_info;
    std::vector<g2o::VertexSE2> _nodes;


};