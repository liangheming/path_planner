#pragma once

#include "base_edges.hpp"

class EdgeViaPoint : public TebUnaryEdge<1, std::pair<FollowerInfo *, Point2D>, Vertex2E>
{
public:
    EdgeViaPoint(FollowerInfo *info, const Point2D &via_point)
    {
        this->setMeasurement(std::make_pair(info, via_point));
    }

    void computeError() override
    {
        const Point2E &pose = static_cast<Vertex2E *>(_vertices[0])->estimate();
        _error[0] = (pose.translation() - _measurement.second.toVector()).norm();
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};