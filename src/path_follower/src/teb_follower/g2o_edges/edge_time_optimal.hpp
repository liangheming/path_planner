#pragma once

#include "base_edges.hpp"

class EdgeTimeOptimal : public TebUnaryEdge<1, double, VertexTimeDiff>
{
public:
    EdgeTimeOptimal()
    {
        this->setMeasurement(0.0);
    }

    void computeError() override
    {
        double dt = static_cast<VertexTimeDiff *>(_vertices[0])->estimate();
        _error[0] = dt;
    }

    void linearizeOplus() override
    {
        _jacobianOplusXi(0, 0) = 1.0;
    }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};