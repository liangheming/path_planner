#pragma once
#include "base_edges.hpp"

class EdgeShortestPath : public TebBinaryEdge<1, double, Vertex2E, Vertex2E>
{
public:
    EdgeShortestPath()
    {
        this->setMeasurement(0.0);
    }

    void computeError() override
    {
        const Point2E &p1 = static_cast<Vertex2E *>(_vertices[0])->estimate();
        const Point2E &p2 = static_cast<Vertex2E *>(_vertices[1])->estimate();
        Point2E delta = Point2E::delta(p1, p2);
        _error[0] = delta.translation().norm();
    }
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};