#pragma once
#include "../commons.h"
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam2d/vertex_se2.h>

class EdgeShortestPath : public g2o::BaseBinaryEdge<1, double, g2o::VertexSE2, g2o::VertexSE2>
{
public:
    EdgeShortestPath()
    {
        _measurement = 0.0;
    }
    void computeError()
    {
        g2o::SE2 v1 = static_cast<g2o::VertexSE2 *>(_vertices[0])->estimate();
        g2o::SE2 v2 = static_cast<g2o::VertexSE2 *>(_vertices[1])->estimate();
        _error[0] = (v1.translation() - v2.translation()).norm();
    }
    bool read(std::istream &is) override
    {
        return true;
    }
    bool write(std::ostream &os) const override
    {
        return os.good();
    }
};