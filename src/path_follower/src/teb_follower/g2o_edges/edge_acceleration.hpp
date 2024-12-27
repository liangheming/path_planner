#pragma once
#include "base_edges.hpp"

class EdgeAcceleration : public TebMultiEdge<2, FollowerInfo *>
{
public:
    EdgeAcceleration()
    {
        this->resize(5);
    }
    void computeError() override
    {
        const Point2E &p1 = static_cast<Vertex2E *>(_vertices[0])->estimate();
        const Point2E &p2 = static_cast<Vertex2E *>(_vertices[1])->estimate();
        const Point2E &p3 = static_cast<Vertex2E *>(_vertices[2])->estimate();
        const double &dt1 = static_cast<VertexTimeDiff *>(_vertices[3])->estimate();
        const double &dt2 = static_cast<VertexTimeDiff *>(_vertices[4])->estimate();
        
    }
};