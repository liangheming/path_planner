#pragma once
#include "../commons.h"
#include <g2o/core/base_unary_edge.h>

class EdgeTimeOptimal : public g2o::BaseUnaryEdge<1, double, VertexDouble>
{
public:
    EdgeTimeOptimal()
    {
        this->setMeasurement(0.);
    }
    void computeError() override
    {
        double dt = static_cast<VertexDouble *>(_vertices[0])->estimate();
        _error[0] = dt;
    }
    void linearizeOplus() override
    {
        _jacobianOplusXi(0, 0) = 1.0;
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