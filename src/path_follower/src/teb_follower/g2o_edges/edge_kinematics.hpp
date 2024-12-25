#pragma once
#include "../commons.h"
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/core/base_binary_edge.h>

struct KinematicMessurement
{
    double mini_turn_radius;
};

class EdgeKinematicCarLike : public g2o::BaseBinaryEdge<2, KinematicMessurement, g2o::VertexSE2, g2o::VertexSE2>
{
public:
    void computeError() override
    {
        g2o::VertexSE2 *v1 = static_cast<g2o::VertexSE2 *>(_vertices[0]);
        g2o::VertexSE2 *v2 = static_cast<g2o::VertexSE2 *>(_vertices[1]);
        Eigen::Vector2d delta_s = (v2->estimate().translation() - v1->estimate().translation());
        Eigen::Vector2d delta_dir = delta_s.normalized();

        Eigen::Vector2d v1_dir = v1->estimate().rotation().toRotationMatrix().col(0);
        Eigen::Vector2d v2_dir = v2->estimate().rotation().toRotationMatrix().col(0);
        _error[0] = fabs((v1_dir.x() + v2_dir.x()) * delta_dir.y() - (v1_dir.y() + v2_dir.y()) * delta_dir.x());

        double angle_diff = g2o::normalize_theta(v2->estimate().rotation().angle() - v1->estimate().rotation().angle());
        if (fabs(angle_diff) <= 0.00001)
            _error[1] = 0.0;
        else
            _error[1] = penaltyBelow(delta_s.norm() / fabs(angle_diff), _measurement.mini_turn_radius, 0.00000);
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