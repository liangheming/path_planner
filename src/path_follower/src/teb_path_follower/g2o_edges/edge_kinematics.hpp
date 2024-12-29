#pragma once

#include "base_edges.hpp"

class EdgeKinematics : public TebBinaryEdge<2, FollowerInfo *, Vertex2E, Vertex2E>
{
public:
    EdgeKinematics(FollowerInfo *info)
    {
        this->setMeasurement(info);
    }
    void computeError() override
    {
        const Point2E &p1 = static_cast<Vertex2E *>(_vertices[0])->estimate();
        const Point2E &p2 = static_cast<Vertex2E *>(_vertices[1])->estimate();
        Point2E delta = Point2E::delta(p1, p2);
        Eigen::Vector2d delta_trans = delta.translation();
        Eigen::Vector2d delta_dir = delta_trans.normalized();
        Eigen::Vector2d p1_dir = p1.direction();
        Eigen::Vector2d p2_dir = p2.direction();

        _error[0] = std::abs((p1_dir.x() + p2_dir.x()) * delta_dir.y() - (p1_dir.y() + p2_dir.y()) * delta_dir.x());

        double delta_angle = delta.angle();
        if (abs(delta_angle) <= 0.001)
            _error[1] = 0;
        else if (_measurement->exact_arc_length)
            _error[1] = penaltyBelow(fabs(delta_trans.norm() / (2 * sin(delta_angle / 2))), _measurement->min_turning_radius, _measurement->margin_kinematics_turning_radius);
        else
            _error[1] = penaltyBelow(fabs(delta_trans.norm() / delta_angle), _measurement->min_turning_radius, _measurement->margin_kinematics_turning_radius);
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};