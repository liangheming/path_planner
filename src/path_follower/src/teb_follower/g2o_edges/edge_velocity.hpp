#pragma once
#include "base_edges.hpp"

class EdgeVelocity : public TebMultiEdge<2, FollowerInfo *>
{
public:
    EdgeVelocity()
    {
        this->resize(3);
    }

    void computeError() override
    {
        const Point2E& p1 = static_cast<Vertex2E *>(_vertices[0])->estimate();
        const Point2E& p2 = static_cast<Vertex2E *>(_vertices[1])->estimate();
        const double& dt = static_cast<VertexTimeDiff *>(_vertices[2])->estimate();
        Point2E delta = Point2E::delta(p1, p2);
        Eigen::Vector2d delta_trans = delta.translation();
        double delta_angle = delta.angle();
        double delta_distance = delta_trans.norm();
        if (_measurement->exact_arc_length && delta_angle > 0.001)
        {
            double radius = delta_distance / (2 * sin(delta_angle / 2));
            delta_distance = fabs(delta_angle * radius);
        }

        double vel_lin = delta_distance / dt;
        double vel_rot = delta_angle / dt;
        vel_lin *= soft_sign(p1.direction().dot(delta_trans), 100.0);
        _error[0] = penaltyBoundedValue(vel_lin, -_measurement->max_vel_backward, _measurement->max_vel_forward, _measurement->margin_max_vel_travel);
        _error[1] = penaltyBoundedValue(vel_rot, -_measurement->max_vel_rotation, _measurement->max_vel_rotation, _measurement->margin_max_vel_rotation);
    }
};
