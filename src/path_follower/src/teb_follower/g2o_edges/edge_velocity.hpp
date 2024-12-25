#pragma once
#include "../commons.h"
#include <g2o/core/base_multi_edge.h>
#include <g2o/types/slam2d/vertex_se2.h>

struct VelocityMessurement
{
    VelocityMessurement() {}
    VelocityMessurement(const double &min_lin, const double &max_lin,
                        const double &min_rot, const double &max_rot,
                        const double &margin_in)
        : min_vel_linear(min_lin), max_vel_linear(max_lin),
          min_vel_rotation(min_rot), max_vel_rotation(max_rot),
          margin(margin_in) {}

    double min_vel_linear;
    double min_vel_rotation;
    double max_vel_linear;
    double max_vel_rotation;
    double margin;
};

class EdgeVelocity : public g2o::BaseMultiEdge<2, VelocityMessurement>
{
public:
    EdgeVelocity()
    {
        this->resize(3);
    }
    void computeError() override
    {
        g2o::VertexSE2 *v1 = static_cast<g2o::VertexSE2 *>(_vertices[0]);
        g2o::VertexSE2 *v2 = static_cast<g2o::VertexSE2 *>(_vertices[1]);
        VertexDouble *dt = static_cast<VertexDouble *>(_vertices[2]);
        Eigen::Vector2d delta_vector = (v2->estimate().translation() - v1->estimate().translation());
        double delta_distance = delta_vector.norm();
        double delta_angle = g2o::normalize_theta(v2->estimate().rotation().angle() - v1->estimate().rotation().angle());
        double vel_lin = delta_distance / dt->estimate();
        double vel_rot = delta_angle / dt->estimate();
        double sign_val = (delta_vector.x() * cos(v1->estimate().rotation().angle()) + delta_vector.y() * sin(v1->estimate().rotation().angle()));
        vel_lin *= soft_sign(sign_val, 100.0);

        _error[0] = penaltyBoundedValue(vel_lin, _measurement.min_vel_linear, _measurement.max_vel_linear, _measurement.margin);
        _error[1] = penaltyBoundedValue(vel_rot, _measurement.max_vel_rotation, _measurement.margin);
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