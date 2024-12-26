#pragma once
#include "../commons.h"
#include <g2o/core/base_multi_edge.h>
#include <g2o/types/slam2d/vertex_se2.h>

struct AccelerationMessurement
{
    AccelerationMessurement() : max_acc_vel(0), max_acc_theta(0), acc_vel_margin(0), acc_theta_margin(0) {}
    AccelerationMessurement(const double &vel, const double &theta, const double &vel_margin, const double &theta_margin)
        : max_acc_vel(vel), max_acc_theta(theta), acc_vel_margin(vel_margin), acc_theta_margin(theta_margin) {}
    double max_acc_vel;
    double max_acc_theta;
    double acc_vel_margin;
    double acc_theta_margin;
    double cur_vel_lin = 0.0;
    double cur_vel_theta = 0.0;
};

class EdgeAcceleration : public g2o::BaseMultiEdge<2, AccelerationMessurement>
{
public:
    EdgeAcceleration()
    {
        this->resize(5);
    }
    bool read(std::istream &is) override
    {
        return true;
    }
    bool write(std::ostream &os) const override
    {
        return os.good();
    }
    void computeError() override
    {
        g2o::VertexSE2 *v1 = static_cast<g2o::VertexSE2 *>(_vertices[0]);
        g2o::VertexSE2 *v2 = static_cast<g2o::VertexSE2 *>(_vertices[1]);
        g2o::VertexSE2 *v3 = static_cast<g2o::VertexSE2 *>(_vertices[2]);

        VertexDouble *dt1 = static_cast<VertexDouble *>(_vertices[3]);
        VertexDouble *dt2 = static_cast<VertexDouble *>(_vertices[4]);

        Eigen::Vector2d diff1 = v2->estimate().translation() - v1->estimate().translation();
        Eigen::Vector2d diff2 = v3->estimate().translation() - v2->estimate().translation();

        double dist1 = diff1.norm();
        double dist2 = diff2.norm();
        double angle_diff1 = g2o::normalize_theta(v2->estimate().rotation().angle() - v1->estimate().rotation().angle());
        double angle_diff2 = g2o::normalize_theta(v3->estimate().rotation().angle() - v2->estimate().rotation().angle());

        double vel1 = dist1 / dt1->estimate();
        double vel2 = dist2 / dt2->estimate();

        int sign_vel = g2o::sign(diff1[0] * cos(v1->estimate().rotation().angle()) + diff1[1] * sin(v1->estimate().rotation().angle()));
        if (sign_vel == 0)
            sign_vel = 1;

        vel1 *= sign_vel;
        sign_vel = g2o::sign(diff2[0] * cos(v2->estimate().rotation().angle()) + diff2[1] * sin(v2->estimate().rotation().angle()));
        if (sign_vel == 0)
            sign_vel = 1;
        vel2 *= sign_vel;

        double acc_lin = (vel2 - vel1) / (dt2->estimate() + dt1->estimate());
        _error[0] = penaltyBoundedValue(acc_lin, _measurement.max_acc_vel, _measurement.acc_vel_margin);

        const double omega1 = angle_diff1 / dt1->estimate();
        const double omega2 = angle_diff2 / dt2->estimate();
        double acc_rot = (omega2 - omega1) / (dt2->estimate() + dt1->estimate());
        _error[1] = penaltyBoundedValue(acc_rot, _measurement.max_acc_theta, _measurement.acc_theta_margin);
    }
};

class EdgeAccelerationStart : public g2o::BaseMultiEdge<2, AccelerationMessurement>
{
public:
    EdgeAccelerationStart()
    {
        this->resize(3);
    }
    bool read(std::istream &is) override
    {
        return true;
    }
    bool write(std::ostream &os) const override
    {
        return os.good();
    }

    void computeError() override
    {
        g2o::SE2 v1 = static_cast<g2o::VertexSE2 *>(_vertices[0])->estimate();
        g2o::SE2 v2 = static_cast<g2o::VertexSE2 *>(_vertices[1])->estimate();
        double dt = static_cast<VertexDouble *>(_vertices[2])->estimate();
        Eigen::Vector2d diff = v2.translation() - v1.translation();
        double dist = diff.norm();
        double vel1 = _measurement.cur_vel_lin;
        double vel2 = dist / dt;
        int sign_vel = g2o::sign(diff[0] * cos(v1.rotation().angle()) + diff[1] * sin(v1.rotation().angle()));
        if (sign_vel == 0)
            sign_vel = 1;
        vel2 *= sign_vel;
        double angle_diff = g2o::normalize_theta(v2.rotation().angle() - v1.rotation().angle());
        double omega1 = _measurement.cur_vel_theta;
        double omega2 = angle_diff / dt;

        double acc_lin = (vel2 - vel1) / dt;
        double acc_rot = (omega2 - omega1) / dt;

        _error[0] = penaltyBoundedValue(acc_lin, _measurement.max_acc_vel, _measurement.acc_vel_margin);
        _error[1] = penaltyBoundedValue(acc_rot, _measurement.max_acc_theta, _measurement.acc_theta_margin);
    }
};

class EdgeAccelerationGoal : public g2o::BaseMultiEdge<2, AccelerationMessurement>
{
public:
    EdgeAccelerationGoal()
    {
        this->resize(3);
    }
    bool read(std::istream &is) override
    {
        return true;
    }
    bool write(std::ostream &os) const override
    {
        return os.good();
    }
    void computeError() override
    {
        g2o::SE2 v1 = static_cast<g2o::VertexSE2 *>(_vertices[0])->estimate();
        g2o::SE2 v2 = static_cast<g2o::VertexSE2 *>(_vertices[1])->estimate();
        double dt = static_cast<VertexDouble *>(_vertices[2])->estimate();
        Eigen::Vector2d diff = v2.translation() - v1.translation();
        double dist_diff = diff.norm();
        double angle_diff = g2o::normalize_theta(v2.rotation().angle() - v1.rotation().angle());

        double vel1 = dist_diff / dt;
        double vel2 = _measurement.cur_vel_lin;

        int sign_vel = g2o::sign(diff.x() * cos(v1.rotation().angle()) + diff.y() * sin(v1.rotation().angle()));
        if (sign_vel == 0)
            sign_vel = 1;
        vel1 *= sign_vel;
        double acc_lin = (vel2 - vel1) / dt;
        _error[0] = penaltyBoundedValue(acc_lin, _measurement.max_acc_vel, _measurement.acc_vel_margin);

        double omega1 = angle_diff / dt;
        double omega2 = _measurement.cur_vel_theta;
        double acc_rot = (omega2 - omega1) / dt;
        _error[1] = penaltyBoundedValue(acc_rot, _measurement.max_acc_theta, _measurement.acc_theta_margin);
    }
};