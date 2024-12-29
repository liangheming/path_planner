#pragma once
#include "base_edges.hpp"

class EdgeAcceleration : public TebMultiEdge<2, FollowerInfo *>
{
public:
    EdgeAcceleration(FollowerInfo *info)
    {
        this->resize(5);
        this->setMeasurement(info);
    }
    void computeError() override
    {
        const Point2E &p1 = static_cast<Vertex2E *>(_vertices[0])->estimate();
        const Point2E &p2 = static_cast<Vertex2E *>(_vertices[1])->estimate();
        const Point2E &p3 = static_cast<Vertex2E *>(_vertices[2])->estimate();
        const double &dt1 = static_cast<VertexTimeDiff *>(_vertices[3])->estimate();
        const double &dt2 = static_cast<VertexTimeDiff *>(_vertices[4])->estimate();

        const Point2E delta1 = Point2E::delta(p1, p2);
        const Point2E delta2 = Point2E::delta(p2, p3);
        const Eigen::Vector2d delta1_trans = delta1.translation();
        const Eigen::Vector2d delta2_trans = delta2.translation();
        const double delta1_theta = delta1.angle();
        const double delta2_theta = delta2.angle();
        double delta1_dist = delta1_trans.norm();
        double delta2_dist = delta2_trans.norm();

        if (_measurement->exact_arc_length)
        {
            if (abs(delta1_theta) > 0.001)
            {
                const double radius = delta1_dist / (2 * sin(delta1_theta / 2));
                delta1_dist = fabs(delta1_theta * radius);
            }
            if (abs(delta2_theta) > 0.001)
            {
                const double radius = delta2_dist / (2 * sin(delta2_theta / 2));
                delta2_dist = fabs(delta2_theta * radius);
            }
        }

        double vel1 = delta1_dist / dt1;
        double vel2 = delta2_dist / dt2;
        vel1 *= soft_sign(delta1_trans.dot(p1.direction()), 100.0);
        vel2 *= soft_sign(delta2_trans.dot(p2.direction()), 100.0);

        double acc_lin = (vel2 - vel1) * 2 / (dt2 + dt1);
        _error[0] = penaltyBoundedValue(acc_lin, _measurement->max_acc_travel, _measurement->margin_max_acc_travel);

        double omega1 = delta1_theta / dt1;
        double omega2 = delta2_theta / dt2;
        double acc_rot = (omega2 - omega1) * 2 / (dt2 + dt1);
        _error[1] = penaltyBoundedValue(acc_rot, _measurement->max_acc_rotation, _measurement->margin_max_acc_rotation);
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class EdgeAccelerationStart : public TebMultiEdge<2, std::pair<FollowerInfo *, Velocity>>
{
public:
    EdgeAccelerationStart(FollowerInfo *info, const Velocity &vel)
    {
        this->resize(3);
        this->setMeasurement(std::make_pair(info, vel));
    }
    void computeError() override
    {
        const Point2E &p1 = static_cast<Vertex2E *>(_vertices[0])->estimate();
        const Point2E &p2 = static_cast<Vertex2E *>(_vertices[1])->estimate();
        const double &dt = static_cast<VertexTimeDiff *>(_vertices[2])->estimate();
        const Point2E delta = Point2E::delta(p1, p2);
        Eigen::Vector2d delta_trans = delta.translation();
        double delta_dist = delta_trans.norm();
        double delta_theta = delta.angle();
        FollowerInfo *info = _measurement.first;
        Velocity vel = _measurement.second;
        if (info->exact_arc_length && abs(delta_theta) > 0.001)
        {

            double radius = delta_dist / (2 * sin(delta_theta / 2));
            delta_dist = fabs(delta_theta * radius);
        }
        double vel1 = vel.x;
        double vel2 = delta_dist / dt;
        vel2 *= soft_sign(delta_trans.dot(p1.direction()), 100.0);
        double acc_lin = (vel2 - vel1) / dt;
        _error[0] = penaltyBoundedValue(acc_lin, info->max_acc_travel, info->margin_max_acc_travel);

        double omega1 = vel.theta;
        double omega2 = delta_theta / dt;
        double acc_rot = (omega2 - omega1) / dt;
        _error[1] = penaltyBoundedValue(acc_rot, info->max_acc_rotation, info->margin_max_acc_rotation);
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class EdgeAccelerationGoal : public TebMultiEdge<2, std::pair<FollowerInfo *, Velocity>>
{
public:
    EdgeAccelerationGoal(FollowerInfo *info, const Velocity &vel)
    {
        this->resize(3);
        this->setMeasurement(std::make_pair(info, vel));
    }
    void computeError() override
    {
        const Point2E &p1 = static_cast<Vertex2E *>(_vertices[0])->estimate();
        const Point2E &p2 = static_cast<Vertex2E *>(_vertices[1])->estimate();
        const double &dt = static_cast<VertexTimeDiff *>(_vertices[2])->estimate();
        Point2E delta = Point2E::delta(p1, p2);
        Eigen::Vector2d delta_trans = delta.translation();
        double delta_dist = delta_trans.norm();
        double delta_theta = delta.angle();
        FollowerInfo *info = _measurement.first;
        Velocity vel = _measurement.second;
        if (info->exact_arc_length && abs(delta_theta) > 0.001)
        {

            double radius = delta_dist / (2 * sin(delta_theta / 2));
            delta_dist = fabs(delta_theta * radius);
        }
        double vel1 = delta_dist / dt;
        vel1 *= soft_sign(delta_trans.dot(p1.direction()), 100.0);
        double vel2 = vel.x;
        double acc_lin = (vel2 - vel1) / dt;
        _error[0] = penaltyBoundedValue(acc_lin, info->max_acc_travel, info->margin_max_acc_travel);
        double omega1 = delta_theta / dt;
        double omega2 = vel.theta;
        double acc_rot = (omega2 - omega1) / dt;
        _error[1] = penaltyBoundedValue(acc_rot, info->max_acc_rotation, info->margin_max_acc_rotation);
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};