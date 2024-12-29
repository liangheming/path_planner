#pragma once
#include "../robot_model.h"
#include "base_edges.hpp"

class EdgeObstacle : public TebUnaryEdge<2, Point2D *, Vertex2E>
{
public:
    EdgeObstacle(Point2D *obstacle)
    {
        this->setMeasurement(obstacle);
    }

    void setInfoParameters(RobotModel *robot, FollowerInfo *info)
    {
        _robot = robot;
        _info = info;
    }

    void computeError() override
    {
        const Point2E &pose = static_cast<Vertex2E *>(_vertices[0])->estimate();
        const Point2D &obstacle = *measurement();
        double dist = _robot->distanceToPoint(pose, obstacle);
        _error[0] = penaltyBelow(dist, _info->obstacle_collision_dist, _info->margin_obstacle_collision);
        _error[1] = penaltyBelow(dist, _info->obstacle_inflation_dist, _info->margin_obstacle_inflation);
    }

private:
    RobotModel *_robot;
    FollowerInfo *_info;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};