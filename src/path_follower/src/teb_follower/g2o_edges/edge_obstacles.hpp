#pragma once
#include "../commons.h"
#include "../robot_model.h"
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/core/base_unary_edge.h>

struct ObstacleMessurement
{
    double min_distance;
    double inflation_distance;
    double min_margin;
    double inflation_margin;
    RobotModel *robot_model;
    Point2D *point;
    ObstacleMessurement() : min_distance(0), inflation_distance(0), min_margin(0), robot_model(nullptr), point(nullptr), inflation_margin(0.0)
    {
    }
    ObstacleMessurement(const double &_min_distance, const double &_inflation_distance, const double &_min_margin, const double &_inflation_margin, RobotModel *_robot, Point2D *point)
        : min_distance(_min_distance), inflation_distance(_inflation_distance), min_margin(_min_margin), inflation_margin(_inflation_margin), robot_model(_robot), point(point)
    {
    }
};

class EdgeObstacle : public g2o::BaseUnaryEdge<2, ObstacleMessurement, g2o::VertexSE2>
{
public:
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
        g2o::SE2 pose = static_cast<g2o::VertexSE2 *>(_vertices[0])->estimate();
        double x = pose.translation().x();
        double y = pose.translation().y();
        double theta = g2o::normalize_theta(pose.rotation().angle());
        double distance = _measurement.robot_model->distanceToPoint(Pose2E(x, y, theta), *_measurement.point);
        _error[0] = penaltyBelow(distance, _measurement.min_distance, _measurement.min_margin);
        _error[1] = penaltyBelow(distance, _measurement.inflation_distance, _measurement.inflation_margin);
    }
};
