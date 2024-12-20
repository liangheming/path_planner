#include "robot_model.h"

double CircleRobotModel::distanceToPoint(const Pose2E &pose, const Point2D &point) const
{
    return std::hypot(point.x - pose.x, point.y - pose.y) - _radius;
}

PolygonRobotModel::PolygonRobotModel(const std::vector<Point2D> &vertices) : _vertices(vertices)
{
    if (_vertices.size() < 3)
        throw std::runtime_error("PolygonRobotModel: Polygon must have at least 3 vertices");
    computeRadius();
}

void PolygonRobotModel::computeRadius()
{
    Point2D center;
    double distance = 0.0;
    double min_dist = std::numeric_limits<double>::max();
    double max_dist = std::numeric_limits<double>::min();
    for (Point2D &vertex : _vertices)
    {
        distance = Point2D::distance(center, vertex);
        min_dist = std::min(min_dist, distance);
        max_dist = std::max(max_dist, distance);
    }

    for (size_t i = 0; i < _vertices.size() - 1; i++)
    {
        distance = pointToLineDistance(center, _vertices[i], _vertices[i + 1]);
        min_dist = std::min(min_dist, distance);
        max_dist = std::max(max_dist, distance);
    }
    distance = pointToLineDistance(center, _vertices.back(), _vertices.front());
    _inscriberadius = std::min(min_dist, distance);
    _circumradius = std::max(max_dist, distance);
}

double PolygonRobotModel::distanceToPoint(const Pose2E &pose, const Point2D &point) const
{
    double distance = Point2D::distance(Point2D(pose.x, pose.y), point);
    if (distance <= _inscriberadius)
        return distance - _circumradius;

    double dist = std::numeric_limits<double>::max();

    Point2D start_point;
    Point2D end_point;
    for (size_t i = 0; i < _vertices.size() - 1; i++)
    {
        start_point.x = cos(pose.theta) * _vertices[i].x - sin(pose.theta) * _vertices[i].y + pose.x;
        start_point.y = sin(pose.theta) * _vertices[i].x + cos(pose.theta) * _vertices[i].y + pose.y;
        end_point.x = cos(pose.theta) * _vertices[i + 1].x - sin(pose.theta) * _vertices[i + 1].y + pose.x;
        end_point.y = sin(pose.theta) * _vertices[i + 1].x + cos(pose.theta) * _vertices[i + 1].y + pose.y;
        distance = pointToLineDistance(point, start_point, end_point);
        dist = std::min(dist, distance);
    }

    start_point.x = cos(pose.theta) * _vertices.back().x - sin(pose.theta) * _vertices.back().y + pose.x;
    start_point.y = sin(pose.theta) * _vertices.back().x + cos(pose.theta) * _vertices.back().y + pose.y;
    end_point.x = cos(pose.theta) * _vertices.front().x - sin(pose.theta) * _vertices.front().y + pose.x;
    end_point.y = sin(pose.theta) * _vertices.front().x + cos(pose.theta) * _vertices.front().y + pose.y;
    distance = pointToLineDistance(point, start_point, end_point);
    dist = std::min(dist, distance);
    return dist;
}
