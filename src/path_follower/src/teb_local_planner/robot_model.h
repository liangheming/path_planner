#include "commons.h"
#include <stdexcept>
#include <limits>

class RobotModel
{
public:
    virtual double distanceToPoint(const Pose2E &pose, const Point2D &point) const = 0;
};

class CircleRobotModel : public RobotModel
{
public:
    CircleRobotModel() : _radius(0) {}
    CircleRobotModel(double &&radius) : _radius(radius) {}
    CircleRobotModel(const double &radius) : _radius(radius) {}

    double distanceToPoint(const Pose2E &pose, const Point2D &point) const override;
    double &radius() { return _radius; }

private:
    double _radius;
};

class PolygonRobotModel : public RobotModel
{
public:
    PolygonRobotModel(const std::vector<Point2D> &vertices);

    double distanceToPoint(const Pose2E &pose, const Point2D &point) const override;
    double inscribedRadius() const { return _inscriberadius; }
    double circumRadius() const { return _circumradius; }

private:
    std::vector<Point2D> _vertices;
    double _inscriberadius;
    double _circumradius;
    void computeRadius();
};