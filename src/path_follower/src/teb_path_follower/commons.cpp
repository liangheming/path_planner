#include "commons.h"

double Point2D::distance(const Point2D &p1, const Point2D &p2)
{
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

Point2D Point2D::delta(const Point2D &p1, const Point2D &p2)
{
    return Point2D(p2.x - p1.x, p2.y - p1.y);
}

double Point2E::distance(const Point2E &p1, const Point2E &p2)
{
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

Point2E Point2E::delta(const Point2E &p1, const Point2E &p2)
{
    return Point2E(p2.x - p1.x, p2.y - p1.y, normalize_theta(p2.theta - p1.theta));
}
Point2E Point2E::average(const Point2E &p1, const Point2E &p2)
{
    double cx = (p1.x + p2.x) / 2;
    double cy = (p1.y + p2.y) / 2;
    double ctheta = average_angle(p1.theta, p2.theta);
    return Point2E(cx, cy, ctheta);
}

void Point2E::setZero()
{
    x = 0.0;
    y = 0.0;
    theta = 0.0;
}
double pointToLineDistance(const Point2D &point, const Point2D &line_start, const Point2D &line_end)
{
    Point2D nearest_point = nearestPointOnLine(point, line_start, line_end);
    return Point2D::distance(point, nearest_point);
}

Point2D nearestPointOnLine(const Point2D &point, const Point2D &line_start, const Point2D &line_end)
{
    double dx = line_end.x - line_start.x;
    double dy = line_end.y - line_start.y;
    double sq_norm = dx * dx + dy * dy;
    if (sq_norm < 1e-6)
        return line_start;
    double u = ((point.x - line_start.x) * dx + (point.y - line_start.y) * dy) / sq_norm;
    if (u <= 0)
        return line_start;
    if (u >= 1)
        return line_end;
    return Point2D(line_start.x + u * dx, line_start.y + u * dy);
}

Vertex2E::Vertex2E(const Point2E &p2e)
{
    _estimate = p2e;
    setFixed(false);
}

Vertex2E::Vertex2E(double &&x, double &&y, double &&theta)
{
    _estimate = Point2E(x, y, theta);
    setFixed(false);
}

Vertex2E::Vertex2E(const double &x, const double &y, const double &theta)
{
    _estimate = Point2E(x, y, theta);
    setFixed(false);
}

void Vertex2E::oplusImpl(const double *update)
{
    _estimate.x += update[0];
    _estimate.y += update[1];
    _estimate.theta = normalize_theta(_estimate.theta + update[2]);
}

VertexTimeDiff::VertexTimeDiff(const double &value)
{
    _estimate = value;
    setFixed(false);
}
VertexTimeDiff::VertexTimeDiff()

{
    setToOriginImpl();
    setFixed(false);
}
