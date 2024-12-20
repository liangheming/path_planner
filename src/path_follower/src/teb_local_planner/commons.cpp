#include "commons.h"

double Point2D::distance(const Point2D &p1, const Point2D &p2)
{
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

double normalizeAngle(const double &angle)
{
    double result = fmod(angle + M_PI, 2 * M_PI);
    if (result <= 0)
        return result + M_PI;
    return result - M_PI;
}

double normalizeAngle(double &&angle)
{
    return normalizeAngle(angle);
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