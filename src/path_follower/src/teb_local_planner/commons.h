#pragma once
#include <cmath>
#include <vector>

double normalizeAngle(const double &angle);

double normalizeAngle(double &&angle);


struct Point2D
{
    double x;
    double y;
    Point2D() : x(0), y(0) {}
    Point2D(const double &_x, const double &_y) : x(_x), y(_y) {}
    Point2D(double &&_x, double &&_y) : x(_x), y(_y) {}
    
    static double distance(const Point2D &p1, const Point2D &p2);
};


struct Pose2E
{
    double x;
    double y;
    double theta;
    Pose2E() : x(0), y(0), theta(0) {}
    Pose2E(const double &_x, const double &_y, const double &_theta = 0.0) : x(_x), y(_y), theta(_theta) {}
    Pose2E(double &&_x, double &&_y, double &&_theta = 0.0) : x(_x), y(_y), theta(_theta) {}
};

using Pose2Es = std::vector<Pose2E>;
using Point2Ds = std::vector<Point2D>;

double pointToLineDistance(const Point2D &point, const Point2D &line_start, const Point2D &line_end);

Point2D nearestPointOnLine(const Point2D &point, const Point2D &line_start, const Point2D &line_end);

