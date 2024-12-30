#pragma once
#include <cmath>
#include <cassert>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include "../third_party/nanoflann.hpp"

/**
 * @brief Normalize the angle to [-pi, pi]
 */
template <typename T>
inline T normalize_theta(const T &angel)
{
    const T PI = static_cast<T>(M_PI);
    T result = fmod(angel + PI, 2 * PI);
    if (result <= 0)
        return result + PI;
    return result - PI;
}

template <typename T>
inline T average_angle(const T &theta1, const T &theta2)
{
    T x = std::cos(theta1) + std::cos(theta2);
    T y = std::sin(theta1) + std::sin(theta2);
    if (x == 0 && y == 0)
        return 0;
    else
        return std::atan2(y, x);
}

template <typename T>
inline T penaltyBoundedValue(const T &var, const T &a, const T &epsilon)
{
    if (var < -a + epsilon)
        return (-a + epsilon - var);
    if (var <= a - epsilon)
        return 0.;
    else
        return (var - (a - epsilon));
}

template <typename T>
inline T penaltyBoundedValue(const T &var, const T &a, const T &b, const T &epsilon)
{
    if (var < a + epsilon)
        return ((a + epsilon) - var);
    if (var <= b - epsilon)
        return 0.0;
    else
        return (var - (b - epsilon));
}
template <typename T>
inline T penaltyBelow(const T &var, const T &a, const T &epsilon)
{
    {
        if (var > a + epsilon)
            return 0.0;
        else
            return a + epsilon - var;
    }
}

inline double soft_sign(const double &x, const double &k = 1.0)
{
    return k * x / (1.0 + std::abs(k * x));
}

inline double sign_nz(const double &x)
{
    return x > 0.0 ? 1.0 : -1.0;
}

class Point2D
{
public:
    Point2D() : x(0), y(0) {}
    Point2D(double &&x_, double &&y_) : x(x_), y(y_) {}
    Point2D(const double &x_, const double &y_) : x(x_), y(y_) {}

    static double distance(const Point2D &p1, const Point2D &p2);

    static Point2D delta(const Point2D &p1, const Point2D &p2);
    Eigen::Vector2d toVector() const { return Eigen::Vector2d(x, y); }

    double x;
    double y;
};

class Point2E
{
public:
    Point2E() : x(0), y(0), theta(0) {}
    Point2E(double &&x_, double &&y_, double &&theta_) : x(x_), y(y_), theta(theta_) {}
    Point2E(const double &x_, const double &y_, const double &theta_) : x(x_), y(y_), theta(theta_) {}

    void setZero();

    static double distance(const Point2E &p1, const Point2E &p2);

    static Point2E delta(const Point2E &p1, const Point2E &p2);

    static Point2E average(const Point2E &p1, const Point2E &p2);
    Eigen::Vector3d toVector() const { return Eigen::Vector3d(x, y, theta); }
    inline Eigen::Vector2d translation() const { return Eigen::Vector2d(x, y); }
    inline Eigen::Vector2d direction() const { return Eigen::Vector2d(cos(theta), sin(theta)); }
    double angle() const { return theta; }

    double x;
    double y;
    double theta;
};

using Velocity = Point2E;

using Point2DVec = std::vector<Point2D>;

using Point2EVec = std::vector<Point2E>;

double pointToLineDistance(const Point2D &point, const Point2D &line_start, const Point2D &line_end);

Point2D nearestPointOnLine(const Point2D &point, const Point2D &line_start, const Point2D &line_end);

class Vertex2E : public g2o::BaseVertex<3, Point2E>
{
public:
    Vertex2E()
    {
        setToOriginImpl();
        setFixed(false);
    }
    Vertex2E(const Point2E &p2e);

    Vertex2E(double &&x, double &&y, double &&theta);

    Vertex2E(const double &x, const double &y, const double &theta);
    inline Eigen::Vector2d translation() { return Eigen::Vector2d(_estimate.x, _estimate.y); }
    inline double theta() { return _estimate.theta; }
    virtual void setToOriginImpl() override { _estimate.setZero(); }

    virtual void oplusImpl(const double *update) override;
    virtual bool read(std::istream &is) override
    {
        is >> _estimate.x >> _estimate.y >> _estimate.theta;
        return true;
    }
    virtual bool write(std::ostream &os) const override
    {
        os << _estimate.x << " " << _estimate.y << " " << _estimate.theta;
        return os.good();
    }
};

class VertexTimeDiff : public g2o::BaseVertex<1, double>
{
public:
    VertexTimeDiff();

    VertexTimeDiff(const double &value);
    virtual void setToOriginImpl() override
    {
        _estimate = 0.0;
    }
    virtual void oplusImpl(const double *update) override
    {
        _estimate += *update;
        if (_estimate <= 0.0)
            _estimate = 1e-6;
    }
    virtual bool read(std::istream &is) override
    {
        is >> _estimate;
        return true;
    }
    virtual bool write(std::ostream &os) const override
    {
        os << _estimate;
        return os.good();
    }
};

class PointCloud2D
{
public:
    PointCloud2D() {}
    void addPoint(const Point2D &p) { _points.push_back(p); }
    void addPoint(double x, double y) { _points.emplace_back(x, y); }
    void clear() { _points.clear(); }
    Point2DVec &points() { return _points; }
    inline size_t kdtree_get_point_count() const { return _points.size(); }
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const { return dim == 0 ? _points[idx].x : _points[idx].y; }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX & /* bb */) const { return false; }

private:
    Point2DVec _points;
};
using KDTree2D = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloud2D>, PointCloud2D, 2>;

struct FollowerInfo
{

    int look_forward_poses_for_control = 1;
    double force_reinit_new_goal_dist = 0.1;
    double force_reinit_new_goal_angular = 0.1;

    double stop_vel_travel = 0.1;
    double stop_vel_rotation = 0.1;

    double ignore_back_range = 0.5;
    double max_follow_range = 4.5;
    double max_follow_length = 5.0;

    double viapoint_seperation = -0.1;
    double obstacle_behind_ignore_range = 1.5;
    double obstacle_search_factory = 1.5;

    double obstacle_collision_dist = 0.1;
    double margin_obstacle_collision = 0.05;
    double obstacle_inflation_dist = 0.2;
    double margin_obstacle_inflation = 0.1;

    int obstacle_search_number = 5;

    bool exact_arc_length = false;
    bool allow_init_backwords = false;
    bool overwrite_orientation = false;

    double dt_ref = 0.2;
    double dt_std = 0.1;

    int min_samples = 16;
    int max_samples = 64;

    double max_vel_forward = 1.5;
    double max_vel_backward = 0.4;
    double max_vel_left = 0.0;
    double max_vel_right = 0.0;
    double margin_max_vel_travel = 0.1;

    double max_vel_rotation = 1.0;
    double margin_max_vel_rotation = 0.1;

    double max_acc_travel = 1.0;
    double max_acc_side = 0.0;
    double margin_max_acc_travel = 0.1;

    double max_acc_rotation = 1.57;
    double margin_max_acc_rotation = 0.1;

    double min_turning_radius = 0.87;
    double margin_kinematics_turning_radius = 0.1;

    double xy_goal_tolerance = 0.2;
    double yaw_goal_tolerance = 0.1;

    bool free_goal_vel = false;
    bool complete_global_plan = true;

    int no_outer_iterations = 3;
    int no_inner_iterations = 4;

    double weight_max_vel_travel = 1.0;
    double weight_max_vel_side = 1.0;

    double weight_max_acc_travel = 1.0;
    double weight_max_acc_side = 1.0;

    double weight_max_vel_rotation = 1.0;
    double weight_max_acc_rotation = 1.0;

    double weight_kinematics_forward = 10.0;
    double weight_kinematics_smooth = 1000.0;
    double weight_kinematics_turning_radius = 5.0;

    double weight_shortest_path = 0.1;
    double weight_optimal_time = 2.0;
    double weight_obstacle_collision = 50.0;
    double weight_obstacle_inflation = 1.0;

    double weight_viapoints = 1.0;
};