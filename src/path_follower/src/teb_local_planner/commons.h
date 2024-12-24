#pragma once
#include <cmath>
#include <vector>

double normalizeAngle(const double &angle);

double normalizeAngle(double &&angle);

double average_angle(double theta1, double theta2);

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

    static Pose2E average(const Pose2E &p1, const Pose2E &p2);
};

using Pose2Es = std::vector<Pose2E>;
using Point2Ds = std::vector<Point2D>;

double pointToLineDistance(const Point2D &point, const Point2D &line_start, const Point2D &line_end);

Point2D nearestPointOnLine(const Point2D &point, const Point2D &line_start, const Point2D &line_end);

struct FollowerInfo
{
    double ignore_back_range = 0.5;
    double max_follow_range = 4.5;
    double max_follow_length = 4.0;
    double viapoint_seperation = -0.1;
    double obstacle_behind_ignore_range = 1.5;
    bool allow_init_backwords = false;
    bool overwrite_orientation = false;
    double dt_ref = 0.2;
    double dt_std = 0.1;
    int max_samples = 64;
    int min_samples = 16;

    double max_vel_x = 1.5;
    double max_vel_x_backwards = 0.4;
    double max_vel_y = 0.0;
    double max_acc_x = 1.0;
    double max_acc_y = 0.0;

    double max_vel_theta = 1.0;
    double max_acc_theta = 1.57;
    double min_turning_radius = 0.87;

    double xy_goal_tolerance = 0.2;
    double yaw_goal_tolerance = 0.1;
    bool free_goal_vel = false;
    bool complete_global_plan = true;

    double min_obstacle_dist = 0.1;
    double obstacle_inflation_dist = 0.2;

    int no_outer_iterations = 3;
    int no_inner_iterations = 4;

    double weight_adapt_factor = 1.5;

    double weight_max_vel_x = 1.0;
    double weight_max_vel_y = 1.0;
    double weight_max_vel_theta = 1.0;
    double weight_max_acc_x = 1.0;
    double weight_max_acc_y = 1.0;
    double weight_max_acc_theta = 1.0;

    double weight_forward_drive = 10.0;
    double weight_kinematics_smooth = 1000.0;
    double weight_kinematics_turning_radius = 5.0;
    double weight_shortest_path = 0.1;
    double weight_optimal_time = 2.0;
    double weight_obstacle_collision = 50.0;
    double weight_obstacle_inflation = 0.1;
    double weight_viapoints = 1.0;

};
