#pragma once
#include <cmath>
#include <vector>

struct Coordinate
{
    float x;
    float y;
    float theta;
    Coordinate();
    Coordinate(float x_in, float y_in);
    Coordinate(float x_in, float y_in, float theta_in);
};
using Motion = Coordinate;
using Motions = std::vector<Motion>;
using Coordinates = std::vector<Coordinate>;
using Pose2d = Coordinate;
using Pose2ds = std::vector<Pose2d>;
enum class MotionType
{
    None,
    Dubins,
    ReedsShepp,
};

struct SearchInfo
{
    unsigned int size_x{0};
    unsigned int size_y{0};
    unsigned int size_theta{72};
    float tolerance{0.5f};
    float tolerance_in_cell{5.0f};
    float minimum_turning_radius{0.84f};
    float minimum_turning_radius_in_cell{8.0f};
    float change_penalty{0.0f};
    float reverse_penalty{2.0f};
    float cost_penalty{2.1f};
    float non_straight_penalty{1.2f};
    float analytic_expansion_ratio{4.0f};
    unsigned int max_iterations = {100000};
    unsigned int max_on_approach_iterations{1000};
    MotionType motion_type = MotionType::Dubins;
    double max_sec{0.5};
    int64_t max_millsec{500};
};

struct SmoothInfo
{
    float minimum_turning_radius;
    MotionType motion_type;
    bool is_holonomic{false};
    unsigned int max_iter{1000};
    double tolerance = 1e-4;
    double w_smooth = 0.3;
    double w_data = 0.2;
};

float normalize_angle(const float &angle);
float shortest_angular_distance(float from, float to);

double normalize_angle(const double &angle);
double shortest_angular_distance(double from, double to);
