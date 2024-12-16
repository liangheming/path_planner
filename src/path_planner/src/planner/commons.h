#pragma once
#include <vector>
#include <cmath>

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
    unsigned int size_x;
    unsigned int size_y;
    unsigned int size_theta;
    float tolerance;
    float minimum_turning_radius;
    float change_penalty;
    float reverse_penalty;
    float cost_penalty;
    float non_straight_penalty;
    float analytic_expansion_ratio;
    unsigned int max_iterations;
    unsigned int max_on_approach_iterations;
    MotionType motion_type;
};
float normalize_angle(const float &angle);

float shortest_angular_distance(float from, float to);

