#include "commons.h"
Coordinate::Coordinate() : x(0.0f), y(0.0f), theta(0.0f) {}
Coordinate::Coordinate(float x_in, float y_in) : x(x_in), y(y_in), theta(0.0f) {}
Coordinate::Coordinate(float x_in, float y_in, float theta_in) : x(x_in), y(y_in), theta(theta_in) {}

float normalize_angle(const float &angle)
{
    const float result = std::fmod(angle + static_cast<float>(M_PI), 2.0f * static_cast<float>(M_PI));
    if (result <= 0.0f)
        return result + static_cast<float>(M_PI);
    return result - static_cast<float>(M_PI);
}
float shortest_angular_distance(float from, float to)
{
    return normalize_angle(to - from);
}

double normalize_angle(const double &angle)
{
    const double result = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (result <= 0.0)
        return result + M_PI;
    return result - M_PI;
}
double shortest_angular_distance(double from, double to)
{
    return normalize_angle(to - from);
}
