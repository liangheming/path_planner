#include "commons.h"
Coordinate::Coordinate() : x(0.0f), y(0.0f), theta(0.0f) {}
Coordinate::Coordinate(float x_in, float y_in) : x(x_in), y(y_in), theta(0.0f) {}
Coordinate::Coordinate(float x_in, float y_in, float theta_in) : x(x_in), y(y_in), theta(theta_in) {}

using CoordinateVector = std::vector<Coordinate>;

float normalize_angle(const float &angle)
{
    const float result = fmod(angle + M_PI, 2.0 * M_PI);
    if (result <= 0.0)
        return result + M_PI;
    return result - M_PI;
}
float shortest_angular_distance(float from, float to)
{
    return normalize_angle(to - from);
}
