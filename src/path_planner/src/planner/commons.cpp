#include "commons.h"
Coordinate::Coordinate() : x(0.0f), y(0.0f), theta(0.0f) {}
Coordinate::Coordinate(float x_in, float y_in) : x(x_in), y(y_in), theta(0.0f) {}
Coordinate::Coordinate(float x_in, float y_in, float theta_in) : x(x_in), y(y_in), theta(theta_in) {}

using CoordinateVector = std::vector<Coordinate>;