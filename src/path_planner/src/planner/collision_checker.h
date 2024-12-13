#pragma once
#include "commons.h"
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

class CollisionChecker
{
public:
    CollisionChecker() : _costmap(nullptr) {}
    void setCostMap(costmap_2d::Costmap2D *costmap) { _costmap = costmap; }
    void initBoundary(float radius);
    void initBoundary(float width, float height);
    void initBoundary(float x_min, float y_min, float x_max, float y_max);

    void initBoundary(std::vector<float> &boundary);
    
    void printBoundary();

    bool checkCollision(const float &x, const float &y, const float &theta);

    float getFootprintCost(const float &x, const float &y, const float &theta);

    bool checkAndGetFootprintCost(const float &x, const float &y, const float &theta, float &cost);
    costmap_2d::Costmap2D *getCostMap() { return _costmap; }

private:
    Coordinates _points;
    costmap_2d::Costmap2D *_costmap;
};