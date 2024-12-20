
#include <iostream>
#include <memory>
#include "teb_local_planner/commons.h"
#include "teb_local_planner/robot_model.h"
int main(int argc, char **argv)
{
    std::shared_ptr<RobotModel> robot_model = std::make_shared<CircleRobotModel>(0.5);
    double distance = robot_model->distanceToPoint(Pose2E(0, 0, 0), Point2D(0, 0.1));
    std::cout << distance << std::endl;

    std::vector<Point2D> vertices;
    vertices.emplace_back(0.3, 0.2);
    vertices.emplace_back(0.3, -0.2);
    vertices.emplace_back(-0.3, -0.2);
    vertices.emplace_back(-0.3, 0.2);

    robot_model.reset(new PolygonRobotModel(vertices));
    distance = robot_model->distanceToPoint(Pose2E(0, 0, 0), Point2D(0.3, 0.3));
    std::cout << distance << std::endl;

    return 0;
};