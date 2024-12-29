#pragma once
#include "commons.h"
#include "teb_optimizer.h"
#include <costmap_2d/costmap_2d.h>
class TebFollower
{
public:
    TebFollower(FollowerInfo *info, RobotModel *robot);
    
    TebFollower(FollowerInfo *info, RobotModel *robot, costmap_2d::Costmap2D *costmap);
    void setCostMap(costmap_2d::Costmap2D *costmap) { _costmap = costmap; }

    void setGlobalPath(std::vector<Point2E> &path);

private:
    FollowerInfo *_info;
    RobotModel *_robot;
    Point2E _robot_pose;
    Velocity _robot_vel;
    std::vector<Point2E> _path_of_global;
    std::vector<Point2E> _path_to_follow;
    costmap_2d::Costmap2D *_costmap;
    TebOptimizer _optimizer;
};