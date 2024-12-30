#pragma once
#include "commons.h"
#include "teb_optimizer.h"
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
enum class FollowResult
{
    EMPTY_PLAN,
    GOAL_REACH,
    SUCCESS,
    FAILED,
    COLLSION,
    OSCILLATION
};

class TebFollower
{
public:
    TebFollower(FollowerInfo *info, RobotModel *robot);

    TebFollower(FollowerInfo *info, RobotModel *robot, costmap_2d::Costmap2D *costmap);
    void setCostMap(costmap_2d::Costmap2D *costmap) { _costmap = costmap; }

    void setGlobalPath(std::vector<Point2E> &path);

    FollowResult makePlane(const Point2E &robot_pose, const Velocity &robot_vel, Velocity &cmd_vel);
    TebOptimizer &mutableOptimizer() { return _optimizer; }
    bool isAlmostStop() { return abs(_robot_vel.x) <= _info->stop_vel_travel && abs(_robot_vel.y) <= _info->stop_vel_travel; }

private:
    void pruneGlobalPath();

    int generatePathToFollow();

    void addViaPoints();

    void addObstacles();

    void overwriteOrientation(const int &global_index);

    bool getVelocityCmd(Velocity &cmd_vel);

    FollowerInfo *_info;
    RobotModel *_robot;
    Point2E _robot_pose;
    Velocity _robot_vel;
    std::vector<Point2E> _path_of_global;
    std::vector<Point2E> _path_to_follow;
    costmap_2d::Costmap2D *_costmap;
    TebOptimizer _optimizer;
};