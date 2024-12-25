#pragma once
#include "commons.h"
#include "teb_optimizer.h"
#include <iostream>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

class TebLocalPlanner
{
public:
    TebLocalPlanner(FollowerInfo *follower_info,costmap_2d::Costmap2D* costmap);
    void setRobotVel(const Pose2E &vel) { _current_vel = vel; }
    void setRobotPose(Pose2E &pose) { _current_pose = pose; }
    void setGlobalPath(Pose2Es &global_path) { _global_path = global_path; }
    Pose2E &mutableRobotVel() { return _current_vel; }
    Pose2E &mutableRobotPose() { return _current_pose; }
    Point2Ds &mutableObstacles() { return _obstacles; }
    Pose2Es &mutableGlobalPath() { return _global_path; }

    Pose2E getLocalTwists();

private:
    FollowerInfo *_follower_info;
    Pose2E _current_pose;
    Pose2E _current_vel;
    Pose2Es _global_path;
    Pose2Es _local_path;
    Point2Ds _via_points;
    Point2Ds _obstacles;
    costmap_2d::Costmap2D *_costmap;
    TebOptimizer _optimizer;


    void pruneGlobalPath();

    void generateLocalPath();

    void generateViaPoints();

    void generateObstacles();
};