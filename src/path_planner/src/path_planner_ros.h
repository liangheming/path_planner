#pragma once

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "planner/smoother.h"
#include "planner/astar_hybrid.h"
#include "planner/collision_checker.h"

struct NodeConfig
{
    bool collision_hole;
    std::vector<float> footprint;
    SearchInfo search_info;
    SmoothInfo smooth_info;
};

class PathPlannerROS
{
public:
    PathPlannerROS(ros::NodeHandle &nh);
    void loadParameters();

    void initSubscribers();

    void initPublishers();

    void initServices();

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

    void goalSetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

private:
    NodeConfig _node_config;
    ros::NodeHandle _nh;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    costmap_2d::Costmap2DROS _costmap_2d_ros;
    CollisionChecker _collision_checker;
    ros::Publisher _path_pub;
    std::shared_ptr<AstarHybrid> _astar;
    std::shared_ptr<Smoother> _smoother;

    ros::Subscriber _initial_pose_sub;
    ros::Subscriber _goal_pose_sub;

    bool _is_start_set;
    geometry_msgs::Pose _temp_start_pose;
    geometry_msgs::Pose _temp_goal_pose;
};