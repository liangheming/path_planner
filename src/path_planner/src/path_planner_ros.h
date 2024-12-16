#pragma once
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>

#include "planner/collision_checker.h"
#include "planner/a_star.h"
#include "planner/smoother.h"
#include <nav_msgs/Path.h>
#include <chrono>

struct NodeConfig
{
    std::vector<float> footprint_boundary;
};

class PathPlannerROS
{
public:
    PathPlannerROS(ros::NodeHandle &nh);
    void loadParameters();
    void initSubscribers();
    void initPublishers();
    void initServices();
    ~PathPlannerROS();

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

    void goalSetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // void tempTest();

private:
    NodeConfig _node_config;
    ros::NodeHandle _nh;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    costmap_2d::Costmap2DROS _costmap_ros;
    bool _is_start_set;
    geometry_msgs::Pose _temp_start_pose;
    geometry_msgs::Pose _temp_goal_pose;

    ros::Subscriber _initial_pose_sub;
    ros::Subscriber _goal_pose_sub;
    SearchInfo _search_info;
    CollisionChecker _collision_checker;
    std::shared_ptr<AstarAlgorithm> _astar;
    ros::Publisher _path_pub;
    ros::Publisher _smooth_path_pub;
    SmoothInfo _smooth_info;

    std::shared_ptr<Smoother> _path_smoother;
};