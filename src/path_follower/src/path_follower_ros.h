#pragma once

#include <tf/tf.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <costmap_2d/costmap_2d_ros.h>

#include "teb_path_follower/teb_follower.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>

struct NodeConfig
{
    std::vector<double> footprint;
};

class PathFollowerROS
{
public:
    PathFollowerROS(ros::NodeHandle &nh);
    void loadParameters();
    void initServices();
    void initPublishers();
    void initSubscribers();

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

    void globalPathCallback(const nav_msgs::Path::ConstPtr &msg);

    void publishFollowedPath();

    void publishCandidateObstacles();

    void publishAffectedObstacles();

private:
    NodeConfig _config;
    ros::NodeHandle _nh;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    tf2_ros::TransformBroadcaster _tf_broadcaster;
    costmap_2d::Costmap2DROS _costmap_2d_ros;
    ros::Subscriber _global_path_sub;
    ros::Subscriber _initial_pose_sub;
    ros::Publisher _candidate_obstacle_pub;
    ros::Publisher _affected_obstacle_pub;
    ros::Publisher _followed_path_pub;
    Point2E _initial_pose;
    FollowerInfo _follower_info;
    std::shared_ptr<TebFollower> _path_follower;
    std::shared_ptr<RobotModel> _robot_model;
};