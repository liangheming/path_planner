#pragma once
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include "teb_local_planner/commons.h"
#include "teb_local_planner/robot_model.h"
#include "teb_local_planner/teb_local_planner.h"

#include <sensor_msgs/PointCloud.h>

struct NodeConfig
{
    std::vector<double> footprint;
};

class PathFollowerROS
{
public:
    PathFollowerROS(ros::NodeHandle nh);
    void loadParameters();
    void initPublishers();
    void initSubscribers();
    void initServices();
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void pathGetCallback(const nav_msgs::Path::ConstPtr &msg);

private:
    NodeConfig _config;
    ros::NodeHandle _nh;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    tf2_ros::TransformBroadcaster _tf_broadcaster;
    costmap_2d::Costmap2DROS _costmap_2d_ros;
    ros::Subscriber _path_sub;
    ros::Subscriber _initial_pose_sub;
    ros::Publisher _obstacle_pub;
    Pose2E _initial_pose;
    FollowerInfo _follower_info;
    std::shared_ptr<TebLocalPlanner> _path_follower;
};