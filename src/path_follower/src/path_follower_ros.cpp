#include "path_follower_ros.h"

PathFollowerROS::PathFollowerROS(ros::NodeHandle nh) : _nh(nh), _tf_buffer(), _tf_listener(_tf_buffer), _costmap_2d_ros("follower_costmap", _tf_buffer)
{
    loadParameters();
    initPublishers();
    initSubscribers();
    initServices();
    _robot_model = std::shared_ptr<RobotModel>(RobotModelHelper::createRobotModel(_config.footprint));
    _path_follower = std::make_unique<TebLocalPlanner>(&_follower_info, _costmap_2d_ros.getCostmap(), _robot_model.get());
    ROS_DEBUG("PathFollowerROS initialized");
}
void PathFollowerROS::loadParameters()
{

    _nh.param<std::vector<double>>("footprint", _config.footprint, {0.76, 0.52});
    ROS_INFO("[PathFollowerROS] Footprint Size: %lu", _config.footprint.size());
}
void PathFollowerROS::initPublishers()
{
    _obstacle_pub = _nh.advertise<sensor_msgs::PointCloud>("/obstacles", 1);
    _path_pub = _nh.advertise<nav_msgs::Path>("/path_follower/path", 1);
}
void PathFollowerROS::initSubscribers()
{
    ROS_DEBUG("[PathPlannerROS] Initializing subscribers");
    _initial_pose_sub = _nh.subscribe("/initialpose", 1, &PathFollowerROS::initialPoseCallback, this);
    _path_sub = _nh.subscribe("/path_planner/path", 1, &PathFollowerROS::pathGetCallback, this);
}
void PathFollowerROS::initServices()
{
}

void PathFollowerROS::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    _initial_pose.x = msg->pose.pose.position.x;
    _initial_pose.y = msg->pose.pose.position.y;
    _initial_pose.theta = tf::getYaw(msg->pose.pose.orientation);
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.frame_id = "map";
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.child_frame_id = "move_base";
    tf_msg.transform.translation.x = msg->pose.pose.position.x;
    tf_msg.transform.translation.y = msg->pose.pose.position.y;
    tf_msg.transform.translation.z = msg->pose.pose.position.z;
    tf_msg.transform.rotation = msg->pose.pose.orientation;
    _tf_broadcaster.sendTransform(tf_msg);
}
void PathFollowerROS::pathGetCallback(const nav_msgs::Path::ConstPtr &msg)
{
    Pose2Es path;
    for (auto &pose : msg->poses)
    {
        path.emplace_back(pose.pose.position.x, pose.pose.position.y, tf::getYaw(pose.pose.orientation));
    }
    ROS_INFO("[PathFollowerROS] Received path with %lu poses", path.size());
    _path_follower->setRobotPose(_initial_pose);
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    _path_follower->setGlobalPath(path);
    _path_follower->getLocalTwists();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    ROS_INFO("[PathFollowerROS] PathFollower took %lu ms", ms.count());

    Point2Ds obstacles = _path_follower->mutableObstacles();
    sensor_msgs::PointCloud obstacle_msg;
    obstacle_msg.header.frame_id = "map";
    obstacle_msg.header.stamp = ros::Time::now();
    for (auto &obstacle : obstacles)
    {
        geometry_msgs::Point32 point;
        point.x = obstacle.x;
        point.y = obstacle.y;
        point.z = 0.0;
        obstacle_msg.points.push_back(point);
    }
    _obstacle_pub.publish(obstacle_msg);

    Pose2Es refine_local_path = _path_follower->mutableTrajectory();
    if (refine_local_path.size())
    {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        for (auto &pose : refine_local_path)
        {
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.frame_id = "map";
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.pose.position.x = pose.x;
            pose_msg.pose.position.y = pose.y;
            pose_msg.pose.position.z = 0.0;
            pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta);
            path_msg.poses.push_back(pose_msg);
        }
        _path_pub.publish(path_msg);
    }
}
