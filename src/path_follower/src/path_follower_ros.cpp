#include "path_follower_ros.h"

PathFollowerROS::PathFollowerROS(ros::NodeHandle &nh)
    : _nh(nh), _tf_buffer(), _tf_listener(_tf_buffer), _costmap_2d_ros("follower_costmap", _tf_buffer)
{
    loadParameters();
    initServices();
    initPublishers();
    initSubscribers();
    _robot_model = std::shared_ptr<RobotModel>(RobotModelHelper::createRobotModel(_config.footprint));
    _path_follower = std::make_shared<TebFollower>(&_follower_info, _robot_model.get(), _costmap_2d_ros.getCostmap());
    ROS_INFO("[PathFollowerROS] Initialized");
}
void PathFollowerROS::loadParameters()
{
    ros::NodeHandle collision_nh(_nh, "collision");
    collision_nh.param<std::vector<double>>("footprint", _config.footprint, {0.76, 0.52});
    ROS_INFO("[PathFollowerROS] Footprint Size: %lu", _config.footprint.size());
    ros::NodeHandle follower_nh(_nh, "path_follower");
    follower_nh.param<int>("look_forward_poses_for_control", _follower_info.look_forward_poses_for_control, _follower_info.look_forward_poses_for_control);
    follower_nh.param<double>("force_reinit_new_goal_dist", _follower_info.force_reinit_new_goal_dist, _follower_info.force_reinit_new_goal_dist);
    follower_nh.param<double>("force_reinit_new_goal_angular", _follower_info.force_reinit_new_goal_angular, _follower_info.force_reinit_new_goal_angular);
    follower_nh.param<double>("stop_vel_travel", _follower_info.stop_vel_travel, _follower_info.stop_vel_travel);
    follower_nh.param<double>("stop_vel_rotation", _follower_info.stop_vel_rotation, _follower_info.stop_vel_rotation);
    follower_nh.param<double>("ignore_back_range", _follower_info.ignore_back_range, _follower_info.ignore_back_range);
    follower_nh.param<double>("max_follow_range", _follower_info.max_follow_range, _follower_info.max_follow_range);
    follower_nh.param<double>("max_follow_length", _follower_info.max_follow_length, _follower_info.max_follow_length);
    follower_nh.param<double>("viapoint_seperation", _follower_info.viapoint_seperation, _follower_info.viapoint_seperation);
    follower_nh.param<double>("obstacle_behind_ignore_range", _follower_info.obstacle_behind_ignore_range, _follower_info.obstacle_behind_ignore_range);
    follower_nh.param<double>("obstacle_search_factory", _follower_info.obstacle_search_factory, _follower_info.obstacle_search_factory);
    follower_nh.param<double>("obstacle_collision_dist", _follower_info.obstacle_collision_dist, _follower_info.obstacle_collision_dist);
    follower_nh.param<double>("margin_obstacle_collision", _follower_info.margin_obstacle_collision, _follower_info.margin_obstacle_collision);
    follower_nh.param<double>("obstacle_inflation_dist", _follower_info.obstacle_inflation_dist, _follower_info.obstacle_inflation_dist);
    follower_nh.param<double>("margin_obstacle_inflation", _follower_info.margin_obstacle_inflation, _follower_info.margin_obstacle_inflation);
    follower_nh.param<int>("obstacle_search_number", _follower_info.obstacle_search_number, _follower_info.obstacle_search_number);
    follower_nh.param<bool>("exact_arc_length", _follower_info.exact_arc_length, _follower_info.exact_arc_length);
    follower_nh.param<bool>("allow_init_backwords", _follower_info.allow_init_backwords, _follower_info.allow_init_backwords);
    follower_nh.param<bool>("overwrite_orientation", _follower_info.overwrite_orientation, _follower_info.overwrite_orientation);
    follower_nh.param<double>("dt_ref", _follower_info.dt_ref, _follower_info.dt_ref);
    follower_nh.param<double>("dt_std", _follower_info.dt_std, _follower_info.dt_std);
    follower_nh.param<int>("min_samples", _follower_info.min_samples, _follower_info.min_samples);
    follower_nh.param<int>("max_samples", _follower_info.max_samples, _follower_info.max_samples);
    follower_nh.param<double>("max_vel_forward", _follower_info.max_vel_forward, _follower_info.max_vel_forward);
    follower_nh.param<double>("max_vel_backward", _follower_info.max_vel_backward, _follower_info.max_vel_backward);
    follower_nh.param<double>("max_vel_left", _follower_info.max_vel_left, _follower_info.max_vel_left);
    follower_nh.param<double>("max_vel_right", _follower_info.max_vel_right, _follower_info.max_vel_right);
    follower_nh.param<double>("margin_max_vel_travel", _follower_info.margin_max_vel_travel, _follower_info.margin_max_vel_travel);
    follower_nh.param<double>("max_vel_rotation", _follower_info.max_vel_rotation, _follower_info.max_vel_rotation);
    follower_nh.param<double>("margin_max_vel_rotation", _follower_info.margin_max_vel_rotation, _follower_info.margin_max_vel_rotation);
    follower_nh.param<double>("max_acc_travel", _follower_info.max_acc_travel, _follower_info.max_acc_travel);
    follower_nh.param<double>("margin_max_acc_travel", _follower_info.margin_max_acc_travel, _follower_info.margin_max_acc_travel);
    follower_nh.param<double>("max_acc_rotation", _follower_info.max_acc_rotation, _follower_info.max_acc_rotation);
    follower_nh.param<double>("margin_max_acc_rotation", _follower_info.margin_max_acc_rotation, _follower_info.margin_max_acc_rotation);
    follower_nh.param<double>("min_turning_radius", _follower_info.min_turning_radius, _follower_info.min_turning_radius);
    follower_nh.param<double>("margin_kinematics_turning_radius", _follower_info.margin_kinematics_turning_radius, _follower_info.margin_kinematics_turning_radius);
    follower_nh.param<double>("xy_goal_tolerance", _follower_info.xy_goal_tolerance, _follower_info.xy_goal_tolerance);
    follower_nh.param<double>("yaw_goal_tolerance", _follower_info.yaw_goal_tolerance, _follower_info.yaw_goal_tolerance);
    follower_nh.param<bool>("free_goal_vel", _follower_info.free_goal_vel, _follower_info.free_goal_vel);
    follower_nh.param<bool>("complete_global_plan", _follower_info.complete_global_plan, _follower_info.complete_global_plan);
    follower_nh.param<int>("no_outer_iterations", _follower_info.no_outer_iterations, _follower_info.no_outer_iterations);
    follower_nh.param<int>("no_inner_iterations", _follower_info.no_inner_iterations, _follower_info.no_inner_iterations);
    follower_nh.param<double>("weight_max_vel_travel", _follower_info.weight_max_vel_travel, _follower_info.weight_max_vel_travel);
    follower_nh.param<double>("weight_max_vel_side", _follower_info.weight_max_vel_side, _follower_info.weight_max_vel_side);
    follower_nh.param<double>("weight_max_vel_rotation", _follower_info.weight_max_vel_rotation, _follower_info.weight_max_vel_rotation);

    follower_nh.param<double>("weight_max_acc_travel", _follower_info.weight_max_acc_travel, _follower_info.weight_max_acc_travel);
    follower_nh.param<double>("weight_max_acc_side", _follower_info.weight_max_acc_side, _follower_info.weight_max_acc_side);
    follower_nh.param<double>("weight_max_acc_rotation", _follower_info.weight_max_acc_rotation, _follower_info.weight_max_acc_rotation);

    follower_nh.param<double>("weight_kinematics_forward", _follower_info.weight_kinematics_forward, _follower_info.weight_kinematics_forward);
    follower_nh.param<double>("weight_kinematics_smooth", _follower_info.weight_kinematics_smooth, _follower_info.weight_kinematics_smooth);
    follower_nh.param<double>("weight_kinematics_turning_radius", _follower_info.weight_kinematics_turning_radius, _follower_info.weight_kinematics_turning_radius);
    follower_nh.param<double>("weight_obstacle_collision", _follower_info.weight_obstacle_collision, _follower_info.weight_obstacle_collision);
    follower_nh.param<double>("weight_obstacle_inflation", _follower_info.weight_obstacle_inflation, _follower_info.weight_obstacle_inflation);
    follower_nh.param<double>("weight_optimal_time", _follower_info.weight_optimal_time, _follower_info.weight_optimal_time);
    follower_nh.param<double>("weight_shortest_path", _follower_info.weight_shortest_path, _follower_info.weight_shortest_path);
    follower_nh.param<double>("weight_viapoints", _follower_info.weight_viapoints, _follower_info.weight_viapoints);

    ROS_DEBUG("[PathFollowerROS] look_forward_poses_for_control: %d", _follower_info.look_forward_poses_for_control);
    ROS_DEBUG("[PathFollowerROS] force_reinit_new_goal_dist: %f", _follower_info.force_reinit_new_goal_dist);
    ROS_DEBUG("[PathFollowerROS] force_reinit_new_goal_angular: %f", _follower_info.force_reinit_new_goal_angular);
    ROS_DEBUG("[PathFollowerROS] stop_vel_travel: %f", _follower_info.stop_vel_travel);
    ROS_DEBUG("[PathFollowerROS] stop_vel_rotation: %f", _follower_info.stop_vel_rotation);

    ROS_DEBUG("[PathFollowerROS] ignore_back_range: %f", _follower_info.ignore_back_range);
    ROS_DEBUG("[PathFollowerROS] max_follow_range: %f", _follower_info.max_follow_range);
    ROS_DEBUG("[PathFollowerROS] max_follow_length: %f", _follower_info.max_follow_length);
    ROS_DEBUG("[PathFollowerROS] viapoint_seperation: %f", _follower_info.viapoint_seperation);

    ROS_DEBUG("[PathFollowerROS] obstacle_behind_ignore_range: %f", _follower_info.obstacle_behind_ignore_range);
    ROS_DEBUG("[PathFollowerROS] obstacle_search_factory: %f", _follower_info.obstacle_search_factory);

    ROS_DEBUG("[PathFollowerROS] obstacle_collision_dist: %f", _follower_info.obstacle_collision_dist);
    ROS_DEBUG("[PathFollowerROS] margin_obstacle_collision: %f", _follower_info.margin_obstacle_collision);
    ROS_DEBUG("[PathFollowerROS] obstacle_inflation_dist: %f", _follower_info.obstacle_inflation_dist);
    ROS_DEBUG("[PathFollowerROS] margin_obstacle_inflation: %f", _follower_info.margin_obstacle_inflation);
    ROS_DEBUG("[PathFollowerROS] obstacle_search_number: %d", _follower_info.obstacle_search_number);
    ROS_DEBUG("[PathFollowerROS] exact_arc_length: %d", _follower_info.exact_arc_length);
    ROS_DEBUG("[PathFollowerROS] allow_init_backwords: %d", _follower_info.allow_init_backwords);
    ROS_DEBUG("[PathFollowerROS] overwrite_orientation: %d", _follower_info.overwrite_orientation);
    ROS_DEBUG("[PathFollowerROS] dt_ref: %f", _follower_info.dt_ref);
    ROS_DEBUG("[PathFollowerROS] dt_std: %f", _follower_info.dt_std);
    ROS_DEBUG("[PathFollowerROS] min_samples: %d", _follower_info.min_samples);
    ROS_DEBUG("[PathFollowerROS] max_samples: %d", _follower_info.max_samples);

    ROS_DEBUG("[PathFollowerROS] max_vel_forward: %f", _follower_info.max_vel_forward);
    ROS_DEBUG("[PathFollowerROS] max_vel_backward: %f", _follower_info.max_vel_backward);
    ROS_DEBUG("[PathFollowerROS] max_vel_left: %f", _follower_info.max_vel_left);
    ROS_DEBUG("[PathFollowerROS] max_vel_right: %f", _follower_info.max_vel_right);
    ROS_DEBUG("[PathFollowerROS] margin_max_vel_travel: %f", _follower_info.margin_max_vel_travel);
    ROS_DEBUG("[PathFollowerROS] max_acc_travel: %f", _follower_info.max_acc_travel);
    ROS_DEBUG("[PathFollowerROS] margin_max_acc_travel: %f", _follower_info.margin_max_acc_travel);
    ROS_DEBUG("[PathFollowerROS] max_acc_rotation: %f", _follower_info.max_acc_rotation);
    ROS_DEBUG("[PathFollowerROS] margin_max_acc_rotation: %f", _follower_info.margin_max_acc_rotation);

    ROS_DEBUG("[PathFollowerROS] min_turning_radius: %f", _follower_info.min_turning_radius);
    ROS_DEBUG("[PathFollowerROS] margin_kinematics_turning_radius: %f", _follower_info.margin_kinematics_turning_radius);
    ROS_DEBUG("[PathFollowerROS] xy_goal_tolerance: %f", _follower_info.xy_goal_tolerance);
    ROS_DEBUG("[PathFollowerROS] yaw_goal_tolerance: %f", _follower_info.yaw_goal_tolerance);
    ROS_DEBUG("[PathFollowerROS] free_goal_vel: %d", _follower_info.free_goal_vel);
    ROS_DEBUG("[PathFollowerROS] complete_global_plan: %d", _follower_info.complete_global_plan);
    ROS_DEBUG("[PathFollowerROS] no_inner_iterations: %d", _follower_info.no_inner_iterations);
    ROS_DEBUG("[PathFollowerROS] no_outer_iterations: %d", _follower_info.no_outer_iterations);
    ROS_DEBUG("[PathFollowerROS] weight_max_vel_travel: %f", _follower_info.weight_max_vel_travel);
    ROS_DEBUG("[PathFollowerROS] weight_max_vel_side: %f", _follower_info.weight_max_vel_side);
    ROS_DEBUG("[PathFollowerROS] weight_max_vel_rotation: %f", _follower_info.weight_max_vel_rotation);
    ROS_DEBUG("[PathFollowerROS] weight_max_acc_travel: %f", _follower_info.weight_max_acc_travel);
    ROS_DEBUG("[PathFollowerROS] weight_max_acc_side: %f", _follower_info.weight_max_acc_side);
    ROS_DEBUG("[PathFollowerROS] weight_max_acc_rotation: %f", _follower_info.weight_max_acc_rotation);

    ROS_DEBUG("[PathFollowerROS] weight_kinematics_forward: %f", _follower_info.weight_kinematics_forward);
    ROS_DEBUG("[PathFollowerROS] weight_kinematics_smooth: %f", _follower_info.weight_kinematics_smooth);
    ROS_DEBUG("[PathFollowerROS] weight_kinematics_turning_radius: %f", _follower_info.weight_kinematics_turning_radius);
    ROS_DEBUG("[PathFollowerROS] weight_obstacle_collision: %f", _follower_info.weight_obstacle_collision);
    ROS_DEBUG("[PathFollowerROS] weight_obstacle_inflation: %f", _follower_info.weight_obstacle_inflation);
    ROS_DEBUG("[PathFollowerROS] weight_optimal_time: %f", _follower_info.weight_optimal_time);
    ROS_DEBUG("[PathFollowerROS] weight_shortest_path: %f", _follower_info.weight_shortest_path);
    ROS_DEBUG("[PathFollowerROS] weight_viapoints: %f", _follower_info.weight_viapoints);
}
void PathFollowerROS::initServices()
{
}
void PathFollowerROS::initPublishers()
{
    _followed_path_pub = _nh.advertise<nav_msgs::Path>("path", 1);
    _candidate_obstacle_pub = _nh.advertise<sensor_msgs::PointCloud>("candidate_obstacles", 1);
    _affected_obstacle_pub = _nh.advertise<sensor_msgs::PointCloud>("affected_obstacles", 1);
}
void PathFollowerROS::initSubscribers()
{
    _initial_pose_sub = _nh.subscribe("/initialpose", 1, &PathFollowerROS::initialPoseCallback, this);
    _global_path_sub = _nh.subscribe("/path_planner/path", 1, &PathFollowerROS::globalPathCallback, this);
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

void PathFollowerROS::globalPathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    Point2EVec path;
    for (auto &p : msg->poses)
    {
        path.emplace_back(p.pose.position.x, p.pose.position.y, tf::getYaw(p.pose.orientation));
    }
    _path_follower->setGlobalPath(path);
    ROS_INFO("[PathFollowerROS] Received path with %lu poses", path.size());
    Velocity robot_vel;
    robot_vel.setZero();
    Velocity robot_cmd;
    std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();
    FollowResult result = _path_follower->makePlane(_initial_pose, robot_vel, robot_cmd);
    std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
    int64_t ms_take = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();
    ROS_INFO("[PathFollowerROS] Follower time: %lu ms", ms_take);
    ROS_INFO("[PathFollowerROS] cmd vel: %f | %f | %f", robot_cmd.x, robot_cmd.y, robot_cmd.theta);

    if (result == FollowResult::SUCCESS)
    {
        ROS_INFO("[PathFollowerROS] Follower success");
        publishFollowedPath();
        publishCandidateObstacles();
        publishAffectedObstacles();
    }
}

void PathFollowerROS::publishFollowedPath()
{
    if (_followed_path_pub.getNumSubscribers() < 1)
        return;
    Point2EVec followed_path = _path_follower->mutableOptimizer().mutableTrajectory();
    if (followed_path.size() > 0)
    {
        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();
        for (auto &p : followed_path)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = p.x;
            pose.pose.position.y = p.y;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(p.theta);
            path.poses.push_back(pose);
        }
        _followed_path_pub.publish(path);
    }
}

void PathFollowerROS::publishCandidateObstacles()
{
    if (_candidate_obstacle_pub.getNumSubscribers() < 1)
        return;
    Point2DVec obstacles = _path_follower->mutableOptimizer().mutableCanidateObstacles();
    if (obstacles.size() > 0)
    {
        sensor_msgs::PointCloud cloud;
        cloud.header.frame_id = "map";
        cloud.header.stamp = ros::Time::now();
        for (auto &p : obstacles)
        {
            geometry_msgs::Point32 point;
            point.x = p.x;
            point.y = p.y;
            point.z = 0.0;

            cloud.points.push_back(point);
        }
        _candidate_obstacle_pub.publish(cloud);
    }
}

void PathFollowerROS::publishAffectedObstacles()
{
    if (_affected_obstacle_pub.getNumSubscribers() < 1)
        return;
    std::vector<Point2D *> obstacles = _path_follower->mutableOptimizer().mutableAffectedObstacles();
    if (obstacles.empty())
        return;
    std::set<Point2D *> unique_obstacles;
    for (auto &p : obstacles)
    {
        unique_obstacles.insert(p);
    }
    sensor_msgs::PointCloud cloud;
    cloud.header.frame_id = "map";
    cloud.header.stamp = ros::Time::now();
    for (auto &p : unique_obstacles)
    {
        geometry_msgs::Point32 point;
        point.x = p->x;
        point.y = p->y;
        point.z = 0.0;

        cloud.points.push_back(point);
    }
    _affected_obstacle_pub.publish(cloud);
}