#include "path_planner_ros.h"
PathPlannerROS::PathPlannerROS(ros::NodeHandle &nh) : _nh(nh), _tf_buffer(), _tf_listener(_tf_buffer), _costmap_2d_ros("planner_costmap", _tf_buffer)
{
    loadParameters();
    initPublishers();
    initSubscribers();
    initServices();
    _collision_checker.setCostMap(_costmap_2d_ros.getCostmap());
    _collision_checker.setHole(_node_config.collision_hole);
    _collision_checker.initBoundary(_node_config.footprint);
    _astar = std::make_shared<AstarHybrid>(&_node_config.search_info, &_collision_checker);
    _smoother = std::make_shared<Smoother>(&_node_config.smooth_info, &_collision_checker);

    _is_start_set = false;
}

void PathPlannerROS::loadParameters()
{
    ROS_DEBUG("[PathPlannerROS] Loading parameters");
    ros::NodeHandle collision_handle(_nh, "collision");
    collision_handle.param<bool>("hole", _node_config.collision_hole, false);
    collision_handle.param<std::vector<float>>("footprint", _node_config.footprint, {1.0f});
    ros::NodeHandle path_planner_handle(_nh, "path_planner");
    path_planner_handle.param<float>("minimum_turning_radius", _node_config.search_info.minimum_turning_radius, 0.5f);
    _node_config.smooth_info.minimum_turning_radius = _node_config.search_info.minimum_turning_radius;
    path_planner_handle.param<float>("cost_penalty", _node_config.search_info.cost_penalty, 2.0f);
    path_planner_handle.param<float>("change_penalty", _node_config.search_info.change_penalty, 0.0f);
    path_planner_handle.param<float>("non_straight_penalty", _node_config.search_info.non_straight_penalty, 1.2f);
    path_planner_handle.param<float>("reverse_penalty", _node_config.search_info.reverse_penalty, 2.1f);
    path_planner_handle.param<float>("analytic_expansion_ratio", _node_config.search_info.analytic_expansion_ratio, 4.0f);

    int max_iterations, max_on_approach_iterations, angle_quantization_bins;
    path_planner_handle.param<int>("angle_quantization_bins", angle_quantization_bins, 72);
    path_planner_handle.param<int>("max_iterations", max_iterations, 100000);
    path_planner_handle.param<int>("max_on_approach_iterations", max_on_approach_iterations, 1000);
    path_planner_handle.param<float>("tolerance", _node_config.search_info.tolerance, 0.5f);
    path_planner_handle.param<double>("max_sec", _node_config.search_info.max_sec, 0.5);
    _node_config.search_info.max_millsec = (unsigned int)(_node_config.search_info.max_sec * 1000);
    _node_config.search_info.max_iterations = max_iterations;
    _node_config.search_info.max_on_approach_iterations = max_on_approach_iterations;
    _node_config.search_info.size_theta = angle_quantization_bins;

    std::string motion_type;
    path_planner_handle.param<std::string>("motion_type", motion_type, "ReedsShepp");
    if (motion_type == "Dubins")
        _node_config.search_info.motion_type = MotionType::Dubins;
    else if (motion_type == "ReedsShepp")
        _node_config.search_info.motion_type = MotionType::ReedsShepp;
    else
    {
        ROS_ERROR("[PathPlannerROS] Invalid motion type: %s", motion_type.c_str());
        _node_config.search_info.motion_type = MotionType::None;
    }
    _node_config.smooth_info.motion_type = _node_config.search_info.motion_type;

    ros::NodeHandle smoother_handle(_nh, "smoother");
    smoother_handle.param<bool>("is_holonomic", _node_config.smooth_info.is_holonomic, false);
    smoother_handle.param<double>("w_data", _node_config.smooth_info.w_data, 0.2);
    smoother_handle.param<double>("w_smooth", _node_config.smooth_info.w_smooth, 0.3);
    smoother_handle.param<double>("tolerance", _node_config.smooth_info.tolerance, 1e-4);
    smoother_handle.param<int>("max_iterations", max_iterations, 3000);
    _node_config.smooth_info.max_iter = max_iterations;
    ROS_DEBUG("[PathPlannerROS] ==========Parameters collision==========");
    ROS_DEBUG("[PathPlannerROS] footprint size: %lu", _node_config.footprint.size());
    ROS_DEBUG("[PathPlannerROS] hole: %s", _node_config.collision_hole ? "true" : "false");
    ROS_DEBUG("[PathPlannerROS] ==========Parameters path_planner==========");
    ROS_DEBUG("[PathPlannerROS] minimum_turning_radius: %f", _node_config.search_info.minimum_turning_radius);
    ROS_DEBUG("[PathPlannerROS] cost_penalty: %f", _node_config.search_info.cost_penalty);
    ROS_DEBUG("[PathPlannerROS] change_penalty: %f", _node_config.search_info.change_penalty);
    ROS_DEBUG("[PathPlannerROS] non_straight_penalty: %f", _node_config.search_info.non_straight_penalty);
    ROS_DEBUG("[PathPlannerROS] reverse_penalty: %f", _node_config.search_info.reverse_penalty);
    ROS_DEBUG("[PathPlannerROS] analytic_expansion_ratio: %f", _node_config.search_info.analytic_expansion_ratio);
    ROS_DEBUG("[PathPlannerROS] max_iterations: %u", _node_config.search_info.max_iterations);
    ROS_DEBUG("[PathPlannerROS] max_on_approach_iterations: %u", _node_config.search_info.max_on_approach_iterations);
    ROS_DEBUG("[PathPlannerROS] tolerance: %f", _node_config.search_info.tolerance);
    ROS_DEBUG("[PathPlannerROS] motion_type: %s", motion_type.c_str());
    ROS_DEBUG("[PathPlannerROS] angle_quantization_bins: %u", _node_config.search_info.size_theta);
    ROS_DEBUG("[PathPlannerROS] ==========Parameters smoother==========");
    ROS_DEBUG("[PathPlannerROS] is_holonomic: %s", _node_config.smooth_info.is_holonomic ? "true" : "false");
    ROS_DEBUG("[PathPlannerROS] w_data: %f", _node_config.smooth_info.w_data);
    ROS_DEBUG("[PathPlannerROS] w_smooth: %f", _node_config.smooth_info.w_smooth);
    ROS_DEBUG("[PathPlannerROS] tolerance: %f", _node_config.smooth_info.tolerance);
}
void PathPlannerROS::initSubscribers()
{
    ROS_DEBUG("[PathPlannerROS] Initializing subscribers");
    _initial_pose_sub = _nh.subscribe("/initialpose", 1, &PathPlannerROS::initialPoseCallback, this);
    _goal_pose_sub = _nh.subscribe("/move_base_simple/goal", 1, &PathPlannerROS::goalSetCallback, this);
}
void PathPlannerROS::initPublishers()
{
    ROS_DEBUG("[PathPlannerROS] Initializing publishers");
    _path_pub = _nh.advertise<nav_msgs::Path>("path", 1);
}
void PathPlannerROS::initServices()
{
}
void PathPlannerROS::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    ROS_INFO("[PathPlannerROS::initialPoseCallback] Received initial pose");
    // convert orientation to yaw
    _temp_start_pose = msg->pose.pose;
    if (!_is_start_set)
        _is_start_set = true;
    float start_yaw = tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("start_point x:%f, y:%f, yaw: %f", _temp_start_pose.position.x, _temp_start_pose.position.y, start_yaw);
}
void PathPlannerROS::goalSetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (!_is_start_set)
    {
        ROS_WARN("[PathPlannerROS::goalSetCallback] Received goal before initial pose, ignoring");
        return;
    }
    _temp_goal_pose = msg->pose;
    Coordinate start, goal;
    start.x = _temp_start_pose.position.x;
    start.y = _temp_start_pose.position.y;
    start.theta = tf::getYaw(_temp_start_pose.orientation);
    goal.x = _temp_goal_pose.position.x;
    goal.y = _temp_goal_pose.position.y;
    goal.theta = tf::getYaw(_temp_goal_pose.orientation);
    Coordinates path;
    int iterations;
    std::chrono::time_point<std::chrono::steady_clock> tic = std::chrono::steady_clock::now();
    bool path_planner_success = _astar->createPath(start, goal, path, iterations);
    std::chrono::time_point<std::chrono::steady_clock> toc = std::chrono::steady_clock::now();
    int64_t duration = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();
    ROS_INFO("[PathPlannerROS::initialPoseCallback] Planner taken: %ld ms", duration);
    if (path_planner_success)
    {
        ROS_INFO("[PathPlannerROS::initialPoseCallback] Path found, iterations: %d, Path size: %lu", iterations, path.size());
        tic = std::chrono::steady_clock::now();
        _smoother->smoothPath(path);
        toc = std::chrono::steady_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();
        ROS_INFO("[PathPlannerROS::initialPoseCallback] Smooth taken: %ld ms", duration);
    }

    else
        ROS_WARN("[PathPlannerROS::initialPoseCallback] No path found");

    if (path_planner_success)
    {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        for (Coordinate &coord : path)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = coord.x;
            pose.pose.position.y = coord.y;
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(coord.theta);
            path_msg.poses.push_back(pose);
        }
        _path_pub.publish(path_msg);
    }
}