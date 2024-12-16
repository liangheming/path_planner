#include "path_planner_ros.h"

PathPlannerROS::PathPlannerROS(ros::NodeHandle &nh) : _nh(nh), _tf_buffer(), _tf_listener(_tf_buffer), _costmap_ros("planner_costmap", _tf_buffer)
{
    loadParameters();
    initPublishers();
    initSubscribers();
    initServices();
    _is_start_set = false;
    _search_info.size_x = _costmap_ros.getCostmap()->getSizeInCellsX();
    _search_info.size_y = _costmap_ros.getCostmap()->getSizeInCellsY();
    _collision_checker.setCostMap(_costmap_ros.getCostmap());
    _collision_checker.initBoundary(_node_config.footprint_boundary);
    _astar = std::make_shared<AstarAlgorithm>(_search_info, _collision_checker);
    _smooth_info.motion_type = _search_info.motion_type;
    _path_smoother = std::make_shared<Smoother>(_smooth_info, _collision_checker);
    ROS_INFO("[PathPlannerROS] Initialized");
}
void PathPlannerROS::loadParameters()
{
    ROS_DEBUG("[PathPlannerROS] Loading parameters");
    _nh.param<std::vector<float>>("footprint_boundary", _node_config.footprint_boundary, {1.0});

    _nh.param<float>("minimum_turning_radius", _search_info.minimum_turning_radius, 0.84);
    _smooth_info.minimum_turning_radius = _search_info.minimum_turning_radius;
    _search_info.minimum_turning_radius = ceil(_search_info.minimum_turning_radius / _costmap_ros.getCostmap()->getResolution());

    _nh.param<float>("cost_penalty", _search_info.cost_penalty, 2.0);
    _nh.param<float>("change_penalty", _search_info.change_penalty, 0.0);
    _nh.param<float>("non_straight_penalty", _search_info.non_straight_penalty, 1.20);
    _nh.param<float>("reverse_penalty", _search_info.reverse_penalty, 2.1);
    _nh.param<float>("analytic_expansion_ratio", _search_info.analytic_expansion_ratio, 4.0);
    int max_iterations, max_on_approach_iterations, size_theta;
    _nh.param<int>("max_iterations", max_iterations, 1000000);
    _nh.param<int>("max_on_approach_iterations", max_on_approach_iterations, 1000);
    _nh.param<int>("angle_quantization_bins", size_theta, 64);
    _nh.param<float>("tolerance", _search_info.tolerance, 0.5);
    _search_info.tolerance = _search_info.tolerance / _costmap_ros.getCostmap()->getResolution();
    _search_info.max_iterations = max_iterations;
    _search_info.max_on_approach_iterations = max_on_approach_iterations;
    _search_info.size_theta = size_theta;
    std::string motion_type;
    _nh.param<std::string>("motion_type", motion_type, "ReedsShepp");
    int smooth_iterations;
    _nh.param<int>("smooth_iterations", smooth_iterations, 1000);
    _smooth_info.max_iter = smooth_iterations;
    _nh.param<double>("smooth_w_data", _smooth_info.w_data, 0.2);
    _nh.param<double>("smooth_w_smooth", _smooth_info.w_smooth, 0.5);

    if (motion_type == "Dubins")
        _search_info.motion_type = MotionType::Dubins;
    else if (motion_type == "ReedsShepp")
        _search_info.motion_type = MotionType::ReedsShepp;
    else
    {
        ROS_ERROR("[PathPlannerROS] Invalid motion type: %s", motion_type.c_str());
        return;
    }
    ROS_INFO("[PathPlannerROS] boundary size: %lu", _node_config.footprint_boundary.size());
    ROS_INFO("[PathPlannerROS] minimum_turning_radius: %f", _search_info.minimum_turning_radius);
    ROS_INFO("[PathPlannerROS] cost_penalty: %f", _search_info.cost_penalty);
    ROS_INFO("[PathPlannerROS] change_penalty: %f", _search_info.change_penalty);
    ROS_INFO("[PathPlannerROS] non_straight_penalty: %f", _search_info.non_straight_penalty);
    ROS_INFO("[PathPlannerROS] reverse_penalty: %f", _search_info.reverse_penalty);
    ROS_INFO("[PathPlannerROS] analytic_expansion_ratio: %f", _search_info.analytic_expansion_ratio);
    ROS_INFO("[PathPlannerROS] max_iterations: %d", max_iterations);
    ROS_INFO("[PathPlannerROS] max_on_approach_iterations: %d", max_on_approach_iterations);
    ROS_INFO("[PathPlannerROS] angle_quantization_bins: %d", size_theta);
    ROS_INFO("[PathPlannerROS] Motion type: %s", motion_type.c_str());
    ROS_INFO("[PathPlannerROS] tolerance: %f", _search_info.tolerance);
    ROS_INFO("[PathPlannerROS] smooth_iterations: %d", smooth_iterations);
    ROS_INFO("[PathPlannerROS] smooth_w_data: %f", _smooth_info.w_data);
    ROS_INFO("[PathPlannerROS] smooth_w_smooth: %f", _smooth_info.w_smooth);
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
    _smooth_path_pub = _nh.advertise<nav_msgs::Path>("smooth_path", 1);
}
void PathPlannerROS::initServices()
{
    ROS_DEBUG("[PathPlannerROS] Initializing services");
}

PathPlannerROS::~PathPlannerROS()
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
    float statr_yaw = tf::getYaw(_temp_start_pose.orientation);
    statr_yaw = statr_yaw / _astar->getMotionTable().bin_size;
    ROS_INFO("statr_yaw before normalization: %f", statr_yaw);
    _astar->getMotionTable().normalizeHeading(statr_yaw);
    ROS_INFO("statr_yaw after normalization: %f", statr_yaw);
    float goal_yaw = tf::getYaw(_temp_goal_pose.orientation);
    goal_yaw = goal_yaw / _astar->getMotionTable().bin_size;
    ROS_INFO("goal_yaw before normalization: %f", goal_yaw);
    _astar->getMotionTable().normalizeHeading(goal_yaw);
    ROS_INFO("goal_yaw after normalization: %f", goal_yaw);

    unsigned int start_x, start_y;
    bool start_point_to_grid = _costmap_ros.getCostmap()->worldToMap(_temp_start_pose.position.x, _temp_start_pose.position.y, start_x, start_y);
    if (!start_point_to_grid)
    {
        ROS_ERROR("[PathPlannerROS::initialPoseCallback] Failed to convert start point to grid");
        return;
    }
    unsigned int goal_x, goal_y;
    bool end_point_to_grid = _costmap_ros.getCostmap()->worldToMap(_temp_goal_pose.position.x, _temp_goal_pose.position.y, goal_x, goal_y);
    if (!end_point_to_grid)
    {
        ROS_ERROR("[PathPlannerROS::initialPoseCallback] Failed to convert end point to grid");
        return;
    }
    unsigned int start_theta = static_cast<unsigned int>(statr_yaw);
    unsigned int goal_theta = static_cast<unsigned int>(goal_yaw);
    ROS_INFO("start_point x:%u, y:%u, yaw:%u", start_x, start_y, start_theta);
    ROS_INFO("goal_point x:%u, y:%u, yaw: %u", goal_x, goal_y, goal_theta);
    _astar->clearGraph();
    if (!_astar->setStart(start_x, start_y, start_theta))
    {
        ROS_ERROR("[PathPlannerROS::initialPoseCallback] Failed to set start point");
        return;
    }
    if (!_astar->setGoal(goal_x, goal_y, goal_theta))
    {
        ROS_ERROR("[PathPlannerROS::initialPoseCallback] Failed to set goal point");
        return;
    }

    Coordinates path;
    int iterations;
    std::chrono::time_point<std::chrono::steady_clock> tic = std::chrono::steady_clock::now();
    bool success = _astar->createPath(path, iterations, _search_info.tolerance);
    std::chrono::time_point<std::chrono::steady_clock> toc = std::chrono::steady_clock::now();
    int64_t duration = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();
    ROS_INFO("[PathPlannerROS::initialPoseCallback] Time taken: %ld ms", duration);
    if (success)
        ROS_INFO("[PathPlannerROS::initialPoseCallback] Path found: %d, iterations: %d, Path size: %lu", success, iterations, path.size());
    else
        ROS_ERROR("[PathPlannerROS::initialPoseCallback] Path not found: %d, iterations: %d", success, iterations);
    bool smooth_success = false;
    Coordinates smooth_path;
    if (success)
    {
        for (Coordinate &coord : path)
        {
            double wx, wy;
            _costmap_ros.getCostmap()->mapToWorld(static_cast<unsigned int>(coord.x), static_cast<unsigned int>(coord.y), wx, wy);
            coord.theta = normalize_angle(coord.theta * _astar->getMotionTable().bin_size);
            coord.x = wx;
            coord.y = wy;
        }
        smooth_path = path;
        smooth_success = _path_smoother->smoothPath(smooth_path);
    }

    if (success)
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

    if (success)
    {
        ROS_INFO("[PathPlannerROS::initialPoseCallback] Smoothing successful");
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        for (Coordinate &coord : smooth_path)
        {

            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = coord.x;
            pose.pose.position.y = coord.y;
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(coord.theta);
            path_msg.poses.push_back(pose);
        }
        _smooth_path_pub.publish(path_msg);
    };
}