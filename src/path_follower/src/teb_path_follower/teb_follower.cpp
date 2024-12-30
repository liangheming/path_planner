#include "teb_follower.h"

TebFollower::TebFollower(FollowerInfo *info, RobotModel *robot)
    : _info(info), _robot(robot), _costmap(nullptr), _optimizer(_info, _robot)
{
}
TebFollower::TebFollower(FollowerInfo *info, RobotModel *robot, costmap_2d::Costmap2D *costmap)
    : _info(info), _robot(robot), _costmap(costmap), _optimizer(_info, _robot)
{
}

void TebFollower::setGlobalPath(std::vector<Point2E> &path)
{
    _path_of_global.clear();
    _path_of_global.assign(path.begin(), path.end());
}

FollowResult TebFollower::makePlane(const Point2E &robot_pose, const Velocity &robot_vel, Velocity &cmd_vel)
{
    if (_path_of_global.empty())
        return FollowResult::EMPTY_PLAN;
    _robot_pose = robot_pose;
    _robot_vel = robot_vel;
    pruneGlobalPath();
    int global_index = generatePathToFollow();

    Point2E robot_to_goal = Point2E::delta(_robot_pose, _path_of_global.back());
    if (robot_to_goal.translation().norm() < _info->xy_goal_tolerance && abs(robot_to_goal.angle()) < _info->yaw_goal_tolerance && (isAlmostStop() || _info->free_goal_vel))
        return FollowResult::GOAL_REACH;

    overwriteOrientation(global_index);
    addViaPoints();
    addObstacles();
    _optimizer.mutableStartVelocity().first = true;
    _optimizer.mutableStartVelocity().second = _robot_vel;
    _optimizer.mutableGoalVelocity().second.setZero();
    if (global_index == _path_of_global.size() - 1)
        _optimizer.mutableGoalVelocity().first = true;
    else
        _optimizer.mutableGoalVelocity().first = false;
    bool success = _optimizer.smoothTrajectory(_path_to_follow);
    if (!success)
    {
        _optimizer.clearCached();
        return FollowResult::FAILED;
    }

    // 这里还需要作一个collsion的判断

    // 这里还需要一个振荡检测

    getVelocityCmd(cmd_vel);
    return FollowResult::SUCCESS;
}
void TebFollower::pruneGlobalPath()
{
    if (_path_of_global.size() <= 1)
        return;
    Point2EVec::iterator erase_iter;
    double ignore_back_distance_sq = _info->ignore_back_range * _info->ignore_back_range;
    for (Point2EVec::iterator it = _path_of_global.begin(); it != _path_of_global.end(); it++)
    {
        Point2E &pose = *it;
        double dx = pose.x - _robot_pose.x;
        double dy = pose.y - _robot_pose.y;
        if (dx * dx + dy * dy < ignore_back_distance_sq)
        {
            erase_iter = it;
            break;
        }
    }
    if (erase_iter != _path_of_global.end())
        _path_of_global.erase(_path_of_global.begin(), erase_iter);
}

int TebFollower::generatePathToFollow()
{
    _path_to_follow.clear();
    if (_path_of_global.empty())
        return -1;
    bool robot_reached = false;
    double sq_dist = 1e10;
    unsigned int start_idx;
    for (unsigned int i = 0; i < _path_of_global.size(); i++)
    {
        double dx = _path_of_global[i].x - _robot_pose.x;
        double dy = _path_of_global[i].y - _robot_pose.y;
        double cur_sq_dist = dx * dx + dy * dy;
        if (robot_reached && cur_sq_dist > sq_dist)
            break;
        if (cur_sq_dist < sq_dist)
        {
            sq_dist = cur_sq_dist;
            start_idx = i;
            if (sq_dist < 0.05)
                robot_reached = true;
        }
    }

    double plan_length = 0;
    double sq_dist_thresh = _info->max_follow_range * _info->max_follow_range;

    while (start_idx < _path_of_global.size() && sq_dist <= sq_dist_thresh && (_info->max_follow_length < 0 || plan_length < _info->max_follow_length))
    {
        double dx = _path_of_global[start_idx].x - _robot_pose.x;
        double dy = _path_of_global[start_idx].y - _robot_pose.y;
        double cur_sq_dist = dx * dx + dy * dy;
        _path_to_follow.push_back(_path_of_global[start_idx]);
        sq_dist = cur_sq_dist;
        if (start_idx > 0 && _info->max_follow_length > 0)
            plan_length += Point2E::distance(_path_of_global[start_idx - 1], _path_of_global[start_idx]);
        start_idx++;
    }

    if (_path_to_follow.size() == 1)
        _path_to_follow.insert(_path_to_follow.begin(), _robot_pose);
    else
        _path_to_follow.front() = _robot_pose;

    return static_cast<int>(start_idx - 1);
}

void TebFollower::addViaPoints()
{
    _optimizer.clearViaPoints();
    if (_info->viapoint_seperation <= 0)
        return;
    unsigned int prev_idx = 0;
    for (unsigned int i = 1; i < _path_to_follow.size(); i++)
    {
        Point2E &pose = _path_to_follow[i];
        double distance = Point2E::distance(_path_to_follow[prev_idx], pose);
        if (distance < _info->viapoint_seperation)
            continue;
        _optimizer.addViaPoint(pose.x, pose.y);
        prev_idx = i;
    }
}

void TebFollower::addObstacles()
{
    _optimizer.clearObstacles();
    unsigned int size_x = _costmap->getSizeInCellsX();
    unsigned int size_y = _costmap->getSizeInCellsY();
    Eigen::Vector2d robot_dir = _robot_pose.direction();
    double max_distance = _info->max_follow_range * 1.2;
    double wx, wy, dx, dy, distance;
    for (unsigned int i = 0; i < size_x; i++)
    {
        for (unsigned int j = 0; j < size_y; j++)
        {
            unsigned char cost = _costmap->getCost(i, j);
            if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            {
                _costmap->mapToWorld(i, j, wx, wy);
                dx = wx - _robot_pose.x;
                dy = wy - _robot_pose.y;
                distance = hypot(dx, dy);
                if (abs(dx) > max_distance || abs(dy) > max_distance)
                    continue;
                if (distance > _info->obstacle_behind_ignore_range && (dx * robot_dir.x() + dy * robot_dir.y() < 0))
                    continue;
                _optimizer.addObstacle(wx, wy);
            }
        }
    }
    if (!_optimizer.mutableCanidateObstacles().empty())
        _optimizer.buildObstacleKDTree();
}

void TebFollower::overwriteOrientation(const int &global_index)
{
    if (global_index < 0 || global_index >= static_cast<int>(_path_of_global.size()) || !_info->overwrite_orientation)
        return;
    /**
     * 1. 首先固定本地路径终点的朝向
     * 2. 然后差值的方式估计轨迹上每一个点的朝向
     */
    // global_index 正好是最后一个点，则使用最后一个点的朝向
    int global_size = static_cast<int>(_path_of_global.size());
    if (global_index >= global_size - 1)
    {
        _path_to_follow.back().theta = normalize_theta(_path_of_global[global_index].theta);
    }
    // global_index 不是最后一个点，则使用当前点与下一个点的差值作为朝向
    else
    {
        Point2E &goal_pose = _path_of_global[global_index];
        Point2E &next_pose = _path_of_global[global_index + 1];
        _path_to_follow.back().theta = normalize_theta(atan2(next_pose.y - goal_pose.y, next_pose.x - goal_pose.x));
    }

    const Point2E &start = _path_to_follow.front();
    const Point2E &goal = _path_to_follow.back();

    bool backward = false;
    Eigen::Vector2d start_dir = start.direction();
    Eigen::Vector2d move_dir = Point2E::delta(start, goal).translation().normalized();
    if (start_dir.dot(move_dir) < 0)
        backward = true;
    for (unsigned int i = 1; i < _path_to_follow.size() - 1; ++i)
    {
        double dx = _path_to_follow[i + 1].x - _path_to_follow[i].x;
        double dy = _path_to_follow[i + 1].y - _path_to_follow[i].y;
        _path_to_follow[i].theta = std::atan2(dy, dx);
        if (backward && _info->allow_init_backwords)
            _path_to_follow[i].theta += M_PI;
        _path_to_follow[i].theta = normalize_theta(_path_to_follow[i].theta);
    }
}

bool TebFollower::getVelocityCmd(Velocity &cmd_vel)
{
    cmd_vel.setZero();
    const std::vector<Point2E> &optimized_trajectory = _optimizer.mutableTrajectory();
    const std::vector<double> &optimized_timediffs = _optimizer.mutableTimediffs();
    assert(optimized_trajectory.size() == optimized_timediffs.size() + 1 && optimized_trajectory.size() > 1);
    const Point2E &start = optimized_trajectory.front();
    size_t next_id = std::min(optimized_trajectory.size() - 1, static_cast<size_t>(_info->look_forward_poses_for_control));
    if (next_id == 0)
        next_id = 1;
    double dt = 0;
    for (size_t counter = 0; counter < next_id; ++counter)
    {
        dt += optimized_timediffs[counter];
    }
    const Point2E &goal = optimized_trajectory[next_id];

    Point2E delta = Point2E::delta(start, goal);
    Eigen::Vector2d delta_trans = delta.translation();
    double delta_angel = delta.angle();
    Eigen::Vector2d start_dir = start.direction();
    double direction = delta_trans.dot(start_dir);
    double distance = delta_trans.norm();
    if (abs(delta_angel > 0.001) && _info->exact_arc_length)
    {
        double radius = distance / (2 * sin(delta_angel / 2));
        distance = abs(radius * delta_angel);
    }
    distance *= static_cast<double>(g2o::sign(direction));
    cmd_vel.x = distance / dt;
    cmd_vel.theta = delta_angel / dt;
    cmd_vel.y = 0;
}