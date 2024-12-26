#include "teb_local_planner.h"

TebLocalPlanner::TebLocalPlanner(FollowerInfo *follower_info, costmap_2d::Costmap2D *costmap, RobotModel *robot_model)
    : _follower_info(follower_info),
      _costmap(costmap),
      _robot_model(robot_model),
      _optimizer(follower_info, robot_model)
{
}

/**
 * 删除全局路径中机器人后面的一些点
 */
void TebLocalPlanner::pruneGlobalPath()
{
    if (_global_path.size() <= 1)
        return;
    Pose2Es::iterator erase_end;
    double ignore_back_distance_sq = _follower_info->ignore_back_range * _follower_info->ignore_back_range;
    for (Pose2Es::iterator it = _global_path.begin(); it != _global_path.end(); it++)
    {
        Pose2E &pose = *it;
        double dx = pose.x - _current_pose.x;
        double dy = pose.y - _current_pose.y;
        if (dx * dx + dy * dy < ignore_back_distance_sq)
        {
            erase_end = it;
            break;
        }
    }
    if (erase_end == _global_path.end())
        return;
    _global_path.erase(_global_path.begin(), erase_end);
}
void TebLocalPlanner::generateLocalPath()
{
    _local_path.clear();
    if (_global_path.size() < 1)
        return;

    // 找到距离机器人最近的轨迹点
    bool robot_reached = false;
    double sq_dist = 1e10;
    unsigned int start_idx;
    for (unsigned int i = 0; i < _global_path.size(); i++)
    {
        double dx = _global_path[i].x - _current_pose.x;
        double dy = _global_path[i].y - _current_pose.y;
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
    double sq_dist_thresh = _follower_info->max_follow_range * _follower_info->max_follow_range;
    /**
     * 截取一定范围内的轨迹点作为局部路径
     */

    while (start_idx < _global_path.size() && sq_dist <= sq_dist_thresh && (_follower_info->max_follow_length < 0 || plan_length < _follower_info->max_follow_length))
    {
        double dx = _global_path[start_idx].x - _current_pose.x;
        double dy = _global_path[start_idx].y - _current_pose.y;
        double cur_sq_dist = dx * dx + dy * dy;
        _local_path.push_back(_global_path[start_idx]);
        sq_dist = cur_sq_dist;
        if (start_idx > 0 && _follower_info->max_follow_length > 0)
            plan_length += std::hypot(_global_path[start_idx].x - _global_path[start_idx - 1].x, _global_path[start_idx].y - _global_path[start_idx - 1].y);
        start_idx++;
    }
}
void TebLocalPlanner::generateViaPoints()
{
    _via_points.clear();
    if (_follower_info->viapoint_seperation <= 0)
        return;
    unsigned int prev_idx = 0;
    for (unsigned int i = 1; i < _local_path.size(); i++)
    {
        Pose2E &pose = _local_path[i];
        double distance = hypot(pose.x - _local_path[prev_idx].x, pose.y - _local_path[prev_idx].y);
        if (distance < _follower_info->viapoint_seperation)
            continue;
        _via_points.emplace_back(pose.x, pose.y);
        prev_idx = i;
    }
}
void TebLocalPlanner::generateObstacles()
{
    _optimizer.clearObstacles();
    unsigned int size_x = _costmap->getSizeInCellsX();
    unsigned int size_y = _costmap->getSizeInCellsY();
    double uni_vec_x = cos(_current_pose.theta);
    double uni_vec_y = sin(_current_pose.theta);
    double max_distance = _follower_info->max_follow_range * 1.2;
    double wx, wy, dx, dy, distance, obs_vec_x, obs_vec_y;
    for (unsigned int i = 0; i < size_x; i++)
    {
        for (unsigned int j = 0; j < size_y; j++)
        {
            unsigned char cost = _costmap->getCost(i, j);
            if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            {
                _costmap->mapToWorld(i, j, wx, wy);
                dx = wx - _current_pose.x;
                dy = wy - _current_pose.y;
                distance = hypot(dx, dy);
                if (abs(dx) > max_distance && abs(dy) > max_distance)
                    continue;
                if (distance > _follower_info->obstacle_behind_ignore_range && (dx * uni_vec_x + dy * uni_vec_y) < 0)
                    continue;
                _optimizer.addObstacles(wx, wy);
            }
        }
    }
    _optimizer.rebuildKDTree();
   
}
Pose2E TebLocalPlanner::getLocalTwists()
{
    pruneGlobalPath();
    generateLocalPath();
    generateViaPoints();
    generateObstacles();
    if (_local_path.size() > 1)
        _local_path.front() = _current_pose;
    _optimizer.plan(_local_path, _current_vel);
    return Pose2E(0, 0, 0);
}
