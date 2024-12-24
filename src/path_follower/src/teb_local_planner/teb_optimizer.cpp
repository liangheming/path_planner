#include "teb_optimizer.h"

bool VertexDouble::read(std::istream &is)
{
    is >> _estimate;
    return true;
}
bool VertexDouble::write(std::ostream &os) const
{
    os << _estimate;
    return true;
}
void VertexDouble::setToOriginImpl()
{
    _estimate = 0.1;
}
void VertexDouble::oplusImpl(const double *update)
{
    _estimate += update[0];
}

TebOptimizer::TebOptimizer(FollowerInfo *follower_info) : _follower_info(follower_info)
{
}

Pose2E TebOptimizer::plan(const Pose2Es &initial_trajectory, const Pose2E &start_vel)
{
    _cached_start_vel = start_vel;
    _cached_trajectory.assign(initial_trajectory.begin(), initial_trajectory.end());
    initTrajectory();
    optimize();
    return Pose2E();
}
void TebOptimizer::initTrajectory()
{
    assert(_cached_trajectory.size() > 1);
    const Pose2E &start = _cached_trajectory.front();
    const Pose2E &goal = _cached_trajectory.back();
    // 对朝向进行差值修正
    if (_follower_info->overwrite_orientation)
    {
        bool backward = false;
        double uni_x = cos(start.theta);
        double uni_y = sin(start.theta);
        double dx = goal.x - start.x;
        double dy = goal.y - start.y;
        if (dx * uni_x + dy * uni_y < 0)
            backward = true;
        for (unsigned int i = 0; i < _cached_trajectory.size() - 1; ++i)
        {
            dx = _cached_trajectory[i + 1].x - _cached_trajectory[i].x;
            dy = _cached_trajectory[i + 1].y - _cached_trajectory[i].y;
            _cached_trajectory[i].theta = atan2(dy, dx);
            if (backward)
                _cached_trajectory[i].theta += M_PI;
        }
    }
    // 进行简单的时间差值初始化
    _cached_time_diffs.clear();
    for (unsigned int i = 0; i < _cached_trajectory.size() - 1; ++i)
    {
        _cached_time_diffs.push_back(estimateTimeDiff(_cached_trajectory[i], _cached_trajectory[i + 1]));
    }
}

void TebOptimizer::pruneTrajectory()
{
}
bool TebOptimizer::optimize()
{
    bool success = false;
    double weight_multiplier = 1.0;
    for (int i = 0; i < _follower_info->no_outer_iterations; ++i)
    {
        autoResize();
        break;

        buildGraph(weight_multiplier);
        success = optimizeGraph();
        if (!success)
        {
            clearGraph();
            return false;
        }
        weight_multiplier *= _follower_info->weight_adapt_factor;
        clearGraph();
    }
    return true;
}

void TebOptimizer::buildGraph(double weight_multiplier)
{
}
void TebOptimizer::clearGraph()
{
}

bool TebOptimizer::optimizeGraph()
{
    return true;
}

void TebOptimizer::autoResize()
{
    assert(!_cached_time_diffs.empty());

    /**
     * 删除时间差值过小的轨迹点
     */
    std::vector<double>::iterator time_diff_iter = _cached_time_diffs.begin();
    Pose2Es::iterator pose_iter = _cached_trajectory.begin() + 1;
    while (time_diff_iter != _cached_time_diffs.end() && pose_iter != _cached_trajectory.end())
    {
        double dt = *time_diff_iter;
        if (dt < _follower_info->dt_ref - _follower_info->dt_std && time_diff_iter != (_cached_time_diffs.end() - 1))
        {
            time_diff_iter = _cached_time_diffs.erase(time_diff_iter);
            pose_iter = _cached_trajectory.erase(pose_iter);
            if (time_diff_iter != _cached_time_diffs.end())
                *time_diff_iter += dt;
        }
        else
        {
            time_diff_iter++;
            pose_iter++;
        }
    }

    assert(time_diff_iter == _cached_time_diffs.end() && pose_iter == _cached_trajectory.end());

    double last_dt = _cached_time_diffs.back();
    if (last_dt < _follower_info->dt_ref - _follower_info->dt_std)
    {
        _cached_time_diffs.erase(_cached_time_diffs.end() - 1);
        _cached_trajectory.erase(_cached_trajectory.end() - 2);
        _cached_time_diffs.back() += last_dt;
    }

    /**
     * 对间隔较大的轨迹点进行插值
     */
    for (unsigned int i = 0; i < 100; ++i)
    {
        time_diff_iter = _cached_time_diffs.begin();
        pose_iter = _cached_trajectory.begin();
        bool changed = false;
        while (time_diff_iter != _cached_time_diffs.end() && pose_iter != (_cached_trajectory.end() - 1))
        {
            double dt = *time_diff_iter;
            if (dt > _follower_info->dt_ref + _follower_info->dt_std)
            {
                Pose2E center_p = Pose2E::average(*pose_iter, *(pose_iter + 1));
                double half_dt = dt / 2;
                pose_iter = _cached_trajectory.insert(pose_iter + 1, center_p);
                *time_diff_iter = half_dt;
                time_diff_iter = _cached_time_diffs.insert(time_diff_iter + 1, half_dt);
                changed = true;
            }
            time_diff_iter++;
            pose_iter++;
        }
        assert(time_diff_iter == _cached_time_diffs.end() && pose_iter == (_cached_trajectory.end() - 1));
        if (!changed || _cached_time_diffs.size() >= _follower_info->max_samples)
            break;
    }

    // time_diff_iter = _cached_time_diffs.begin();
    // pose_iter = _cached_trajectory.begin();
    // while (time_diff_iter != _cached_time_diffs.end())
    // {
    //     double distance = std::hypot(pose_iter->x - (pose_iter + 1)->x, pose_iter->y - (pose_iter + 1)->y);
    //     double delta_theta = std::abs(normalizeAngle(pose_iter->theta - (pose_iter + 1)->theta));

    //     // std::cout << "time_diff: " << *time_diff_iter << " | " << std::max(distance / _follower_info->max_vel_x, delta_theta / _follower_info->max_vel_theta) << std::endl;
    //     std::cout << pose_iter->x << " " << pose_iter->y << " | " << pose_iter->theta / M_PI * 180 << std::endl;
    //     time_diff_iter++;
    //     pose_iter++;
    // }
}
double TebOptimizer::estimateTimeDiff(const Pose2E &p1, const Pose2E &p2)
{
    double dt_constant_motion = 0.1;
    if (_follower_info->max_vel_x > 0)
    {
        double distance = std::hypot(p2.x - p1.x, p2.y - p1.y);
        dt_constant_motion = distance / _follower_info->max_vel_x;
    }
    if (_follower_info->max_vel_theta > 0)
    {
        double rot_dis = std::abs(normalizeAngle(p2.theta - p1.theta));
        dt_constant_motion = std::max(dt_constant_motion, rot_dis / _follower_info->max_vel_theta);
    }
    return dt_constant_motion;
}