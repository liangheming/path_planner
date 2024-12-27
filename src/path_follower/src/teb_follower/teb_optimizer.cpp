#include "teb_optimizer.h"

TebOptimizer::TebOptimizer(FollowerInfo *follower_info, RobotModel *robot_model)
    : _info(follower_info), _robot(robot_model), _kdtree(2, _obstacles)
{
    initOptimizer();
}

void TebOptimizer::initOptimizer()
{
    std::unique_ptr<TebLinearSolver> linear_solver = std::make_unique<TebLinearSolver>();
    linear_solver->setBlockOrdering(true);
    std::unique_ptr<TebBlockSolver> block_solver = std::make_unique<TebBlockSolver>(std::move(linear_solver));
    _optimizer = std::make_shared<g2o::SparseOptimizer>();
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
    _optimizer->setAlgorithm(solver);
    _optimizer->setVerbose(false);
    _optimizer->initMultiThreading();
}

void TebOptimizer::cacheToVertices()
{
    _trajectory.clear();
    _timediffs.clear();
    unsigned int id_counter = 0;
    for (unsigned int i = 0; i < _cached_trajectory.size(); ++i)
    {
        Vertex2E *vertex = new Vertex2E(_cached_trajectory[i]);
        if (i == 0 || i == _cached_trajectory.size() - 1)
            vertex->setFixed(true);
        vertex->setId(id_counter++);
        _optimizer->addVertex(vertex);
        _trajectory.push_back(vertex);
    }

    for (unsigned int i = 0; i < _cached_timediffs.size(); ++i)
    {
        VertexTimeDiff *timediff = new VertexTimeDiff(_cached_timediffs[i]);
        timediff->setId(id_counter++);
        _optimizer->addVertex(timediff);
        _timediffs.push_back(timediff);
    }
}

void TebOptimizer::verticesToCache()
{
    _cached_timediffs.clear();
    _cached_trajectory.clear();
    for (unsigned int i = 0; i < _trajectory.size(); ++i)
    {
        _cached_trajectory.push_back(_trajectory[i]->estimate());
    }
    for (unsigned int i = 0; i < _timediffs.size(); ++i)
    {
        _cached_timediffs.push_back(_timediffs[i]->estimate());
    }
}

void TebOptimizer::buildGraph()
{
}

bool TebOptimizer::optimizeGraph()
{
    return _optimizer->initializeOptimization() && _optimizer->optimize(_info->no_inner_iterations) > 0;
}

void TebOptimizer::autoResize()
{
    if (_cached_timediffs.size() < 2)
        return;
    /**
     * 删除时间间隔较小的节点
     */
    std::vector<double>::iterator timediffs_iter = _cached_timediffs.begin();
    std::vector<Point2E>::iterator trajectory_iter = _cached_trajectory.begin() + 1;
    while (timediffs_iter != _cached_timediffs.end() && trajectory_iter != _cached_trajectory.end())
    {
        double dt = *timediffs_iter;
        if (dt < _info->dt_ref - _info->dt_std && timediffs_iter != (_cached_timediffs.end() - 1))
        {
            timediffs_iter = _cached_timediffs.erase(timediffs_iter);
            trajectory_iter = _cached_trajectory.erase(trajectory_iter);
            if (timediffs_iter != _cached_timediffs.end())
                *timediffs_iter += dt;
        }
        else
        {
            timediffs_iter++;
            trajectory_iter++;
        }
    }
    assert(timediffs_iter == _cached_timediffs.end() && trajectory_iter == _cached_trajectory.end());
    double last_dt = _cached_timediffs.back();
    if (last_dt < _info->dt_ref - _info->dt_std)
    {
        _cached_timediffs.erase(_cached_timediffs.end() - 1);
        _cached_trajectory.erase(_cached_trajectory.end() - 2);
        _cached_timediffs.back() += last_dt;
    }

    /**
     * 对时间间隔较大的点进行插值
     */
    for (unsigned int i = 0; i < 100; ++i)
    {
        timediffs_iter = _cached_timediffs.begin();
        trajectory_iter = _cached_trajectory.begin();
        bool changed = false;
        while (timediffs_iter != _cached_timediffs.end() && trajectory_iter != (_cached_trajectory.end() - 1))
        {
            double dt = *timediffs_iter;
            if (dt > _info->dt_ref + _info->dt_std)
            {
                Point2E interpolate = Point2E::average(*trajectory_iter, *(trajectory_iter + 1));
                double half_dt = dt / 2;
                trajectory_iter = _cached_trajectory.insert(trajectory_iter + 1, interpolate);
                *timediffs_iter = half_dt;
                timediffs_iter = _cached_timediffs.insert(timediffs_iter + 1, half_dt);
                changed = true;
            }
            timediffs_iter++;
            trajectory_iter++;
        }
        assert(timediffs_iter == _cached_timediffs.end() && trajectory_iter == _cached_trajectory.end() - 1);
        if (!changed || _cached_timediffs.size() >= _info->max_samples)
            break;
    }
    
}