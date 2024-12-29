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

double TebOptimizer::esitmateTimediff(const Point2E &p1, const Point2E &p2)
{
    double dt_constant_motion = 0.1;
    Point2E delta = Point2E::delta(p1, p2);
    if (_info->max_vel_forward > 0)
        std::max(dt_constant_motion, delta.translation().norm() / _info->max_vel_forward);
    if (_info->max_vel_rotation > 0)
        std::max(dt_constant_motion, std::abs(delta.angle() / _info->max_vel_rotation));
    return dt_constant_motion;
}

void TebOptimizer::initTrajectory(std::vector<Point2E> &trajectory)
{
    _cached_trajectory.clear();
    _cached_trajectory.assign(trajectory.begin(), trajectory.end());
    assert(_cached_trajectory.size() > 1);
    const Point2E &start = _cached_trajectory.front();
    const Point2E &goal = _cached_trajectory.back();
    if (_info->overwrite_orientation)
    {
        bool backward = false;
        Eigen::Vector2d start_dir = start.direction();
        Eigen::Vector2d move_dir = Point2E::delta(start, goal).translation().normalized();
        if (start_dir.dot(move_dir) < 0)
            backward = true;
        for (unsigned int i = 1; i < _cached_trajectory.size() - 1; ++i)
        {
            double dx = _cached_trajectory[i + 1].x - _cached_trajectory[i].x;
            double dy = _cached_trajectory[i + 1].y - _cached_trajectory[i].y;
            _cached_trajectory[i].theta = std::atan2(dy, dx);
            if (backward && _info->allow_init_backwords)
                _cached_trajectory[i].theta += M_PI;
            _cached_trajectory[i].theta = normalize_theta(_cached_trajectory[i].theta);
        }
    }
    _cached_timediffs.clear();
    for (unsigned int i = 1; i < _cached_trajectory.size(); ++i)
    {
        double dt = esitmateTimediff(_cached_trajectory[i - 1], _cached_trajectory[i]);
        _cached_timediffs.push_back(dt);
    }
}

bool TebOptimizer::optimizeTrajectory()
{

    for (int iter = 0; iter < _info->no_outer_iterations; ++iter)
    {
        autoResize();
        assert(_cached_trajectory.size() == _cached_timediffs.size() + 1);
        cacheToVertices();
        buildGraph(iter);
        if (!optimizeGraph())
        {
            clearGraph();
            return false;
        }
        verticesToCache();
        clearGraph();
    }
    return true;
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

void TebOptimizer::buildGraph(const int &iteration)
{
    addVelocityEdges();
    addAccelerationEdges();
    addKineticEdges();
    addObstacleEdges();
    addViapointEdges();
    addShortestPathEdges();
    addTimeOptimalEdges();
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

void TebOptimizer::addVelocityEdges()
{
    if (_info->weight_max_vel_travel == 0 && _info->weight_max_vel_rotation == 0)
        return;
    Eigen::Matrix2d info = Eigen::Matrix2d::Identity();
    info(0, 0) = _info->weight_max_vel_travel;
    info(1, 1) = _info->weight_max_vel_rotation;
    for (unsigned int i = 0; i < _timediffs.size(); i++)
    {
        EdgeVelocity *edge = new EdgeVelocity(_info);
        edge->setVertex(0, _trajectory[i]);
        edge->setVertex(1, _trajectory[i + 1]);
        edge->setVertex(2, _timediffs[i]);
        edge->setInformation(info);
        _optimizer->addEdge(edge);
    }
}
void TebOptimizer::addAccelerationEdges()
{
    if (_info->weight_max_acc_travel == 0 && _info->weight_max_acc_rotation == 0)
        return;
    Eigen::Matrix2d info = Eigen::Matrix2d::Identity();
    info(0, 0) = _info->weight_max_acc_travel;
    info(1, 1) = _info->weight_max_acc_rotation;

    if (_start_vel.first && _timediffs.size() > 0)
    {
        EdgeAccelerationStart *edge = new EdgeAccelerationStart(_info, _start_vel.second);
        edge->setVertex(0, _trajectory[0]);
        edge->setVertex(1, _trajectory[1]);
        edge->setVertex(2, _timediffs[0]);
        edge->setInformation(info);
        _optimizer->addEdge(edge);
    }
    for (unsigned int i = 0; i < _timediffs.size() - 1; i++)
    {
        EdgeAcceleration *edge = new EdgeAcceleration(_info);
        edge->setVertex(0, _trajectory[i]);
        edge->setVertex(1, _trajectory[i + 1]);
        edge->setVertex(2, _trajectory[i + 2]);
        edge->setVertex(3, _timediffs[i]);
        edge->setVertex(4, _timediffs[i + 1]);
        edge->setInformation(info);
        _optimizer->addEdge(edge);
    }
    if (_goal_vel.first && _timediffs.size() > 0)
    {
        EdgeAccelerationGoal *edge = new EdgeAccelerationGoal(_info, _goal_vel.second);
        edge->setVertex(0, _trajectory[_trajectory.size() - 2]);
        edge->setVertex(1, _trajectory[_trajectory.size() - 1]);
        edge->setVertex(2, _timediffs[_timediffs.size() - 1]);
        edge->setInformation(info);
        _optimizer->addEdge(edge);
    }
}
void TebOptimizer::addObstacleEdges()
{
    _affected_obstacles.clear();
    double circum_radius = _robot->circumRadius();
    double affect_radius = (circum_radius + _info->obstacle_inflation_dist) * _info->obstacle_search_factory;
    int max_search_number = _info->obstacle_search_number;
    std::vector<nanoflann::ResultItem<uint32_t, double>> results;
    std::vector<Point2D> &obstacle_points = _obstacles.points();
    Eigen::Matrix2d info = Eigen::Matrix2d::Identity();
    info(0, 0) = _info->weight_obstacle_collision;
    info(1, 1) = _info->weight_obstacle_inflation;
    for (unsigned int i = 0; i < _trajectory.size(); ++i)
    {
        results.clear();
        double query_points[2] = {_trajectory[i]->estimate().x, _trajectory[i]->estimate().y};
        _kdtree.radiusSearch(query_points, affect_radius, results);
        int num_obstacles = std::min(max_search_number, (int)results.size());
        if (num_obstacles == 0)
            continue;
        for (unsigned int j = 0; j < num_obstacles; ++j)
        {
            nanoflann::ResultItem<uint32_t, double> &item = results[j];
            EdgeObstacle *edge = new EdgeObstacle(&obstacle_points[item.first]);
            edge->setInfoParameters(_robot, _info);
            edge->setVertex(0, _trajectory[i]);
            edge->setInformation(info);
            _optimizer->addEdge(edge);
        }
    }
}
void TebOptimizer::addKineticEdges()
{
    if (_info->weight_kinematics_smooth == 0 && _info->weight_kinematics_turning_radius == 0)
        return;
    Eigen::Matrix2d info = Eigen::Matrix2d::Identity();
    info(0, 0) = _info->weight_kinematics_smooth;
    info(1, 1) = _info->weight_kinematics_turning_radius;
    for (unsigned int i = 0; i < _trajectory.size() - 1; i++)
    {
        EdgeKinematics *edge = new EdgeKinematics(_info);
        edge->setVertex(0, _trajectory[i]);
        edge->setVertex(1, _trajectory[i + 1]);
        edge->setInformation(info);
        _optimizer->addEdge(edge);
    }
}
void TebOptimizer::addShortestPathEdges()
{
    if (_info->weight_shortest_path == 0)
        return;
    Eigen::Matrix<double, 1, 1> info = Eigen::Matrix<double, 1, 1>::Identity() * _info->weight_shortest_path;
    for (unsigned int i = 0; i < _trajectory.size() - 1; i++)
    {
        EdgeShortestPath *edge = new EdgeShortestPath();
        edge->setVertex(0, _trajectory[i]);
        edge->setVertex(1, _trajectory[i + 1]);
        edge->setInformation(info);
        _optimizer->addEdge(edge);
    }
}
void TebOptimizer::addTimeOptimalEdges()
{
    if (_info->weight_optimal_time == 0)
        return;
    Eigen::Matrix<double, 1, 1> info = Eigen::Matrix<double, 1, 1>::Identity() * _info->weight_optimal_time;
    for (unsigned int i = 0; i < _timediffs.size() - 1; i++)
    {
        EdgeTimeOptimal *edge = new EdgeTimeOptimal();
        edge->setVertex(0, _timediffs[i]);
        edge->setInformation(info);
        _optimizer->addEdge(edge);
    }
}
void TebOptimizer::addViapointEdges()
{
    if (_info->weight_viapoints == 0)
        return;
}