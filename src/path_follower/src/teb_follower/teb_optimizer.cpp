#include "teb_optimizer.h"

TebOptimizer::TebOptimizer(FollowerInfo *follower_info, RobotModel *robot_model) : _follower_info(follower_info), _kdtree(2, _obstacles), _robot_model(robot_model)
{
    std::unique_ptr<TebLinearSolver> linear_solver = std::make_unique<TebLinearSolver>();
    linear_solver->setBlockOrdering(true);
    std::unique_ptr<TebBlockSolver> block_solver = std::make_unique<TebBlockSolver>(std::move(linear_solver));
    _optimizer = std::make_shared<g2o::SparseOptimizer>();
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
    // 这里只需要new就好了，_optimizer 析构的时候会自动析构solver
    _optimizer->setAlgorithm(solver);
    _optimizer->initMultiThreading();
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
            if (i == 0)
                continue;
            dx = _cached_trajectory[i + 1].x - _cached_trajectory[i].x;
            dy = _cached_trajectory[i + 1].y - _cached_trajectory[i].y;
            _cached_trajectory[i].theta = atan2(dy, dx);
            if (backward)
                _cached_trajectory[i].theta += M_PI;
            _cached_trajectory[i].theta = normalizeAngle(_cached_trajectory[i].theta);
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
        setVertices();
        buildGraph(weight_multiplier);
        success = optimizeGraph();
        if (!success)
        {
            clearGraph();
            return false;
        }
        weight_multiplier *= _follower_info->weight_adapt_factor;
        getVertices();
        clearGraph();
    }

    return true;
}
void TebOptimizer::setVertices()
{
    _trajectory.clear();
    _time_diffs.clear();
    unsigned int id_counter = 0;
    for (unsigned int i = 0; i < _cached_trajectory.size(); ++i)
    {
        g2o::VertexSE2 *vertex = new g2o::VertexSE2();
        vertex->setFixed(false);
        if (i == 0 || i == _cached_trajectory.size() - 1)
            vertex->setFixed(true);
        vertex->setId(id_counter++);
        vertex->setEstimate(g2o::SE2(_cached_trajectory[i].x, _cached_trajectory[i].y, _cached_trajectory[i].theta));
        _optimizer->addVertex(vertex);
        _trajectory.emplace_back(vertex);
        // std::cout << "pose: " << _cached_trajectory[i].x << " " << _cached_trajectory[i].y << " " << _cached_trajectory[i].theta << std::endl;
    }

    for (unsigned int i = 0; i < _cached_time_diffs.size(); ++i)
    {
        VertexDouble *vertex = new VertexDouble();
        vertex->setId(id_counter++);
        vertex->setEstimate(_cached_time_diffs[i]);
        _optimizer->addVertex(vertex);
        _time_diffs.emplace_back(vertex);
        // std::cout << "time_diff: " << _cached_time_diffs[i] << std::endl;
    }
}

void TebOptimizer::getVertices()
{
    _cached_trajectory.clear();
    _cached_time_diffs.clear();
    for (unsigned int i = 0; i < _trajectory.size(); ++i)
    {
        g2o::VertexSE2 *vertex = _trajectory[i];
        g2o::SE2 pose = vertex->estimate();
        _cached_trajectory.emplace_back(pose.translation().x(), pose.translation().y(), pose.rotation().angle());
        // std::cout << "pose: " << pose.translation().x() << " " << pose.translation().y() << " " << pose.rotation().angle() << std::endl;
    }
    for (unsigned int i = 0; i < _time_diffs.size(); ++i)
    {
        VertexDouble *vertex = _time_diffs[i];
        _cached_time_diffs.emplace_back(vertex->estimate());
        // std::cout << "time_diff: " << vertex->estimate() << std::endl;
    }
}
void TebOptimizer::buildGraph(double weight_multiplier)
{
    addVelocityEdges();
    addKinematicsEdges();
    addAccelerationEdges();
    addShortestPathEdges();
    addTimeOptimalEdges();
    addObstacleEdges();
}

void TebOptimizer::clearGraph()
{
    _optimizer->clear();
}

bool TebOptimizer::optimizeGraph()
{
    _optimizer->setVerbose(false);
    _optimizer->initializeOptimization();
    int iter = _optimizer->optimize(_follower_info->no_inner_iterations);
    return iter > 0;
}
void TebOptimizer::addVelocityEdges()
{

    Eigen::Matrix2d info = Eigen::Matrix2d::Identity();
    info(0, 0) = _follower_info->weight_max_vel_x;
    info(1, 1) = _follower_info->weight_max_vel_theta;
    for (unsigned int i = 0; i < _cached_time_diffs.size(); ++i)
    {
        EdgeVelocity *edge = new EdgeVelocity();
        edge->setInformation(info);
        edge->setVertex(0, _trajectory[i]);
        edge->setVertex(1, _trajectory[i + 1]);
        edge->setVertex(2, _time_diffs[i]);
        VelocityMessurement messurement(-_follower_info->max_vel_x_backwards, _follower_info->max_vel_x,
                                        -_follower_info->max_vel_theta, _follower_info->max_vel_theta, 0.1);
        edge->setMeasurement(messurement);
        _optimizer->addEdge(edge);
    }
}
void TebOptimizer::addKinematicsEdges()
{
    Eigen::Matrix2d info = Eigen::Matrix2d::Identity();
    info(0, 0) = _follower_info->weight_kinematics_smooth;
    info(1, 1) = _follower_info->weight_kinematics_turning_radius;
    for (unsigned int i = 0; i < _trajectory.size() - 1; i++)
    {
        EdgeKinematicCarLike *edge = new EdgeKinematicCarLike();
        edge->setInformation(info);
        edge->setVertex(0, _trajectory[i]);
        edge->setVertex(1, _trajectory[i + 1]);
        edge->setMeasurement(KinematicMessurement{_follower_info->min_turning_radius});
        _optimizer->addEdge(edge);
    }
}
void TebOptimizer::addAccelerationEdges()
{
    Eigen::Matrix2d info = Eigen::Matrix2d::Identity();
    info(0, 0) = _follower_info->weight_max_acc_x;
    info(1, 1) = _follower_info->weight_max_acc_theta;

    EdgeAccelerationStart *edge_start = new EdgeAccelerationStart();
    edge_start->setInformation(info);
    edge_start->setVertex(0, _trajectory[0]);
    edge_start->setVertex(1, _trajectory[1]);
    edge_start->setVertex(2, _time_diffs[0]);
    AccelerationMessurement messurement(_follower_info->max_acc_x, _follower_info->max_acc_theta, 0.1, 0.1);
    messurement.cur_vel_lin = _cached_start_vel.x;
    messurement.cur_vel_theta = _cached_start_vel.theta;
    edge_start->setMeasurement(messurement);
    _optimizer->addEdge(edge_start);

    for (unsigned int i = 0; i < _trajectory.size() - 2; ++i)
    {
        EdgeAcceleration *edge = new EdgeAcceleration();
        edge->setInformation(info);
        edge->setVertex(0, _trajectory[i]);
        edge->setVertex(1, _trajectory[i + 1]);
        edge->setVertex(2, _trajectory[i + 2]);
        edge->setVertex(3, _time_diffs[i]);
        edge->setVertex(4, _time_diffs[i + 1]);
        edge->setMeasurement(AccelerationMessurement{_follower_info->max_acc_x, _follower_info->max_acc_theta, 0.1, 0.1});
        _optimizer->addEdge(edge);
    }
}
void TebOptimizer::addObstacleEdges()
{
    double inscribe_radius = _robot_model->inscribedRadius();
    double circum_radius = _robot_model->circumRadius();
    double search_radius = (inscribe_radius + _follower_info->min_obstacle_dist) * _follower_info->obstacle_search_factory;
    search_radius = std::max(search_radius, (circum_radius + _follower_info->obstacle_inflation_dist) * 1.5);
    std::vector<nanoflann::ResultItem<uint32_t, double>> results;
    std::vector<Point2D> &obstacle_points = _obstacles.mutablePoints();

    Eigen::Matrix2d info = Eigen::Matrix2d::Identity();
    info(0, 0) = _follower_info->weight_obstacle_collision;
    info(1, 1) = _follower_info->weight_obstacle_inflation;
    for (unsigned int i = 0; i < _trajectory.size(); ++i)
    {
        results.clear();
        double query_points[2] = {_trajectory[i]->estimate().translation().x(), _trajectory[i]->estimate().translation().y()};
        _kdtree.radiusSearch(query_points, search_radius, results);
        int num_obstacles = std::min(int(results.size()), _follower_info->max_obstacle_num_per_node);
        if (num_obstacles < 1)
            continue;
        for (nanoflann::ResultItem<uint32_t, double> &item : results)
        {
            Point2D *obs_ptr = &(obstacle_points[item.first]);
            ObstacleMessurement messurement(
                _follower_info->min_obstacle_dist,
                _follower_info->obstacle_inflation_dist,
                0.05,
                0.1,
                _robot_model,
                obs_ptr);
            EdgeObstacle *edge = new EdgeObstacle();
            edge->setInformation(info);
            edge->setMeasurement(messurement);
            edge->setVertex(0, _trajectory[i]);
            _optimizer->addEdge(edge);
        }
    }
}
void TebOptimizer::addShortestPathEdges()
{
    Eigen::Matrix<double, 1, 1> info = Eigen::Matrix<double, 1, 1>::Identity() * _follower_info->weight_shortest_path;

    for (unsigned int i = 0; i < _trajectory.size() - 1; ++i)
    {
        EdgeShortestPath *edge = new EdgeShortestPath();
        edge->setInformation(info);
        edge->setVertex(0, _trajectory[i]);
        edge->setVertex(1, _trajectory[i + 1]);
        _optimizer->addEdge(edge);
    }
}
void TebOptimizer::addTimeOptimalEdges()
{
    Eigen::Matrix<double, 1, 1> info = Eigen::Matrix<double, 1, 1>::Identity() * _follower_info->weight_optimal_time;
    for (unsigned int i = 0; i < _time_diffs.size() - 1; ++i)
    {
        EdgeTimeOptimal *edge = new EdgeTimeOptimal();
        edge->setInformation(info);
        edge->setVertex(0, _time_diffs[i]);
        _optimizer->addEdge(edge);
    }
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
void TebOptimizer::clearObstacles()
{
    _obstacles.clear();
}
void TebOptimizer::addObstacles(const double &x, const double &y)
{
    _obstacles.addPoint(x, y);
}
void TebOptimizer::rebuildKDTree()
{
    _kdtree.buildIndex();
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