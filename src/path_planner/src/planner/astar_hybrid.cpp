#include "astar_hybrid.h"

AstarHybrid::AstarHybrid(SearchInfo *search_info, CollisionChecker *collision_checker)
    : _search_info(search_info), _collision_checker(collision_checker),
      _start(nullptr), _goal(nullptr)
{
    assert(_collision_checker->getCostMap() != nullptr);
    _search_info->minimum_turning_radius_in_cell = ceil(search_info->minimum_turning_radius / static_cast<float>(_collision_checker->getCostMap()->getResolution()));
    _search_info->size_x = _collision_checker->getCostMap()->getSizeInCellsX();
    _search_info->size_y = _collision_checker->getCostMap()->getSizeInCellsY();
    _search_info->tolerance_in_cell = ceil(search_info->tolerance / static_cast<float>(_collision_checker->getCostMap()->getResolution()));
    _search_info->analytic_expansion_ratio_in_cell = ceil(search_info->analytic_expansion_ratio / static_cast<float>(_collision_checker->getCostMap()->getResolution()));
    _search_info->analytic_expansion_tolerance_in_cell = ceil(search_info->analytic_expansion_tolerance / static_cast<float>(_collision_checker->getCostMap()->getResolution()));
    _motion_table.init(*_search_info);
}
void AstarHybrid::resetIterations(unsigned int &max_iterations, unsigned int &max_on_approach_iterations)
{
    _search_info->max_iterations = max_iterations;
    _search_info->max_on_approach_iterations = max_on_approach_iterations;
}
void AstarHybrid::clearGraph()
{
    NodeGraph g;
    g.reserve(100000);
    std::swap(_graph, g);
}
void AstarHybrid::clearQueue()
{
    NodeQueue q;
    std::swap(_queue, q);
}
bool AstarHybrid::isInputValid()
{
    if (_start == nullptr || _goal == nullptr || _graph.empty())
        return false;
    float start_foot_print_cost, goal_foot_print_cost;
    if (_collision_checker->checkAndGetFootprintCost(_start->coordinate().x, _start->coordinate().y, _start->coordinate().theta * _motion_table.bin_size, start_foot_print_cost))
        return false;
    _start->cellCost() = start_foot_print_cost;
    if (_collision_checker->checkAndGetFootprintCost(_goal->coordinate().x, _goal->coordinate().y, _goal->coordinate().theta * _motion_table.bin_size, goal_foot_print_cost))
        return false;
    _goal->cellCost() = goal_foot_print_cost;
    return true;
}
bool AstarHybrid::setStart(const float &x_in_cell, const float &y_in_cell, const float &theta_in_cell)
{
    unsigned int x_idx = static_cast<unsigned int>(floor(x_in_cell));
    unsigned int y_idx = static_cast<unsigned int>(floor(y_in_cell));
    float mutable_theta_in_cell = theta_in_cell + _motion_table.size_theta_float;
    _motion_table.normalizeHeading(mutable_theta_in_cell);
    unsigned int theta_idx = static_cast<unsigned int>(floor(mutable_theta_in_cell));
    _start = addToGraph(_motion_table.getIndex(x_idx, y_idx, theta_idx));
    _start->coordinate() = {x_in_cell, y_in_cell, mutable_theta_in_cell};
    return true;
}
bool AstarHybrid::setGoal(const float &x_in_cell, const float &y_in_cell, const float &theta_in_cell)
{
    unsigned int x_idx = static_cast<unsigned int>(floor(x_in_cell));
    unsigned int y_idx = static_cast<unsigned int>(floor(y_in_cell));
    float mutable_theta_in_cell = theta_in_cell + _motion_table.size_theta_float;
    _motion_table.normalizeHeading(mutable_theta_in_cell);
    unsigned int theta_idx = static_cast<unsigned int>(floor(mutable_theta_in_cell));
    _goal = addToGraph(_motion_table.getIndex(x_idx, y_idx, mutable_theta_in_cell));
    _goal->coordinate() = {x_in_cell, y_in_cell, mutable_theta_in_cell};
    return true;
}
AstarHybrid::NodePtr AstarHybrid::addToGraph(unsigned int index)
{
    auto iter = _graph.find(index);
    if (iter != _graph.end())
        return &(iter->second);
    return &(_graph.emplace(index, Node(index)).first->second);
}
bool AstarHybrid::isGoal(NodePtr &node)
{
    return node == _goal;
}
void AstarHybrid::addToQueue(const float cost, NodePtr &node)
{
    _queue.emplace(cost, node);
}
float AstarHybrid::getMotionHeuristicCost(const Coordinate &start_coord, const Coordinate &goal_coord)
{
    ompl::base::ScopedState<> from(_motion_table.state_space), to(_motion_table.state_space);
    from[0] = static_cast<double>(start_coord.x);
    from[1] = static_cast<double>(start_coord.y);
    from[2] = normalize_angle(static_cast<double>(start_coord.theta * _motion_table.bin_size));
    to[0] = static_cast<double>(goal_coord.x);
    to[1] = static_cast<double>(goal_coord.y);
    to[2] = normalize_angle(static_cast<double>(goal_coord.theta * _motion_table.bin_size));
    return static_cast<float>(_motion_table.state_space->distance(from(), to()));
}
float AstarHybrid::distance2d(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2)
{
    return std::hypotf(static_cast<float>(x1) - static_cast<float>(x2), static_cast<float>(y1) - static_cast<float>(y2));
}
bool AstarHybrid::cacheObstacleHeuristic()
{
    const unsigned int size_x = _motion_table.size_x;
    int size_x_int = static_cast<int>(size_x);
    const unsigned int size_y = _motion_table.size_y;
    int size_y_int = static_cast<int>(size_y);
    unsigned int size = size_x * size_y;
    if (_obstacle_heuristic_lookup_table.size() != size)
        _obstacle_heuristic_lookup_table.resize(size);
    std::fill(_obstacle_heuristic_lookup_table.begin(), _obstacle_heuristic_lookup_table.end(), 0.0f);
    _obstacle_heuristic_queue.clear();
    _obstacle_heuristic_queue.reserve(size);
    unsigned int goal_x = static_cast<unsigned int>(_goal->coordinate().x);
    unsigned int goal_y = static_cast<unsigned int>(_goal->coordinate().y);
    const unsigned int goal_index = goal_y * size_x + goal_x;
    unsigned int start_x = static_cast<unsigned int>(_start->coordinate().x);
    unsigned int start_y = static_cast<unsigned int>(_start->coordinate().y);
    const unsigned int start_index = start_y * size_x + start_x;
    float distance_to_goal = distance2d(start_x, start_y, goal_x, goal_y);
    _obstacle_heuristic_queue.emplace_back(distance_to_goal, goal_index);
    _obstacle_heuristic_lookup_table[goal_index] = -0.00001f;
    std::make_heap(_obstacle_heuristic_queue.begin(), _obstacle_heuristic_queue.end(), ObstacleHeuristicComparator{});
    const std::vector<std::pair<int, int>> neighborhood = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}};
    unsigned int idx, mx, my, mx_idx, my_idx, new_idx = 0;
    float c_cost, cost, travel_cost, new_cost, existing_cost;
    float sqrt_2 = sqrtf(2.0f);
    while (!_obstacle_heuristic_queue.empty())
    {
        idx = _obstacle_heuristic_queue.front().second;
        std::pop_heap(
            _obstacle_heuristic_queue.begin(), _obstacle_heuristic_queue.end(),
            ObstacleHeuristicComparator{});
        _obstacle_heuristic_queue.pop_back();
        c_cost = _obstacle_heuristic_lookup_table[idx];
        if (c_cost > 0.0f)
            continue;
        c_cost = -c_cost;
        _obstacle_heuristic_lookup_table[idx] = c_cost;
        my_idx = idx / size_x;
        mx_idx = idx - (my_idx * size_x);
        for (unsigned int i = 0; i != neighborhood.size(); i++)
        {
            mx = mx_idx + neighborhood[i].first;
            my = my_idx + neighborhood[i].second;
            if (mx <= 0 || mx >= size_x_int - 1 || my <= 0 || my >= size_y_int - 1)
                continue;
            cost = _collision_checker->getCost(mx, my);
            if (cost >= 253.0)
                continue;
            new_idx = my * size_x + mx;
            existing_cost = _obstacle_heuristic_lookup_table[new_idx];
            // 没有遍历过
            if (existing_cost <= 0.0f)
            {
                travel_cost = ((i <= 3) ? 1.0f : sqrt_2) * (1.0f + (_search_info->cost_penalty * cost / 252.0f));
                new_cost = c_cost + travel_cost;
                if (existing_cost == 0.0f || -existing_cost > new_cost)
                {
                    _obstacle_heuristic_lookup_table[new_idx] = -new_cost;
                    _obstacle_heuristic_queue.emplace_back(new_cost + distance2d(mx, my, start_x, start_y), new_idx);
                    std::push_heap(_obstacle_heuristic_queue.begin(), _obstacle_heuristic_queue.end(), ObstacleHeuristicComparator{});
                }
            }
        }
        if (idx == start_index)
            break;
    }
    return idx == start_index;
}
float AstarHybrid::getObstacleHeuristic(const Coordinate &start_coord, const Coordinate &goal_coord)
{
    const unsigned int size_x = _motion_table.size_x;
    const unsigned int size_y = _motion_table.size_y;
    const unsigned int start_x = static_cast<unsigned int>(start_coord.x);
    const unsigned int start_y = static_cast<unsigned int>(start_coord.y);
    const unsigned int start_index = start_y * size_x + start_x;
    const float &requested_node_cost = _obstacle_heuristic_lookup_table[start_index];

    int start_x_int = static_cast<int>(start_coord.x), start_y_int = static_cast<int>(start_coord.y);
    int goal_x_int = static_cast<int>(goal_coord.x), goal_y_int = static_cast<int>(goal_coord.y);
    int x_abs_distance = abs(start_x_int - goal_x_int);
    int y_abs_distance = abs(start_y_int - goal_y_int);
    const float sqrt_2 = sqrtf(2.0f);
    float diagonal_distance = std::min(x_abs_distance, y_abs_distance) * sqrt_2 + std::abs(x_abs_distance - y_abs_distance);

    if (requested_node_cost > 0.0f)
        return std::max(requested_node_cost, diagonal_distance);
    for (auto &n : _obstacle_heuristic_queue)
    {
        unsigned int c_idx = n.second;
        unsigned int c_y = c_idx / size_x;
        unsigned int c_x = c_idx - (c_y * size_x);
        n.first = -_obstacle_heuristic_lookup_table[n.second] + distance2d(c_x, c_y, start_x, start_y);
    }
    std::make_heap(_obstacle_heuristic_queue.begin(), _obstacle_heuristic_queue.end(), ObstacleHeuristicComparator{});
    const int size_x_int = static_cast<int>(size_x);
    const int size_y_int = static_cast<int>(size_y);

    unsigned int idx, mx, my, mx_idx, my_idx, new_idx = 0;
    float c_cost, cost, travel_cost, new_cost, existing_cost;
    const std::vector<std::pair<int, int>> neighborhood = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}};

    while (!_obstacle_heuristic_queue.empty())
    {
        idx = _obstacle_heuristic_queue.front().second;
        std::pop_heap(
            _obstacle_heuristic_queue.begin(), _obstacle_heuristic_queue.end(),
            ObstacleHeuristicComparator{});
        _obstacle_heuristic_queue.pop_back();
        c_cost = _obstacle_heuristic_lookup_table[idx];
        if (c_cost > 0.0f)
            continue;
        c_cost = -c_cost;
        _obstacle_heuristic_lookup_table[idx] = c_cost;
        my_idx = idx / size_x;
        mx_idx = idx - (my_idx * size_x);
        for (unsigned int i = 0; i != neighborhood.size(); i++)
        {
            mx = mx_idx + neighborhood[i].first;
            my = my_idx + neighborhood[i].second;
            if (mx <= 0 || mx >= size_x_int - 1 || my <= 0 || my >= size_y_int - 1)
                continue;
            cost = _collision_checker->getCost(mx, my);
            if (cost >= 253.0)
                continue;
            new_idx = my * size_x + mx;
            existing_cost = _obstacle_heuristic_lookup_table[new_idx];
            // 没有遍历过
            if (existing_cost <= 0.0f)
            {
                travel_cost = ((i <= 3) ? 1.0f : sqrt_2) * (1.0f + (_search_info->cost_penalty * cost / 252.0f));
                new_cost = c_cost + travel_cost;
                if (existing_cost == 0.0f || -existing_cost > new_cost)
                {
                    _obstacle_heuristic_lookup_table[new_idx] = -new_cost;
                    _obstacle_heuristic_queue.emplace_back(new_cost + distance2d(mx, my, start_x, start_y), new_idx);
                    std::push_heap(_obstacle_heuristic_queue.begin(), _obstacle_heuristic_queue.end(), ObstacleHeuristicComparator{});
                }
            }
        }
        if (idx == start_index)
            break;
    }
    return std::max(requested_node_cost, diagonal_distance);
}
float AstarHybrid::getHeuristicCost(const Coordinate &start_coord, const Coordinate &goal_coord)
{
    float motion_heuristic_cost = getMotionHeuristicCost(start_coord, goal_coord);
    float obstacle_heuristic_cost = getObstacleHeuristic(start_coord, goal_coord);
    return Node::NeutralCost * std::max(motion_heuristic_cost, obstacle_heuristic_cost);
}
bool AstarHybrid::backtracePath(NodePtr &node, Coordinates &path)
{
    if (!node->parent)
    {
        return false;
    }
    NodePtr current_node = node;

    while (current_node->parent)
    {
        path.push_back(current_node->coordinate());
        current_node = current_node->parent;
    }
    path.push_back(current_node->coordinate());
    return path.size() > 1;
}
void AstarHybrid::getNeighbors(NodePtr &node, NodePtrVector &neighbors)
{
    neighbors.clear();
    unsigned int index, m_x, m_y, m_theta;
    NodePtr neighbor = nullptr;
    Motions motion_projections = _motion_table.getProjections(node);
    for (unsigned int i = 0; i != motion_projections.size(); i++)
    {
        Motion &motion = motion_projections[i];
        unsigned int m_x = static_cast<unsigned int>(motion.x);
        unsigned int m_y = static_cast<unsigned int>(motion.y);
        unsigned int m_theta = static_cast<unsigned int>(motion.theta);
        if (m_x <= 0 || m_x >= _motion_table.size_x - 1 || m_y <= 0 || m_y >= _motion_table.size_y - 1)
            continue;
        index = _motion_table.getIndex(m_x, m_y, m_theta);
        neighbor = addToGraph(index);
        if (neighbor->visited())
            continue;
        float footprint_cost;
        bool is_collision = _collision_checker->checkAndGetFootprintCost(motion.x, motion.y, motion.theta * _motion_table.bin_size, footprint_cost);
        if (is_collision)
            continue;
        neighbor->cellCost() = footprint_cost;
        neighbor->coordinate() = {motion.x, motion.y, motion.theta};
        neighbor->motionIndex() = i;
        neighbors.push_back(neighbor);
    }
}

AstarHybrid::NodePtr AstarHybrid::getAnalyticExpansion(NodePtr &current_node)
{
    const Coordinate &node_coord = current_node->coordinate();
    const Coordinate &goal_coord = _goal->coordinate();
    ompl::base::ScopedState<> from(_motion_table.state_space), to(_motion_table.state_space), s(_motion_table.state_space);
    from[0] = static_cast<double>(node_coord.x);
    from[1] = static_cast<double>(node_coord.y);
    from[2] = normalize_angle(static_cast<double>(node_coord.theta * _motion_table.bin_size));
    to[0] = static_cast<double>(goal_coord.x);
    to[1] = static_cast<double>(goal_coord.y);
    to[2] = normalize_angle(static_cast<double>(goal_coord.theta * _motion_table.bin_size));
    float motion_distance = _motion_table.state_space->distance(from(), to());
    float sqrt_2 = sqrtf(2.0f);
    // std::cout << "from " << from[0] << " " << from[1] << " " << from[2] << std::endl;
    // std::cout << "to  " << to[0] << " " << to[1] << " " << to[2] << std::endl;

    std::vector<Node> possible_nodes;
    std::vector<double> reals;
    float angle = 0.0;
    float num_intervals = std::floor(motion_distance / sqrt_2);
    unsigned int x_idx, y_idx;

    for (float i = 1; i < num_intervals; i += 1.0f)
    {
        _motion_table.state_space->interpolate(from(), to(), i / num_intervals, s());
        reals = s.reals();
        angle = static_cast<float>(reals[2]) / _motion_table.bin_size + _motion_table.size_theta_float;
        _motion_table.normalizeHeading(angle);
        x_idx = static_cast<unsigned int>(reals[0]);
        y_idx = static_cast<unsigned int>(reals[1]);
        if (x_idx <= 0 || x_idx >= _motion_table.size_x - 1 || y_idx <= 0 || y_idx >= _motion_table.size_y - 1)
            return NodePtr(nullptr);
        if (_collision_checker->checkFootPrintCollision(static_cast<float>(reals[0]), static_cast<float>(reals[1]), static_cast<float>(reals[2])))
            return NodePtr(nullptr);
        possible_nodes.emplace_back(_motion_table.getIndex(x_idx, y_idx, static_cast<unsigned int>(angle)));
        possible_nodes.back().coordinate() = {static_cast<float>(reals[0]), static_cast<float>(reals[1]), angle};
    }

    if (possible_nodes.size() == 0)
        return NodePtr(nullptr);
    NodePtr prev = current_node;
    for (Node &node : possible_nodes)
    {
        NodePtr neighbor = addToGraph(node.index());
        if (neighbor->visited())
            continue;
        neighbor->parent = prev;
        neighbor->visited() = true;
        neighbor->coordinate() = node.coordinate();
        prev = neighbor;
    }
    if (prev != _goal)
    {
        _goal->parent = prev;
        _goal->visited() = true;
    }
    return _goal;
}

AstarHybrid::NodePtr AstarHybrid::tryAnalyticExpansion(NodePtr &current_node, int &analytic_iterations, int &closest_distance)
{
    closest_distance = std::min(static_cast<int>(getHeuristicCost(current_node->coordinate(), _goal->coordinate()) / Node::NeutralCost), closest_distance);
    int desired_iterations = std::max(
        static_cast<int>(closest_distance / _search_info->analytic_expansion_ratio_in_cell),
        static_cast<int>(std::ceil(_search_info->analytic_expansion_ratio_in_cell)));
    analytic_iterations = std::min(analytic_iterations, desired_iterations);
    if (analytic_iterations <= 0)
    {
        analytic_iterations = desired_iterations;
        return getAnalyticExpansion(current_node);
    }
    analytic_iterations--;
    return NodePtr(nullptr);
}

float AstarHybrid::getTravelCost(NodePtr &from, NodePtr &to)
{
    const float normalized_cost = to->cellCost() / 252.0;
    assert(!std::isnan(normalized_cost));
    if (from->motionIndex() == std::numeric_limits<unsigned int>::max())
        return Node::NeutralCost;
    float travel_cost = 0.0;
    float travel_cost_raw = Node::NeutralCost + _search_info->cost_penalty * normalized_cost;
    if (to->motionIndex() == 0 || to->motionIndex() == 3)
    {
        travel_cost = travel_cost_raw;
    }
    else
    {
        if (from->motionIndex() == to->motionIndex())
            travel_cost = travel_cost_raw * _search_info->non_straight_penalty;
        else
            travel_cost = travel_cost_raw * (_search_info->change_penalty + _search_info->non_straight_penalty);
    }
    if (to->motionIndex() > 2)
        travel_cost *= _search_info->reverse_penalty;
    return travel_cost;
}

bool AstarHybrid::createPath(const Coordinate &start, const Coordinate &goal, Coordinates &path, int &iterations)
{
    unsigned int s_x_in_cell, s_y_in_cell, g_x_in_cell, g_y_in_cell;
    float s_theta_in_cell, g_theta_in_cell;

    float original_x = static_cast<float>(_collision_checker->getCostMap()->getOriginX());
    float original_y = static_cast<float>(_collision_checker->getCostMap()->getOriginY());
    float resolution = static_cast<float>(_collision_checker->getCostMap()->getResolution());

    if (!_collision_checker->getCostMap()->worldToMap(start.x, start.y, s_x_in_cell, s_y_in_cell))
        return false;

    s_theta_in_cell = start.theta / _motion_table.bin_size;
    if (!_collision_checker->getCostMap()->worldToMap(goal.x, goal.y, g_x_in_cell, g_y_in_cell))
        return false;

    g_theta_in_cell = goal.theta / _motion_table.bin_size;

    clearGraph();
    float x_in_cell, y_in_cell;
    x_in_cell = (start.x - original_x) / resolution;
    y_in_cell = (start.y - original_y) / resolution;

    if (!setStart(x_in_cell, y_in_cell, s_theta_in_cell))
        return false;
    x_in_cell = (goal.x - original_x) / resolution;
    y_in_cell = (goal.y - original_y) / resolution;
    if (!setGoal(x_in_cell, y_in_cell, g_theta_in_cell))
        return false;
    Coordinates temp_path;
    bool search_success = createPath(temp_path, iterations);
    if (search_success)
        path = fromCellToWorld(temp_path);
    return search_success;
}
bool AstarHybrid::createPath(Coordinates &path, int &iterations)
{
    std::chrono::time_point<std::chrono::high_resolution_clock> tic_time, toc_time;
    tic_time = std::chrono::high_resolution_clock::now();
    if (!cacheObstacleHeuristic())
        return false;
    clearQueue();
    iterations = 0;
    int approach_iterations = 0;
    int analytic_iterations = 0;
    int closest_distance = std::numeric_limits<int>::max();
    float tolerance = _search_info->tolerance_in_cell * Node::NeutralCost;
    std::pair<float, unsigned int> best_heuristic_node = {std::numeric_limits<float>::max(), 0};
    if (!isInputValid())
        return false;
    addToQueue(0.0, _start);
    _start->accumulatedCost() = 0.0;

    NodePtr current_node = nullptr;
    NodePtrVector neighbors;
    NodePtr neighbor = nullptr;
    float g_cost = 0.0;

    while (iterations < _search_info->max_iterations && !_queue.empty())
    {
        float queue_cost = _queue.top().first;
        current_node = _queue.top().second;
        _queue.pop();
        if (current_node->visited())
            continue;

        iterations++;
        current_node->visited() = true;
        toc_time = std::chrono::high_resolution_clock::now();

        if (std::chrono::duration_cast<std::chrono::milliseconds>(toc_time - tic_time).count() > _search_info->max_millsec)
            return false;

        if (best_heuristic_node.first < _search_info->analytic_expansion_tolerance_in_cell)
        {
            NodePtr result = tryAnalyticExpansion(current_node, analytic_iterations, closest_distance);

            if (result != nullptr)
            {
                current_node = result;
            }
        }

        if (isGoal(current_node))
        {
            bool ret = backtracePath(current_node, path);
            return ret;
        }
        else if (best_heuristic_node.first < tolerance)
        {
            approach_iterations++;
            if (approach_iterations > _search_info->max_on_approach_iterations || iterations + 1 > _search_info->max_iterations)
            {
                NodePtr node = &(_graph.at(best_heuristic_node.second));
                _goal->parent = node;
                bool ret = backtracePath(_goal, path);
                return ret;
            }
        }
        getNeighbors(current_node, neighbors);
        for (unsigned int i = 0; i < neighbors.size(); i++)
        {
            neighbor = neighbors[i];
            g_cost = current_node->accumulatedCost() + getTravelCost(current_node, neighbor);
            if (g_cost < neighbor->accumulatedCost())
            {
                neighbor->accumulatedCost() = g_cost;
                neighbor->parent = current_node;
                float heuristic_cost = getHeuristicCost(neighbor->coordinate(), _goal->coordinate());
                if (heuristic_cost < best_heuristic_node.first)
                    best_heuristic_node = {heuristic_cost, neighbor->index()};
                addToQueue(g_cost + heuristic_cost, neighbor);
            }
        }
    }
    return false;
}
Coordinates AstarHybrid::fromCellToWorld(const Coordinates &path)
{
    Coordinates world_paths;
    for (const Coordinate &coord : path)
    {
        double wx, wy;
        _collision_checker->getCostMap()->mapToWorld(static_cast<unsigned int>(coord.x), static_cast<unsigned int>(coord.y), wx, wy);
        float nx = static_cast<float>(wx);
        float ny = static_cast<float>(wy);
        float ntheta = normalize_angle(coord.theta * _motion_table.bin_size);
        world_paths.emplace_back(nx, ny, ntheta);
    }
    return world_paths;
}