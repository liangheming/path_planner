#include "smoother.h"

Smoother::Smoother(SmoothInfo *smooth_info, CollisionChecker *collision_checker) : _collision_checker(collision_checker), _smooth_info(smooth_info)
{
    _costmap = collision_checker->getCostMap();
    assert(_costmap != nullptr);
    if (_smooth_info->motion_type == MotionType::Dubins)
        _state_space = std::make_shared<ompl::base::DubinsStateSpace>(_smooth_info->minimum_turning_radius);
    else if (_smooth_info->motion_type == MotionType::ReedsShepp)
        _state_space = std::make_shared<ompl::base::ReedsSheppStateSpace>(_smooth_info->minimum_turning_radius);
    else
        throw std::invalid_argument("Motion type not supported");
}

unsigned int Smoother::findShortestBoundaryExpansionIdx(const BoundaryExpansions &boundary_expansions)
{
    double min_length = 1e9;
    int shortest_boundary_expansion_idx = 1e9;
    for (unsigned int idx = 0; idx != boundary_expansions.size(); idx++)
    {
        if (boundary_expansions[idx].expansion_path_length < min_length &&
            !boundary_expansions[idx].is_collision &&
            boundary_expansions[idx].path_end_idx > 0.0 &&
            boundary_expansions[idx].expansion_path_length > 0.0)
        {
            min_length = boundary_expansions[idx].expansion_path_length;
            shortest_boundary_expansion_idx = idx;
        }
    }

    return shortest_boundary_expansion_idx;
}

void Smoother::findBoundaryExpansion(const Coordinate &start, const Coordinate &end, BoundaryExpansion &expansion)
{
    ompl::base::ScopedState<> from(_state_space), to(_state_space), s(_state_space);
    from[0] = start.x;
    from[1] = start.y;
    from[2] = start.theta;
    to[0] = end.x;
    to[1] = end.y;
    to[2] = end.theta;
    double d = _state_space->distance(from(), to());
    if (d > 2.0 * expansion.original_path_length)
        return;

    std::vector<double> reals;
    double theta(0.0), x(0.0), y(0.0);
    double x_m = static_cast<double>(start.x);
    double y_m = static_cast<double>(start.y);
    double path_end_idx_double = static_cast<double>(expansion.path_end_idx);
    for (double i = 0; i <= path_end_idx_double; i += 1)
    {
        _state_space->interpolate(from(), to(), i / path_end_idx_double, s());
        reals = s.reals();
        const double result = fmod(reals[2] + M_PI, 2.0 * M_PI);
        if (result <= 0.0)
            theta = result + M_PI;
        else
            theta = result - M_PI;
        x = reals[0];
        y = reals[1];
        unsigned int mx, my;
        if (!_costmap->worldToMap(x_m, y_m, mx, my) || _costmap->getCost(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            expansion.is_collision = true;
        expansion.expansion_path_length += hypot(x - x_m, y - y_m);
        x_m = x;
        y_m = y;
        expansion.pts.emplace_back(static_cast<float>(x), static_cast<float>(y), static_cast<float>(theta));
    }
}

template <typename IteratorT>
BoundaryExpansions Smoother::generateBoundaryExpansionPoints(IteratorT start, IteratorT end)
{
    std::vector<double> distances = {
        _smooth_info->minimum_turning_radius,             // Radius
        2.0 *  _smooth_info->minimum_turning_radius,       // Diameter
        M_PI *  _smooth_info->minimum_turning_radius,      // 50% Circumference
        2.0 * M_PI *  _smooth_info->minimum_turning_radius // Circumference
    };
    BoundaryExpansions boundary_expansions;
    boundary_expansions.resize(distances.size());
    float curr_dist = 0.0;
    float x_last = start->x;
    float y_last = start->y;
    unsigned int curr_dist_idx = 0;

    for (IteratorT iter = start; iter != end; iter++)
    {
        Coordinate coord = *iter;
        curr_dist += hypot(coord.x - x_last, coord.y - y_last);
        x_last = coord.x;
        y_last = coord.y;
        if (curr_dist >= distances[curr_dist_idx])
        {
            boundary_expansions[curr_dist_idx].path_end_idx = iter - start;
            boundary_expansions[curr_dist_idx].original_path_length = static_cast<double>(curr_dist);
            curr_dist_idx++;
        }
        if (curr_dist_idx >= distances.size())
            break;
    }
    return boundary_expansions;
}
void Smoother::enforceStartBoundaryConditions(const Coordinate &start_coord, Coordinates &path, const bool &is_reverse)
{
    BoundaryExpansions boundary_expansions = generateBoundaryExpansionPoints<ForwardPathIterator>(path.begin(), path.end());
    for (unsigned int i = 0; i != boundary_expansions.size(); i++)
    {
        BoundaryExpansion &expansion = boundary_expansions[i];
        if (expansion.path_end_idx == 0)
            continue;
        if (!is_reverse)
            findBoundaryExpansion(start_coord, path[expansion.path_end_idx], expansion);
        else
            findBoundaryExpansion(path[expansion.path_end_idx], start_coord, expansion);
    }
    unsigned int best_expansion_idx = findShortestBoundaryExpansionIdx(boundary_expansions);
    if (best_expansion_idx > boundary_expansions.size())
        return;
    BoundaryExpansion &best_expansion = boundary_expansions[best_expansion_idx];
    if (is_reverse)
        std::reverse(best_expansion.pts.begin(), best_expansion.pts.end());
    for (unsigned int i = 0; i != best_expansion.pts.size(); i++)
        path[i] = best_expansion.pts[i];
}
void Smoother::enforceEndBoundaryConditions(const Coordinate &end_coord, Coordinates &path, const bool &is_reverse)
{
    BoundaryExpansions boundary_expansions = generateBoundaryExpansionPoints<BackwardPathIterator>(path.rbegin(), path.rend());
    unsigned int expansion_starting_idx;
    for (unsigned int i = 0; i != boundary_expansions.size(); i++)
    {
        BoundaryExpansion &expansion = boundary_expansions[i];
        if (expansion.path_end_idx == 0)
            continue;
        expansion_starting_idx = path.size() - expansion.path_end_idx - 1;
        if (!is_reverse)
            findBoundaryExpansion(path[expansion_starting_idx], end_coord, expansion);
        else
            findBoundaryExpansion(end_coord, path[expansion_starting_idx], expansion);
    }
    unsigned int best_expansion_idx = findShortestBoundaryExpansionIdx(boundary_expansions);
    if (best_expansion_idx > boundary_expansions.size())
        return;
    BoundaryExpansion &best_expansion = boundary_expansions[best_expansion_idx];
    if (is_reverse)
        std::reverse(best_expansion.pts.begin(), best_expansion.pts.end());
    expansion_starting_idx = path.size() - best_expansion.path_end_idx - 1;
    for (unsigned int i = 0; i != best_expansion.pts.size(); i++)
        path[expansion_starting_idx + i] = best_expansion.pts[i];
}
bool Smoother::smoothPath(Coordinates &path)
{
    Segments path_segments = findDirectionSegments(path);
    Coordinates current_path;
    bool success = true, reversing_segment;
    for (unsigned int i = 0; i != path_segments.size(); i++)
    {
        if (path_segments[i].end - path_segments[i].start <= 5)
            continue;
        current_path.clear();
        std::copy(path.begin() + path_segments[i].start,
                  path.begin() + path_segments[i].end + 1,
                  std::back_inserter(current_path));
        const Coordinate start_pose = current_path.front();
        const Coordinate goal_pose = current_path.back();
        bool local_success = smoothSegment(current_path, reversing_segment);
        success = success && local_success;
        if (!_smooth_info->is_holonomic && local_success)
        {
            enforceStartBoundaryConditions(start_pose, current_path, reversing_segment);
            enforceEndBoundaryConditions(goal_pose, current_path, reversing_segment);
        }
        std::copy(
            current_path.begin(),
            current_path.end(),
            path.begin() + path_segments[i].start);
    }

    return success;
}

bool Smoother::smoothSegment(Coordinates &path, bool &is_reverse)
{
    unsigned int iterations = 0;
    double change_dist = _smooth_info->tolerance;
    Coordinates new_path = path;
    Coordinates last_path = path;
    unsigned int mx, my;

    while (change_dist >= _smooth_info->tolerance)
    {
        iterations += 1;
        change_dist = 0.0;
        if (iterations >= _smooth_info->max_iter)
        {
            path = last_path;
            updateApproximatePathOrientations(path, is_reverse);
            return false;
        }

        for (unsigned int i = 1; i != path.size() - 1; i++)
        {
            const Coordinate &origin_coord = path[i];
            Coordinate &cur_new_coord = new_path[i];
            Coordinate &pre_new_coord = new_path[i - 1];
            Coordinate &nex_new_coord = new_path[i + 1];
            Coordinate change_before = cur_new_coord;
            cur_new_coord.x += _smooth_info->w_data * (origin_coord.x - cur_new_coord.x) + _smooth_info->w_smooth * (nex_new_coord.x + pre_new_coord.x - 2.0 * cur_new_coord.x);
            cur_new_coord.y += _smooth_info->w_data * (origin_coord.y - cur_new_coord.y) + _smooth_info->w_smooth * (nex_new_coord.y + pre_new_coord.y - 2.0 * cur_new_coord.y);
            change_dist += abs(cur_new_coord.x - change_before.x);
            change_dist += abs(cur_new_coord.y - change_before.y);

            if (_costmap->worldToMap(cur_new_coord.x, cur_new_coord.y, mx, my))
            {
                unsigned char cost = _costmap->getCost(mx, my);
                if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
                {
                    path = last_path;
                    updateApproximatePathOrientations(path, is_reverse);
                    return false;
                }
            }
            else
            {
                path = last_path;
                updateApproximatePathOrientations(path, is_reverse);
                return false;
            }
        }
        last_path = new_path;
    }

    path = last_path;
    updateApproximatePathOrientations(path, is_reverse);
    return true;
}

Segments Smoother::findDirectionSegments(Coordinates &path)
{
    Segments segments;
    Segment cur_segment;
    cur_segment.start = 0;
    if (_smooth_info->is_holonomic)
    {
        cur_segment.end = path.size() - 1;
        segments.push_back(cur_segment);
        return segments;
    }
    for (unsigned int i = 1; i < path.size() - 1; i++)
    {
        float oa_x = path[i].x - path[i - 1].x;
        float oa_y = path[i].y - path[i - 1].y;
        float ab_x = path[i + 1].x - path[i].x;
        float ab_y = path[i + 1].y - path[i].y;
        float dot_prod = oa_x * ab_x + oa_y * ab_y;
        if (dot_prod < 0.0)
        {
            cur_segment.end = i;
            segments.push_back(cur_segment);
            cur_segment.start = i;
        }
        float dtheta = shortest_angular_distance(path[i].theta, path[i + 1].theta);
        if (fabs(ab_x) < 1e-4 && fabs(ab_y) < 1e-4 && fabs(dtheta) > 1e-4)
        {
            cur_segment.end = i;
            segments.push_back(cur_segment);
            cur_segment.start = i;
        }
    }
    cur_segment.end = path.size() - 1;
    segments.push_back(cur_segment);
    return segments;
}

void Smoother::updateApproximatePathOrientations(Coordinates &path, bool &is_reverse)
{
    float dx = path[2].x - path[1].x;
    float dy = path[2].y - path[1].y;
    float theta = atan2(dy, dx);
    float pt_yaw = path[1].theta;
    if (!_smooth_info->is_holonomic && fabs(shortest_angular_distance(pt_yaw, theta)) > M_PI_2)
        is_reverse = true;
    for (unsigned int i = 0; i != path.size() - 1; i++)
    {
        dx = path[i + 1].x - path[i].x;
        dy = path[i + 1].y - path[i].y;
        theta = atan2(dy, dx);
        if (fabs(dx) < 1e-4 && fabs(dy) < 1e-4)
            continue;
        if (is_reverse)
            theta += M_PI;
        path[i].theta = normalize_angle(theta);
    }
}
