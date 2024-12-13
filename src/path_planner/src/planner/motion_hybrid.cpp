#include "motion_hybrid.h"

void MotionHybrid::init(const SearchInfo &search_info)
{
    size_x = search_info.size_x;
    size_y = search_info.size_y;
    size_theta = search_info.size_theta;
    size_theta_float = static_cast<float>(size_theta);

    cost_penalty = search_info.cost_penalty;
    change_penalty = search_info.change_penalty;
    reverse_penalty = search_info.reverse_penalty;
    non_straight_penalty = search_info.non_straight_penalty;
    minimum_turning_radius = search_info.minimum_turning_radius;
    bin_size = 2.0f * static_cast<float>(M_PI) / size_theta_float;

    if (search_info.motion_type == MotionType::Dubins)
        initDubins();
    else if (search_info.motion_type == MotionType::ReedsShepp)
        initReedsShepp();
    else
        throw std::runtime_error("Motion type not supported");
}
void MotionHybrid::initDubins()
{
    float angle = 2.0 * asin(sqrt(2.0) / (2 * minimum_turning_radius));
    float increments = 1.0f;
    if (angle > bin_size)
        increments = ceil(angle / bin_size);
    angle = bin_size * increments;

    float delta_x = minimum_turning_radius * sin(angle);
    float delta_y = minimum_turning_radius * (1.0f - cos(angle));
    projections.clear();
    projections.reserve(3);
    projections.emplace_back(hypotf(delta_x, delta_y), 0.0, 0.0);
    projections.emplace_back(delta_x, delta_y, increments);
    projections.emplace_back(delta_x, -delta_y, -increments);

    state_space = std::make_unique<ompl::base::DubinsStateSpace>(minimum_turning_radius);
}
void MotionHybrid::initReedsShepp()
{
    float angle = 2.0 * asin(sqrt(2.0) / (2 * minimum_turning_radius));
    float increments = 1.0f;
    if (angle > bin_size)
        increments = ceil(angle / bin_size);
    angle = bin_size * increments;
    float delta_x = minimum_turning_radius * sin(angle);
    float delta_y = minimum_turning_radius * (1.0f - cos(angle));
    // float min_translation = hypotf(delta_x, delta_y);
    // std::cout << "minimum_turning_radius: " << minimum_turning_radius << std::endl;
    // std::cout << "Minimum translation: " << min_translation << std::endl;
    projections.clear();
    projections.reserve(6);
    projections.emplace_back(hypotf(delta_x, delta_y), 0.0, 0.0);  // Forward
    projections.emplace_back(delta_x, delta_y, increments);        // Forward + Left
    projections.emplace_back(delta_x, -delta_y, -increments);      // Forward + Right
    projections.emplace_back(-hypotf(delta_x, delta_y), 0.0, 0.0); // Backward
    projections.emplace_back(-delta_x, delta_y, -increments);      // Backward + Left
    projections.emplace_back(-delta_x, -delta_y, increments);      // Backward + Right
    state_space = std::make_unique<ompl::base::ReedsSheppStateSpace>(minimum_turning_radius);
}
void MotionHybrid::normalizeHeading(float &heading)
{
    while (heading >= size_theta_float)
        heading -= size_theta_float;
    while (heading < 0.0)
        heading += size_theta_float;
}
Motion MotionHybrid::getProjection(NodeHybrid *node, unsigned int motion_index)
{
    const Motion &motion_model = projections[motion_index];
    const float &node_heading = node->coordinate().theta;
    const float cos_theta = cos(node_heading * bin_size);
    const float sin_theta = sin(node_heading * bin_size);
    const float delta_x = motion_model.x * cos_theta - motion_model.y * sin_theta;
    const float delta_y = motion_model.x * sin_theta + motion_model.y * cos_theta;
    float new_heading = node_heading + motion_model.theta;
    normalizeHeading(new_heading);
    return Motion(delta_x + node->coordinate().x, delta_y + node->coordinate().y, new_heading);
}
Motions MotionHybrid::getProjections(NodeHybrid *node)
{
    Motions projection_list;
    for (unsigned int i = 0; i < projections.size(); i++)
        projection_list.push_back(getProjection(node, i));
    return projection_list;
}
unsigned int MotionHybrid::getIndex(const unsigned int &x, const unsigned int &y, const unsigned int &theta)
{
    return theta + x * size_theta + y * size_theta * size_x;
}
Coordinate MotionHybrid::getCoordinate(const unsigned int &index)
{
    unsigned int x_idx = (index / size_theta) % size_x;
    unsigned int y_idx = index / (size_theta * size_x);
    unsigned theta_idx = index % size_theta;
    return Coordinate{static_cast<float>(x_idx), static_cast<float>(y_idx), static_cast<float>(theta_idx)};
}