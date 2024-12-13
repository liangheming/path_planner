#include "collision_checker.h"

void CollisionChecker::initBoundary(float radius)
{
    assert(radius > 0 && _costmap != nullptr);
    int radius_int = ceil(radius / _costmap->getResolution());
    for (int i = -radius_int; i <= radius_int; i++)
    {
        for (int j = -radius_int; j <= radius_int; j++)
        {
            if (i * i + j * j <= radius_int * radius_int)
                _points.emplace_back(i, j, 0.0f);
        }
    }
}
void CollisionChecker::initBoundary(float width, float height)
{
    assert(width > 0 && height > 0 && _costmap != nullptr);
    int half_width_int = ceil(0.5 * width / _costmap->getResolution());
    int half_height_int = ceil(0.5 * height / _costmap->getResolution());
    for (int i = -half_width_int; i <= half_width_int; i++)
    {
        for (int j = -half_height_int; j <= half_height_int; j++)
        {
            _points.emplace_back(i, j, 0.0f);
        }
    }
}
void CollisionChecker::initBoundary(float x_min, float y_min, float x_max, float y_max)
{
    assert(x_min <= 0 && y_min <= 0 && x_max >= 0 && y_max >= 0 && _costmap != nullptr);
    int x_min_int = floor(x_min / _costmap->getResolution());
    int y_min_int = floor(y_min / _costmap->getResolution());
    int x_max_int = ceil(x_max / _costmap->getResolution());
    int y_max_int = ceil(y_max / _costmap->getResolution());
    for (int i = x_min_int; i <= x_max_int; i++)
    {
        for (int j = y_min_int; j <= y_max_int; j++)
        {
            _points.emplace_back(i, j, 0.0f);
        }
    }
}
void CollisionChecker::initBoundary(std::vector<float> &boundary)
{
    assert(boundary.size() == 1 || boundary.size() == 2 || boundary.size() == 4);
    if (boundary.size() == 1)
        initBoundary(boundary[0]);
    if (boundary.size() == 2)
        initBoundary(boundary[0], boundary[1]);
    if (boundary.size() == 4)
        initBoundary(boundary[0], boundary[1], boundary[2], boundary[3]);
}

void CollisionChecker::printBoundary()
{
    std::cout << "=======================" << std::endl;
    if (_points.size() < 1)
        return;
    int x_min = std::numeric_limits<int>().max(), y_min = std::numeric_limits<int>().max(), x_max = std::numeric_limits<int>().min(), y_max = std::numeric_limits<int>().min();
    for (auto &point : _points)
    {
        if (point.x < x_min)
            x_min = point.x;
        if (point.y < y_min)
            y_min = point.y;
        if (point.x > x_max)
            x_max = point.x;
        if (point.y > y_max)
            y_max = point.y;
    }
    unsigned int size_y = y_max - y_min + 1, size_x = x_max - x_min + 1;
    char arr[size_y][size_x];
    std::fill_n(&arr[0][0], size_y * size_x, '-');
    for (auto &point : _points)
    {
        int x_idx = point.x - x_min;
        int y_idx = point.y - y_min;
        arr[y_idx][x_idx] = '*';
    }

    for (size_t i = 0; i < size_y; i++)
    {
        for (size_t j = 0; j < size_x; j++)
        {
            std::cout << arr[i][j] << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "=======================" << std::endl;
}
/**
 * x: costmap的坐标系的x
 * y: costmap的坐标系的y
 * theta: costmap的坐标系的theta
 */
bool CollisionChecker::checkCollision(const float &x, const float &y, const float &theta)
{
    assert(_costmap != nullptr && _points.size() > 0);
    unsigned int size_x = _costmap->getSizeInCellsX();
    unsigned int size_y = _costmap->getSizeInCellsY();
    for (auto &point : _points)
    {
        float x_new = x + point.x * cos(theta) - point.y * sin(theta);
        float y_new = y + point.x * sin(theta) + point.y * cos(theta);
        unsigned int x_idx = floor(x_new);
        unsigned int y_idx = floor(y_new);
        // 在地图范围内且碰到障碍物
        if (x_idx >= 0 && x_idx < size_x && y_idx >= 0 && y_idx < size_y && _costmap->getCost(x_idx, y_idx) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            return true;
    }
    return false;
}

float CollisionChecker::getFootprintCost(const float &x, const float &y, const float &theta)
{
    assert(_costmap != nullptr && _points.size() > 0);
    float temp_cost = 0.0f;
    unsigned char cell_cost;
    unsigned int size_x = _costmap->getSizeInCellsX();
    unsigned int size_y = _costmap->getSizeInCellsY();
    for (auto &point : _points)
    {
        float x_new = x + point.x * cos(theta) - point.y * sin(theta);
        float y_new = y + point.x * sin(theta) + point.y * cos(theta);
        unsigned int x_idx = floor(x_new);
        unsigned int y_idx = floor(y_new);
        // 在地图范围内且碰到障碍物
        if (x_idx >= 0 && x_idx < size_x && y_idx >= 0 && y_idx < size_y)
        {
            cell_cost = _costmap->getCost(x_idx, y_idx);
            if (cell_cost == costmap_2d::FREE_SPACE)
                continue;
            temp_cost = std::max(static_cast<float>(cell_cost), temp_cost);
        }
    }
    return temp_cost;
}

bool CollisionChecker::checkAndGetFootprintCost(const float &x, const float &y, const float &theta, float &cost)
{
    assert(_costmap != nullptr && _points.size() > 0);

    bool flag = false;
    float temp_cost = 0.0f;
    unsigned char cell_cost;
    unsigned int size_x = _costmap->getSizeInCellsX();
    unsigned int size_y = _costmap->getSizeInCellsY();
    for (auto &point : _points)
    {
        float x_new = x + point.x * cos(theta) - point.y * sin(theta);
        float y_new = y + point.x * sin(theta) + point.y * cos(theta);
        unsigned int x_idx = floor(x_new);
        unsigned int y_idx = floor(y_new);
        // 在地图范围内且碰到障碍物
        if (x_idx >= 0 && x_idx < size_x && y_idx >= 0 && y_idx < size_y)
        {
            cell_cost = _costmap->getCost(x_idx, y_idx);
            if (cell_cost == costmap_2d::FREE_SPACE)
                continue;
            if (cell_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
                flag = true;
            temp_cost = std::max(static_cast<float>(cell_cost), temp_cost);
        }
    }
    cost = temp_cost;
    return flag;
}