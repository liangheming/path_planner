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