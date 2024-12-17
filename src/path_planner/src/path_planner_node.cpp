#include "path_planner_ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh("~");
    PathPlannerROS path_planner(nh);
    ros::spin();
    return 0;
}