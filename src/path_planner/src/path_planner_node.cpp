#include "path_planner_ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh("~");
    PathPlannerROS pp(nh);
    ros::spin();
    return 0;
}