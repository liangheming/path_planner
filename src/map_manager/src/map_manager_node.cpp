#include "map_manager_ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_manager");
    ros::NodeHandle nh("~");
    MapManagerROS map_manager(nh);
    ros::spin();
    return 0;
}