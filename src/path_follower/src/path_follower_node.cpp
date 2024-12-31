#include "path_follower_ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_follower");
    ros::NodeHandle private_nh("~");
    PathFollowerROS path_follower(private_nh);
    ros::spin();
    ros::shutdown();
    return 0;
}