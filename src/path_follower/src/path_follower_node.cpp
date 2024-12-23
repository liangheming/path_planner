#include "path_follower_ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_follower");
    ros::NodeHandle nh("~");
    PathFollowerROS path_follower_ros(nh);
    ros::spin();
    return 0;
}