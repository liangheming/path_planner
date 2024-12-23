#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_pub");
    ros::NodeHandle nh;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "move_base";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;
    ROS_INFO("tf_pub_once_node start");
    for (unsigned int i = 0; i < 10; i++)
    {
        transformStamped.header.stamp = ros::Time::now();
        br.sendTransform(transformStamped);
        ros::Duration(0.2).sleep();
    }
    ros::spin();
    return 0;
}