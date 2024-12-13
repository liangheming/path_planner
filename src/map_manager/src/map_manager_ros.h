#pragma once

#include <ros/ros.h>
#include <filesystem>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>

#include <std_srvs/Trigger.h>
// #include <zouyu_srvs/ChangeMap.h>
#include <yaml-cpp/yaml.h>


#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

struct NodeConfig
{
    std::string map_topic = "/map";
    std::string map_frame_id = "map";
    std::string metadata_topic = "/map_metadata";
    std::string map_dir = "/home/lion/config/maps";
};

struct NodeState
{
    std::string yaml_name = "";
    nav_msgs::OccupancyGrid current_map;
    nav_msgs::MapMetaData current_metadata;
};

class MapManagerROS
{
public:
    MapManagerROS() = delete;

    MapManagerROS(const MapManagerROS &) = delete;

    MapManagerROS &operator=(const MapManagerROS &) = delete;

    MapManagerROS(ros::NodeHandle& nh);

    void loadParameters();

    void initPublishers();

    void initServices();

    bool loadMap();

    void publishMap();
    
    // bool changeMapService(zouyu_srvs::ChangeMap::Request &req, zouyu_srvs::ChangeMap::Response &res);

    bool reloadMapService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    
    bool currentMapService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    

private:
    ros::NodeHandle m_nh;
    ros::Publisher m_map_pub;
    ros::Publisher m_metadata_pub;
    // ros::ServiceServer m_change_map_srv;
    ros::ServiceServer m_reload_map_srv;
    ros::ServiceServer m_current_map_srv;
    NodeConfig m_config;
    NodeState m_state;
};
