#include "map_manager_ros.h"

MapManagerROS::MapManagerROS(ros::NodeHandle &nh) : m_nh(nh)
{
    loadParameters();
    initPublishers();
    initServices();
    if (loadMap())
    {
        ROS_DEBUG("[MapManagerROS] Map loaded");
        publishMap();
    }
}

void MapManagerROS::loadParameters()
{
    ROS_DEBUG("[MapManagerROS] Loading parameters");
    m_nh.param<std::string>("map_topic", m_config.map_topic, "/map");
    m_nh.param<std::string>("map_frame_id", m_config.map_frame_id, "map");
    m_nh.param<std::string>("metadata_topic", m_config.metadata_topic, "/map_metadata");
    m_nh.param<std::string>("map_dir", m_config.map_dir, "/home/lion/config/maps");
    m_nh.param<std::string>("yaml_name", m_state.yaml_name, "map");
}

void MapManagerROS::initPublishers()
{
    m_map_pub = m_nh.advertise<nav_msgs::OccupancyGrid>(m_config.map_topic, 1, true);
    m_metadata_pub = m_nh.advertise<nav_msgs::MapMetaData>(m_config.metadata_topic, 1, true);
}

void MapManagerROS::initServices()
{
    // m_change_map_srv = m_nh.advertiseService("change_map", &MapManagerROS::changeMapService, this);
    m_reload_map_srv = m_nh.advertiseService("reload_map", &MapManagerROS::reloadMapService, this);
    m_current_map_srv = m_nh.advertiseService("current_map", &MapManagerROS::currentMapService, this);
}

void MapManagerROS::publishMap()
{
    ros::Time::waitForValid();
    m_state.current_map.header.stamp = ros::Time::now();
    m_state.current_map.info.map_load_time = ros::Time::now();
    m_state.current_map.header.frame_id = m_config.map_frame_id;
    m_map_pub.publish(m_state.current_map);
    m_state.current_metadata.map_load_time = ros::Time::now();
    m_metadata_pub.publish(m_state.current_metadata);
}

bool MapManagerROS::loadMap()
{
    std::filesystem::path map_dir(m_config.map_dir);
    std::filesystem::path yaml_file = map_dir / (m_state.yaml_name + ".yaml");
    if (!std::filesystem::exists(yaml_file))
    {
        ROS_WARN("[MapManagerROS] Map file %s does not exist", yaml_file.c_str());
        return false;
    }
    ROS_DEBUG("[MapManagerROS] Loading map from %s", yaml_file.c_str());
    YAML::Node yaml = YAML::LoadFile(yaml_file);
    std::string image_name = yaml["image"].as<std::string>();
    std::filesystem::path image_file = map_dir / image_name;
    if (!std::filesystem::exists(image_file))
    {
        ROS_WARN("[MapManagerROS] Image file %s does not exist", image_file.c_str());
        return false;
    }
    cv::Mat image = cv::imread(image_file.c_str(), cv::IMREAD_GRAYSCALE);
    if (image.empty())
    {
        ROS_WARN("[MapManagerROS] Failed to load image file %s", image_file.c_str());
        return false;
    }
    double occ_th = yaml["occupied_thresh"].as<double>();
    double free_th = yaml["free_thresh"].as<double>();
    bool negate = yaml["negate"].as<bool>();

    std::vector<double> origin = yaml["origin"].as<std::vector<double>>();
    m_state.current_metadata.resolution = yaml["resolution"].as<float>();
    m_state.current_metadata.width = image.cols;
    m_state.current_metadata.height = image.rows;
    m_state.current_metadata.origin.position.x = origin[0];
    m_state.current_metadata.origin.position.y = origin[1];
    m_state.current_metadata.origin.position.z = 0.0;
    tf::Quaternion q = tf::createQuaternionFromYaw(origin[2]);

    m_state.current_metadata.origin.orientation.x = q.x();
    m_state.current_metadata.origin.orientation.y = q.y();
    m_state.current_metadata.origin.orientation.z = q.z();
    m_state.current_metadata.origin.orientation.w = q.w();
    m_state.current_map.info = m_state.current_metadata;
    m_state.current_map.data.resize(m_state.current_metadata.width * m_state.current_metadata.height);

    for (int i = 0; i < m_state.current_metadata.height; i++)
    {
        for (int j = 0; j < m_state.current_metadata.width; j++)
        {
            int value = image.at<uchar>(i, j);
            if (negate)
                value = 255 - value;
            if (value > occ_th)
                value = 100;
            else if (value < free_th)
                value = 0;
            else
                value = static_cast<int>((value - free_th) / (occ_th - free_th) * 100);
            m_state.current_map.data[MAP_IDX(m_state.current_metadata.width, j, m_state.current_metadata.height - i - 1)] = value;
        }
    }
    return true;
}

// bool MapManagerROS::changeMapService(zouyu_srvs::ChangeMap::Request &req, zouyu_srvs::ChangeMap::Response &res)
// {
//     m_state.yaml_name = req.name;
//     if (loadMap())
//     {
//         publishMap();
//         res.success = true;
//         res.message = "Map changed to " + req.name + ".yaml";
//         return true;
//     }
//     res.success = false;
//     res.message = "Failed to change map to " + req.name + ".yaml";
//     return false;
// }

bool MapManagerROS::reloadMapService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if (loadMap())
    {
        publishMap();
        res.success = true;
        res.message = "Map reloaded " + m_state.yaml_name + ".yaml";
        return true;
    }
    res.success = false;
    res.message = "Failed to reload map " + m_state.yaml_name + ".yaml";
    return false;
}

bool MapManagerROS::currentMapService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.success = true;
    res.message = m_state.yaml_name;
    return true;
}
