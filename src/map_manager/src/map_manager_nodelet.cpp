#include "map_manager_ros.h"
#include <nodelet/nodelet.h>

namespace navigations
{
    class MapManagerNodelet : public nodelet::Nodelet
    {
    private:
        std::shared_ptr<MapManagerROS> map_manager;
        virtual void onInit()
        {
            map_manager = std::make_shared<MapManagerROS>(getPrivateNodeHandle());
        }
    };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(navigations::MapManagerNodelet, nodelet::Nodelet)