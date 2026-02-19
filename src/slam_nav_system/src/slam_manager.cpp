#include "slam_nav_system/slam_manager.hpp"

namespace slam_nav_system {

SLAMManager::SLAMManager(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
}

void SLAMManager::startSLAM() {}

void SLAMManager::stopSLAM() {}

nav_msgs::msg::OccupancyGrid::SharedPtr
SLAMManager::getLatestMap() const
{
    return latest_map_;
}

void SLAMManager::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    latest_map_ = msg;
}

} // namespace slam_nav_system