#include "slam_nav_system/navigation_manager.hpp"

namespace slam_nav_system {

NavigationManager::NavigationManager(
    const rclcpp::Node::SharedPtr& node)
: node_(node)
{
}

void NavigationManager::sendGoal(
    const geometry_msgs::msg::PoseStamped& goal)
{
}

}