#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>

namespace slam_nav_system::pose_utils {
    geometry_msgs::msg::PoseStamped makePoseStamped(
        double x,
        double y,
        double yaw,
        const std::string &frame_id);

} // namespace slam_nav_system::pose_utils