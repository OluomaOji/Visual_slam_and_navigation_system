#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace slam_nav_system {

    class SLAMManager {
    public:
        explicit SLAMManager(const rclcpp::Node::SharedPtr& node);

        void startSLAM();
        void stopSLAM();

        nav_msgs::msg::OccupancyGrid::SharedPtr getLatestMap() const; // Method to retrieve the latest map

    private:
        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg); // Callback for receiving map updates

        rclcpp::Node::SharedPtr node_; // ROS node handle
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_; // Subscriber for map updates
        nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_; // Latest map received from the SLAM system
    };
} // namespace slam_nav_system