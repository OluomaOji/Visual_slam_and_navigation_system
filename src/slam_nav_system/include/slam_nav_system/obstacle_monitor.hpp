#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


namespace slam_nav_system {

    class ObstacleMonitor {
    public:
        explicit ObstacleMonitor(const rclcpp::Node::SharedPtr& node); // Constructor that takes a shared pointer to a ROS node

        bool isObstacleTooClose() const;

    private:
        void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg); // Callback for receiving laser scan data

        rclcpp::Node::SharedPtr node_;// ROS node handle
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; // Subscriber for laser scan data

        double min_distance_; // Minimum distance threshold to consider an obstacle as "too close"
        bool obstacle_detected_; // Flag to indicate if an obstacle is detected within the threshold distance
    };
} // namespace slam_nav_system