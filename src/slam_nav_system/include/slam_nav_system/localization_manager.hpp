// This wil use TF2 to get the robot's current pose in the map frame and provide it to other components of the system.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


namespace slam_nav_system {
    class LocalizationManager {
    public:
        explicit LocalizationManager(const rclcpp::Node::SharedPtr& node);

        geometry_msgs::msg::PoseStamped getCurrentPose(); // Method to get the current pose of the robot in the map frame
        bool isLocalizationAvailable(); // Method to check if localization data is available
        bool isLocalizationHealthy(); // Method to check the health of the localization data based on covariance or other metrics
    private:
        rclcpp::Node::SharedPtr node_; // ROS node handle
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_; // TF buffer to store transform data
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_; // TF listener to receive transform updates

        std::string global_frame_; // Typically "map"
        std::string local_frame_; // Typically "map" and "base_link" or "odom"
        double pose_cov_threshold_;
    };
} // namespace slam_nav_system
