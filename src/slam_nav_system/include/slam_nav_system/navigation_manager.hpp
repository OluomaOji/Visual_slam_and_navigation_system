#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace slam_nav_system {

    class NavigationManager {
    public:
        using NavigationToPose = nav2_msgs::action::NavigateToPose; // Action definition for navigating to a pose
        using GoalHandleNavigationToPose = rclcpp_action::Client<NavigationToPose>; // send goal and get result from the action server

        explicit NavigationManager(const rclcpp::Node::SharedPtr& node);

        void sendGoal(const geometry_msgs::msg::PoseStamped& goal); // Method to send a navigation goal to the action server
        void cancelGoal();
        bool isNavigating() const;
    private:
        rclcpp::Node::SharedPtr node_; // ROS node handle
        rclcpp_action::Client<NavigationToPose>::SharedPtr action_client_; // Action client for sending goals to the navigation stack
        bool navigating_;
    };
} // namespace slam_nav_system


