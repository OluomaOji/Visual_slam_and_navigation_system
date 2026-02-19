#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "slam_nav_system/slam_manager.hpp"
#include "slam_nav_system/localization_manager.hpp"
#include "slam_nav_system/navigation_manager.hpp"
#include "slam_nav_system/obstacle_monitor.hpp"

namespace slam_nav_system {

    class SlamNavSystemNode : public rclcpp::Node {

    public:
        SlamNavSystemNode()
        : Node("slam_nav_system_node")
        {
            declareParameters();
            initModules();
            setupGoalSubscription();
            setupStatusPublisher();

            timer_ = create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&SlamNavSystemNode::controlLoop, this));
        }

    private:
        void declareParameters() {
            // Declare parameters for SLAM, localization, navigation, and obstacle monitoring
            this->declare_parameter<std::string>("global_frame", "map");
            this->declare_parameter<std::string>("base_frame", "base_footprint");
            this->declare_parameter<double>("obstacle_distance_threshold", 0.35);
        }

        void initModules() {
            auto node_shared = shared_from_this();
            slam_ = std::make_unique<SLAMManager>(node_shared);
            localization_ = std::make_unique<LocalizationManager>(node_shared);
            navigation_ = std::make_unique<NavigationManager>(node_shared);
            obstacles_ = std::make_unique<ObstacleMonitor>(node_shared);
        }   

        void setupGoalSubscription() {
            goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "goal_pose",
                10,
               [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
                {
                    RCLCPP_INFO(this->get_logger(), "Received new goal pose");
                    navigation_->sendGoal(*msg);
                });
        }

        void setupStatusPublisher() {
            status_pub_ = this->create_publisher<std_msgs::msg::String>("navigation_status", 10);
        }

        void controlLoop(){

        }

        std::unique_ptr<SLAMManager> slam_;
        std::unique_ptr<LocalizationManager> localization_;
        std::unique_ptr<NavigationManager> navigation_;
        std::unique_ptr<ObstacleMonitor> obstacles_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
        rclcpp::TimerBase::SharedPtr timer_;


    };
} // namespace slam_nav_system

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<slam_nav_system::SlamNavSystemNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;   
}

