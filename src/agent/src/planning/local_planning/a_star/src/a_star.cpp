#include "a_star/a_star.hpp"
#include <rclcpp/logging.hpp>

namespace local_planning
{
    void AStar::configure(rclcpp_lifecycle::LifecycleNode * parent,
                          std::shared_ptr<tf2_ros::Buffer> tf)
    {
        this->parent_node = parent;
        this->tf_buffer = tf;

        this->name = name;

        RCLCPP_INFO(rclcpp::get_logger(this->name), "Configured");

    }

    void AStar::cleanup()
    {
        this->parent_node = nullptr;
        this->tf_buffer = nullptr;
        RCLCPP_INFO(rclcpp::get_logger(this->name), "cleanup");

    }

    void AStar::activate()
    {
        // Implementation details for activation
        RCLCPP_INFO(rclcpp::get_logger(this->name), "activate");

    }

    void AStar::deactivate()
    {
        // Implementation details for deactivation
        RCLCPP_INFO(rclcpp::get_logger(this->name), "deactivate");
    }

    nav_msgs::msg::Path AStar::computeTrajectory(const nav2_msgs::msg::Costmap::SharedPtr costmap,
                                                 const nav_msgs::msg::Odometry::SharedPtr odom,
                                                 const geometry_msgs::msg::PoseStamped::SharedPtr next_waypoint)
    {
        nav_msgs::msg::Path path;
        // Implementation details for trajectory computation using A*
        return path;
    }
}

