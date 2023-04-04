#include "trajectory_scorer/trajectory_scorer_ros.hpp"
#include <nav2_util/node_utils.hpp>

namespace local_planning
{
    TrajectoryScorerROS::TrajectoryScorerROS(const std::string &name)
        : TrajectoryScorerROS(name, "/", name) {}

    TrajectoryScorerROS::TrajectoryScorerROS(
        const std::string &name,
        const std::string &parent_namespace,
        const std::string &local_namespace)
        : LifecycleNode(name, "", true,
                        rclcpp::NodeOptions().arguments({"--ros-args", "-r", std::string("__ns:=") + nav2_util::add_namespaces(parent_namespace, local_namespace),
                                                         "--ros-args", "-r", name + ":" + std::string("__node:=") + name})),
          name_(name),
          parent_namespace_(parent_namespace)
    {
    }

    TrajectoryScorerROS ::~TrajectoryScorerROS()
    {
    }

    nav2_util::CallbackReturn TrajectoryScorerROS::on_configure(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "on_configure");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn TrajectoryScorerROS::on_activate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "on_activate");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn TrajectoryScorerROS::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "on_deactivate");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn TrajectoryScorerROS::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "on_cleanup");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn TrajectoryScorerROS::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "on_shutdown");
        return nav2_util::CallbackReturn::SUCCESS;
    }
} // local_planning
