#include "trajectory_generator/trajectory_generator_ros.hpp"
#include <nav2_util/node_utils.hpp>

namespace local_planning
{

    TrajectoryGeneratorROS::TrajectoryGeneratorROS(const std::string &name)
        : TrajectoryGeneratorROS(name, "/", name) {}

    TrajectoryGeneratorROS::TrajectoryGeneratorROS(
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

    // TrajectoryGeneratorROS::TrajectoryGeneratorROS() : LifecycleNode("trajectory_generator", "local_planner", true)
    // {

    // }

    TrajectoryGeneratorROS ::~TrajectoryGeneratorROS()
    {
    }

        nav2_util::CallbackReturn TrajectoryGeneratorROS::on_configure(const rclcpp_lifecycle::State &state)
        {
            RCLCPP_INFO(this->get_logger(), "on_configure");
            return nav2_util::CallbackReturn::SUCCESS;
        }

        nav2_util::CallbackReturn TrajectoryGeneratorROS::on_activate(const rclcpp_lifecycle::State &state)
        {
            RCLCPP_INFO(this->get_logger(), "on_activate");
            return nav2_util::CallbackReturn::SUCCESS;
        }

        nav2_util::CallbackReturn TrajectoryGeneratorROS::on_deactivate(const rclcpp_lifecycle::State &state)
        {
            RCLCPP_INFO(this->get_logger(), "on_deactivate");
            return nav2_util::CallbackReturn::SUCCESS;
        }

        nav2_util::CallbackReturn TrajectoryGeneratorROS::on_cleanup(const rclcpp_lifecycle::State &state)
        {
            RCLCPP_INFO(this->get_logger(), "on_cleanup");
            return nav2_util::CallbackReturn::SUCCESS;
        }

        nav2_util::CallbackReturn TrajectoryGeneratorROS::on_shutdown(const rclcpp_lifecycle::State &state)
        {
            RCLCPP_INFO(this->get_logger(), "on_shutdown");
            return nav2_util::CallbackReturn::SUCCESS;
        }
    } // local_planning
