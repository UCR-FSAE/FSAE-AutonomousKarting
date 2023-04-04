#include "trajectory_picker/trajectory_picker_ros.hpp"
#include <nav2_util/node_utils.hpp>

namespace local_planning
{
    TrajectoryPickerROS::TrajectoryPickerROS(const std::string &name)
        : TrajectoryPickerROS(name, "/", name) {}

    TrajectoryPickerROS::TrajectoryPickerROS(
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

    TrajectoryPickerROS::~TrajectoryPickerROS()
    {

    }

    nav2_util::CallbackReturn
    TrajectoryPickerROS::on_configure(const rclcpp_lifecycle::State &state)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn TrajectoryPickerROS::on_activate(const rclcpp_lifecycle::State &state)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn TrajectoryPickerROS::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn TrajectoryPickerROS::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn TrajectoryPickerROS::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }
} // local_planning
