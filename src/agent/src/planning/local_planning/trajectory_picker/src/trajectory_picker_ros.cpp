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
        trajectory_publisher_ = this->create_publisher<planning_interfaces::msg::Trajectory>(
            std::string(this->get_namespace()) + std::string(this->get_name()) + "/best_trajectory", 10);
        new_trajectory_sub_ = this->create_subscription<planning_interfaces::msg::Trajectory>(
            std::string(this->get_namespace()) + std::string(this->get_name()) + "/possible_trajectory", 
            10,
            std::bind(&TrajectoryPickerROS::on_new_trajectory_received, this, std::placeholders::_1));
        reset_trajectory_picker_service = this->create_service<planning_interfaces::srv::ResetTrajectoryPicker>(
            "reset", std::bind(&TrajectoryPickerROS::reset_trajectory_picker, 
                                this, std::placeholders::_1, std::placeholders::_2));
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn TrajectoryPickerROS::on_activate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "on_activate");
        trajectory_publisher_->on_activate();
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn TrajectoryPickerROS::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "on_deactivate");
        trajectory_publisher_->on_deactivate();
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn TrajectoryPickerROS::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "on_cleanup");
        
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn TrajectoryPickerROS::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "on_shutdown");

        return nav2_util::CallbackReturn::SUCCESS;
    }


    void TrajectoryPickerROS::on_new_trajectory_received(const planning_interfaces::msg::Trajectory::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "on_new_trajectory_recevied");

    }
    void TrajectoryPickerROS::on_publish_trajectory(const std::shared_ptr<planning_interfaces::msg::Trajectory> traj)
    {
        RCLCPP_INFO(get_logger(), "on_publish_trajectory");

    }

    void TrajectoryPickerROS::reset_trajectory_picker(const std::shared_ptr<planning_interfaces::srv::ResetTrajectoryPicker::Request> req,
                                 std::shared_ptr<planning_interfaces::srv::ResetTrajectoryPicker::Response> resp)
    {
        RCLCPP_INFO(get_logger(), "reset_trajectory_picker");

    }
} // local_planning
