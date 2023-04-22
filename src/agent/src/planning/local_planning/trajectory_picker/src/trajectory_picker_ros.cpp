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
        best_trajectory_publisher_ = this->create_publisher<planning_interfaces::msg::Trajectory>(
            std::string(this->get_namespace()) + "/"+std::string(this->get_name()) + "/best_trajectory", 10);
        new_trajectory_sub_ = this->create_subscription<planning_interfaces::msg::Trajectory>(
            std::string(this->get_namespace()) + "/"+std::string(this->get_name()) + "/possible_trajectory", 
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
        best_trajectory_publisher_->on_activate();
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn TrajectoryPickerROS::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "on_deactivate");
        best_trajectory_publisher_->on_deactivate();
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
        RCLCPP_DEBUG(get_logger(), "on_new_trajectory_recevied");

        // TODO: do some selection here
        this->publish_best_trajectory(msg);

    }
    void TrajectoryPickerROS::publish_best_trajectory(const std::shared_ptr<planning_interfaces::msg::Trajectory> traj)
    {
        float score = traj->score.scale*traj->score.raw_score;
        RCLCPP_INFO(get_logger(), "new local trajectory published -- length: [%d] - Score: [%.3f]", traj->trajectory.poses.size(), score);
        this->best_trajectory_publisher_->publish(*traj);
        this->p_reset_trajectory_picker();
    }

    void TrajectoryPickerROS::reset_trajectory_picker(const std::shared_ptr<planning_interfaces::srv::ResetTrajectoryPicker::Request> req,
                                 std::shared_ptr<planning_interfaces::srv::ResetTrajectoryPicker::Response> resp)
    {
        RCLCPP_DEBUG(get_logger(), "reset_trajectory_picker");
        this->p_reset_trajectory_picker();
    }

    void TrajectoryPickerROS::p_reset_trajectory_picker()
    {
        RCLCPP_DEBUG(get_logger(), "p_reset_trajectory_picker");
        this->trajectories.clear();
    }

} // local_planning
