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

    TrajectoryGeneratorROS ::~TrajectoryGeneratorROS()
    {
    }

    /* Lifecycle */
    nav2_util::CallbackReturn TrajectoryGeneratorROS::on_configure(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(this->get_logger(), "on_configure");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn TrajectoryGeneratorROS::on_activate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(this->get_logger(), "on_activate");
        this->action_server_ = rclcpp_action::create_server<TrajectoryGeneration>(
            this,
            "trajectory_generation",
            std::bind(&TrajectoryGeneratorROS::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TrajectoryGeneratorROS::handle_cancel, this, std::placeholders::_1),
            std::bind(&TrajectoryGeneratorROS::handle_accepted, this, std::placeholders::_1));
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

    /* Action server */
    rclcpp_action::GoalResponse
    TrajectoryGeneratorROS::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const TrajectoryGeneration::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "handle_goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse
    TrajectoryGeneratorROS::handle_cancel(const std::shared_ptr<GoalHandleTrajectoryGeneration> goal_handle)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void TrajectoryGeneratorROS::handle_accepted(const std::shared_ptr<GoalHandleTrajectoryGeneration> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "handle_accepted");

        std::thread{std::bind(&TrajectoryGeneratorROS::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void TrajectoryGeneratorROS::execute(const std::shared_ptr<GoalHandleTrajectoryGeneration> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        auto feedback = std::make_shared<TrajectoryGeneration::Feedback>();
        goal_handle->publish_feedback(feedback);

        auto result = std::make_shared<TrajectoryGeneration::Result>();
        goal_handle->succeed(result);
    }

} // local_planning
