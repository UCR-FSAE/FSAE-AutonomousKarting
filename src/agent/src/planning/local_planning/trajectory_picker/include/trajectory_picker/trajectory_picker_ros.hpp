#include "nav2_util/lifecycle_node.hpp"
#include "planning_interfaces/msg/trajectory.hpp"
#include "planning_interfaces/srv/reset_trajectory_picker.hpp"

#ifndef TRAJECTORY_PICKER_NODE_HPP_
#define TRAJECTORY_PICKER_NODE_HPP_
namespace local_planning
{
    class TrajectoryPickerROS : public nav2_util::LifecycleNode
    {
    public:
        explicit TrajectoryPickerROS(const std::string &name);
        explicit TrajectoryPickerROS(
            const std::string &name,
            const std::string &parent_namespace,
            const std::string &local_namespace);
        ~TrajectoryPickerROS();

    protected:
        // implement the lifecycle interface
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

        std::string name_;
        std::string parent_namespace_;

        std::vector<planning_interfaces::msg::Trajectory> trajectories;

        void on_new_trajectory_received(const planning_interfaces::msg::Trajectory::SharedPtr msg);
        rclcpp::Subscription<planning_interfaces::msg::Trajectory>::SharedPtr new_trajectory_sub_;

        void on_publish_trajectory(const std::shared_ptr<planning_interfaces::msg::Trajectory> traj); 
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<planning_interfaces::msg::Trajectory>> trajectory_publisher_;

        void reset_trajectory_picker(const std::shared_ptr<planning_interfaces::srv::ResetTrajectoryPicker::Request> req,
                                     std::shared_ptr<planning_interfaces::srv::ResetTrajectoryPicker::Response> resp);
        rclcpp::Service<planning_interfaces::srv::ResetTrajectoryPicker>::SharedPtr reset_trajectory_picker_service;
    };
} // local_planning
#endif