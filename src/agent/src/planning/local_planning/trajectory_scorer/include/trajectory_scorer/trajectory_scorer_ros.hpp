#include "nav2_util/lifecycle_node.hpp"
#include "planning_interfaces/srv/trajectory_scoring.hpp"
#ifndef TRAJECTORY_SCORER_NODE_HPP_
#define TRAJECTORY_SCORER_NODE_HPP_
namespace local_planning
{
    class TrajectoryScorerROS : public nav2_util::LifecycleNode
    {
    public:
        explicit TrajectoryScorerROS(const std::string &name);
        explicit TrajectoryScorerROS(
            const std::string &name,
            const std::string &parent_namespace,
            const std::string &local_namespace);
        ~TrajectoryScorerROS();

    protected:
        // implement the lifecycle interface
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

        std::string name_;
        std::string parent_namespace_;

        rclcpp::Service<planning_interfaces::srv::TrajectoryScoring>::SharedPtr service;

        void score_trajectory(
            const std::shared_ptr<planning_interfaces::srv::TrajectoryScoring::Request> request,
            const std::shared_ptr<planning_interfaces::srv::TrajectoryScoring::Response> response);
    };
} // local_planning
#endif