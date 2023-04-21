#ifndef TRAJECTORY_GENERATOR_NODE_HPP_
#define TRAJECTORY_GENERATOR_NODE_HPP_
#include "nav2_util/lifecycle_node.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "planning_interfaces/action/trajectory_generation.hpp"
#include "planning_interfaces/msg/trajectory.hpp"
#include "trajectory_generator/trajectory_generator_interface.hpp"

namespace local_planning
{
    class TrajectoryGeneratorROS : public nav2_util::LifecycleNode
    {
        using TrajectoryGeneration = planning_interfaces::action::TrajectoryGeneration;
        using GoalHandleTrajectoryGeneration = rclcpp_action::ServerGoalHandle<TrajectoryGeneration>;

        public:
            explicit TrajectoryGeneratorROS(const std::string &name);
            explicit TrajectoryGeneratorROS(
                const std::string &name,
                const std::string &parent_namespace,
                const std::string &local_namespace);

            ~TrajectoryGeneratorROS();

            void registerTrajectoryGenerator(const std::shared_ptr<TrajectoryGeneratorInterface> generator);

        protected:
            // implement the lifecycle interface
            nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;

            nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

            nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

            nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

            nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

            std::string name_;
            std::string parent_namespace_;

            /* Action server */
            rclcpp_action::Server<TrajectoryGeneration>::SharedPtr action_server_;
            rclcpp_action::GoalResponse
            handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const TrajectoryGeneration::Goal> goal);
            rclcpp_action::CancelResponse
            handle_cancel(const std::shared_ptr<GoalHandleTrajectoryGeneration> goal_handle);
            void handle_accepted(const std::shared_ptr<GoalHandleTrajectoryGeneration> goal_handle);
            void execute(const std::shared_ptr<GoalHandleTrajectoryGeneration> goal_handle);

            std::vector<std::shared_ptr<TrajectoryGeneratorInterface>> trajectory_generators;
    };
} // local_planning
#endif