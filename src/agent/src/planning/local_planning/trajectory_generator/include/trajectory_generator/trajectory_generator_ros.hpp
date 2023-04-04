#include "nav2_util/lifecycle_node.hpp"
#ifndef TRAJECTORY_GENERATOR_NODE_HPP_
#define TRAJECTORY_GENERATOR_NODE_HPP_
namespace local_planning
{
    class TrajectoryGeneratorROS : public nav2_util::LifecycleNode
    {
    public:
        explicit TrajectoryGeneratorROS(const std::string &name);
        explicit TrajectoryGeneratorROS(
            const std::string &name,
            const std::string &parent_namespace,
            const std::string &local_namespace);
        // TrajectoryGeneratorROS();
        ~TrajectoryGeneratorROS();

    protected:
        // implement the lifecycle interface
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

        std::string name_;
        std::string parent_namespace_;
    };
} // local_planning
#endif