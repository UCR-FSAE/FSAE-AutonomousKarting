#include "nav2_util/lifecycle_node.hpp"
#ifndef TRAJECTORY_GENERATOR_NODE_HPP_
#define TRAJECTORY_GENERATOR_NODE_HPP_
namespace local_planning {
    class TrajectoryGeneratorNode : public nav2_util::LifecycleNode
    {
        public:
            TrajectoryGeneratorNode();
            ~TrajectoryGeneratorNode();

        protected:
            // implement the lifecycle interface
            nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
            
            nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

            nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

            nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

            nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
    };
} // local_planning
#endif