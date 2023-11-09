#ifndef TRAJECTORY_GENERATOR_INTERFACE_HPP
#define TRAJECTORY_GENERATOR_INTERFACE_HPP

#include "planning_interfaces/action/trajectory_generation.hpp"
#include "tf2_ros/buffer.h"
#include "trajectory_generator/vehicle_model_interface.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace local_planning {
class TrajectoryGeneratorInterface {
  public:
    virtual void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
                           std::shared_ptr<tf2_ros::Buffer> tf) = 0;
    /**
     * @brief Method to cleanup resources used on shutdown.
     */
    virtual void cleanup() = 0;

    /**
     * @brief Method to active planner and any threads involved in execution.
     */
    virtual void activate() = 0;

    /**
     * @brief Method to deactive planner and any threads involved in execution.
     */
    virtual void deactivate() = 0;
    virtual nav_msgs::msg::Path computeTrajectory(
        const std::shared_ptr<
            const planning_interfaces::action::TrajectoryGeneration_Goal>
            request) = 0;

    std::string name = "TrajectoryGenerator";
};
} // namespace local_planning

#endif // TRAJECTORY_GENERATOR_INTERFACE_HPP