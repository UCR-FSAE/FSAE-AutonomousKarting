#ifndef TRAJECTORY_GENERATOR_INTERFACE_HPP
#define TRAJECTORY_GENERATOR_INTERFACE_HPP

#include "trajectory_generator/vehicle_model_interface.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace local_planning
{
    class TrajectoryGeneratorInterface
    {
        public:
            virtual nav_msgs::msg::Path computeTrajectory(const nav2_msgs::msg::Costmap::SharedPtr costmap,
                                                          const nav_msgs::msg::Odometry::SharedPtr odom,
                                                          const geometry_msgs::msg::PoseStamped::SharedPtr next_waypoint) = 0;

            virtual void setVehicleModel(const std::shared_ptr<VehicleModelInterface> model) = 0;
            virtual std::shared_ptr<VehicleModelInterface> getVehicleModel() = 0;

            std::string name = "TrajectoryGenerator";
    };
}

#endif // TRAJECTORY_GENERATOR_INTERFACE_HPP