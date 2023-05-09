#ifndef A_STAR_HPP
#define A_STAR_HPP

#include "trajectory_generator/trajectory_generator_interface.hpp"

namespace local_planning
{
    class AStar : public TrajectoryGeneratorInterface
    {
        public:
            void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf) override;
            void cleanup() override;
            void activate() override;
            void deactivate() override;
            nav_msgs::msg::Path computeTrajectory(const nav2_msgs::msg::Costmap::SharedPtr costmap,
                                                  const nav_msgs::msg::Odometry::SharedPtr odom,
                                                  const geometry_msgs::msg::PoseStamped::SharedPtr next_waypoint) override;

            std::string name = "AStar";
    };
}

#endif // A_STAR_HPP
