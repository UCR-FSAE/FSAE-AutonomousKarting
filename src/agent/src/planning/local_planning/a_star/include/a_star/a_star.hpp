#ifndef A_STAR_HPP_
#define A_STAR_HPP_
#include "trajectory_generator/trajectory_generator_interface.hpp"
#include <memory>
namespace local_planning
{
    class AStar : public TrajectoryGeneratorInterface
    {
    public:
    AStar(){
        this->name = "A* algo";
    } 
    nav_msgs::msg::Path
    computeTrajectory(const nav2_msgs::msg::Costmap::SharedPtr costmap,
                        const nav_msgs::msg::Odometry::SharedPtr odom,
                        const geometry_msgs::msg::PoseStamped::SharedPtr next_waypoint) override;

    void configure(rclcpp_lifecycle::LifecycleNode * parent, 
                    std::shared_ptr<tf2_ros::Buffer> tf) override ;

    void cleanup() override; 

    void activate() override;

    void deactivate();

    private:
            rclcpp_lifecycle::LifecycleNode * parent_node;
            std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    };
}
#endif // A_STAR_HPP_
