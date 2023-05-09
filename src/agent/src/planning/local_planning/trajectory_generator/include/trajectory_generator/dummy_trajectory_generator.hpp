#ifndef DUMMY_TRAJECTORY_GENERATOR_HPP
#define DUMMY_TRAJECTORY_GENERATOR_HPP
#include "trajectory_generator/trajectory_generator_interface.hpp"
#include <memory>
namespace local_planning
{
    class DummyTrajectoryGenerator : public TrajectoryGeneratorInterface
    {
    public:
        DummyTrajectoryGenerator(){
            this->name = "DummyTrajectoryGenerator";
            
    } 
    nav_msgs::msg::Path
    computeTrajectory(const nav2_msgs::msg::Costmap::SharedPtr costmap,
                        const nav_msgs::msg::Odometry::SharedPtr odom,
                        const geometry_msgs::msg::PoseStamped::SharedPtr next_waypoint) override
    {
        nav_msgs::msg::Path path;
        // dummy trajectory generator just directly output path to next_waypoint            
        path.poses.push_back(*next_waypoint);
        return path;
    }

    void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr parent, 
                    std::string name, 
                    std::shared_ptr<tf2_ros::Buffer> tf) override 
    {
    
    }

    void cleanup() override
    {

    }

    void activate() override
    {

    }

    void deactivate() override
    {

    }

    private:
        std::shared_ptr<VehicleModelInterface> vehicle_model;
    };
}
#endif // DUMMY_TRAJECTORY_GENERATOR_HPP
