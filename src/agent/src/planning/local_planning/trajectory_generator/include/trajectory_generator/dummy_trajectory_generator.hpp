#ifndef DUMMY_TRAJECTORY_GENERATOR_CPP
#define DUMMY_TRAJECTORY_GENERATOR_CPP
#include "trajectory_generator/trajectory_generator_interface.hpp"

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

    void setVehicleModel(const std::shared_ptr<VehicleModelInterface> model) override
    {
        this->vehicle_model = model;
    }

    std::shared_ptr<VehicleModelInterface> getVehicleModel() override
    {
        // Return null pointer
        return this->vehicle_model;
    }

    private:
        std::shared_ptr<VehicleModelInterface> vehicle_model;
    };
}
#endif // DUMMY_TRAJECTORY_GENERATOR_CPP
