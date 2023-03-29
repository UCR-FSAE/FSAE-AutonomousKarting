#include "trajectory_generator/vehicle_model_interface.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/msg/costmap.hpp>

namespace local_planning
{
    class TrajectoryGeneratorInterface
    {
        public:
            virtual nav_msgs::msg::Path computeTrajectory(const nav2_msgs::msg::Costmap &costmap, const nav_msgs::msg::Odometry &odom);
            
            virtual void setVehicleModel(const VehicleModelInterface &model);
            virtual void getVehicleModel();
    };
}
