#include <controller_manager/pid_controller.hpp>

namespace controller 
{
    void PIDController::setTrajectory(const nav_msgs::msg::Path::SharedPtr trajectory)
    {
        this->trajectory = trajectory;
    }

    ControlResult 
    PIDController::compute(const  nav_msgs::msg::Odometry::SharedPtr odom,
                           std::mutex& odom_mutex,
                           const std::map<const std::string, boost::any> extra)
    {
        ControlResult result;

        ackermann_msgs::msg::AckermannDrive msg;

        msg.acceleration = 1.0;
        msg.steering_angle = 0.0;
        result.drive = msg;
        result.waypoint_index = 0;

        return result;
    }



} // controller