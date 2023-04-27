#include <controller_manager/pid_controller.hpp>

namespace controller 
{
    void 
    PIDController::setup(const std::map<const std::string, boost::any> dict)
    {

    }

    void 
    PIDController::update(const std::map<const std::string, boost::any> dict)
    {

    }

    ackermann_msgs::msg::AckermannDrive 
    PIDController::compute(
                const nav_msgs::msg::Path::SharedPtr trajectory,
                const nav_msgs::msg::Odometry::SharedPtr odom,
                const std::map<const std::string, boost::any> extra
                )
    {
        ackermann_msgs::msg::AckermannDrive result;

        result.acceleration = 1.0;
        result.steering_angle = 0.0;
        return result;
    }

} // controller