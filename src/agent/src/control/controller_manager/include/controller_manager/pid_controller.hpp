#include "controller_manager/controller_interface.hpp"

namespace controller 
{
    class PIDController : public ControllerInterface
    {   
        public:
            void setup(const std::map<const std::string, boost::any> dict) override;
            ackermann_msgs::msg::AckermannDrive compute(
                const nav_msgs::msg::Path::SharedPtr trajectory,
                const nav_msgs::msg::Odometry::SharedPtr odom,
                const std::map<const std::string, boost::any> extra) override;
    };
}