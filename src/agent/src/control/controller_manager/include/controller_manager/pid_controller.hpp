#include "controller_interface.hpp"

namespace controller 
{
    class PIDController : public ControllerInterface
    {   
        public:
            void setup(const std::map<const std::string, boost::any> dict);
            void update(const std::map<const std::string, boost::any> dict);
            ackermann_msgs::msg::AckermannDrive compute(
                const nav_msgs::msg::Path::SharedPtr trajectory,
                const nav_msgs::msg::Odometry::SharedPtr odom,
                const std::map<const std::string, boost::any> extra);
        
        
    };
}