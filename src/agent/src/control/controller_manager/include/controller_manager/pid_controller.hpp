#include "controller_manager/controller_interface.hpp"

namespace controller 
{
    class PIDController : public ControllerInterface
    {   
        public:
            void setTrajectory(const nav_msgs::msg::Path::SharedPtr trajectory)  override;
            ControlResult compute(const nav_msgs::msg::Odometry::SharedPtr odom, 
                                std::mutex& odom_mutex,
                                const std::map<const std::string, boost::any> extra) override;
        private:
            std::shared_ptr<nav_msgs::msg::Path> trajectory;
    };
}