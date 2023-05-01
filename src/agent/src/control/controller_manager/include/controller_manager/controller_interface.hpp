#ifndef CONTROLLER_INTERFACE_HPP_
#define CONTROLLER_INTERFACE_HPP_
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <map>
#include <boost/any.hpp>
#include <math.h>

namespace controller 
{
    class ControllerInterface
    {
        public: 
            virtual void setup(std::map<const std::string, boost::any> dict) = 0;
            virtual ackermann_msgs::msg::AckermannDrive compute(const nav_msgs::msg::Path::SharedPtr trajectory,
                                                                const nav_msgs::msg::Odometry::SharedPtr odom,
                                                                const std::map<const std::string, boost::any> extra) = 0;
    };
} // controller

#endif