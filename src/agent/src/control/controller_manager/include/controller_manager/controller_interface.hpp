#ifndef CONTROLLER_INTERFACE_HPP_
#define CONTROLLER_INTERFACE_HPP_
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <map>
#include <boost/any.hpp>
#include <math.h>
#include <mutex>

struct ControlResult {
  ackermann_msgs::msg::AckermannDrive drive;
  size_t waypoint_index;
};

namespace controller 
{
    class ControllerInterface
    {
        public: 

            virtual void setTrajectory(const nav_msgs::msg::Path::SharedPtr trajectory) = 0;
            virtual ControlResult compute(const nav_msgs::msg::Odometry::SharedPtr odom,
                                        std::mutex& odom_mutex,
                                        const std::map<const std::string, boost::any> extra) = 0;

    };
} // controller

#endif