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
            /**
             * check if is close enough to the last index of the trajectory
             * 
             * Note: this function assumes trajectory and odom are in the same frame of reference
            */
            bool isDone(const nav_msgs::msg::Path::SharedPtr trajectory, const nav_msgs::msg::Odometry::SharedPtr odom, float closeness_threshold=1)
            {
                // Get the last point in the trajectory
                geometry_msgs::msg::PoseStamped last_pose = trajectory->poses.back();

                // Get the current position of the robot
                geometry_msgs::msg::Pose current_pose = odom->pose.pose;

                // Calculate the Euclidean distance between the last point in the trajectory and the current position of the robot
                double distance = sqrt(pow(last_pose.pose.position.x - current_pose.position.x, 2) +
                                    pow(last_pose.pose.position.y - current_pose.position.y, 2) +
                                    pow(last_pose.pose.position.z - current_pose.position.z, 2));

                // Check if the distance is less than a certain threshold
                if (distance < closeness_threshold) {
                    return true;
                } else {
                    return false;
                }
            }

    };
} // controller

#endif