#ifndef CONTROLLER_MANAGER_NODE_HPP_
#define CONTROLLER_MANAGER_NODE_HPP_

#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_interfaces/action/control.hpp"
#include <nav2_util/lifecycle_node.hpp>

namespace controller
{
    class ControllerManagerNode : public nav2_util::LifecycleNode
    {
        using ControlAction = control_interfaces::action::Control;
        using GoalHandleControlAction = rclcpp_action::ServerGoalHandle<ControlAction>;

        public:
            ControllerManagerNode();
            ~ControllerManagerNode();


    };

} // controller
#endif