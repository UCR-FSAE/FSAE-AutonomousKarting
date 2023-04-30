#ifndef CONTROLLER_MANAGER_ROS_HPP_
#define CONTROLLER_MANAGER_ROS_HPP_

#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_interfaces/action/control.hpp"
#include <nav2_util/lifecycle_node.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include "controller_manager/controller_interface.hpp"

namespace controller
{
    enum Algorithms {
        PID,
    };
    class ControllerManagerNode : public nav2_util::LifecycleNode
    {
        using ControlAction = control_interfaces::action::Control;
        using GoalHandleControlAction = rclcpp_action::ServerGoalHandle<ControlAction>;

        public:
            ControllerManagerNode();
            ~ControllerManagerNode();

            void registerControlAlgorithm(const Algorithms algo, const std::map<const std::string, boost::any> configs);


    protected:
        // implement the lifecycle interface
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

        // execution
        void p_execute(const std::shared_ptr<GoalHandleControlAction> goal_handle);
        int num_execution = 0;
        bool canExecute();
        std::thread execution_thread_;
        std::atomic<bool> stop_flag;


        /* Odometry */
        std::shared_ptr<nav_msgs::msg::Odometry> latest_odom;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr
            odom_sub_;
        std::mutex odom_mutex_;
        void onLatestOdomReceived(nav_msgs::msg::Odometry::SharedPtr msg);


        /**
         * action server
        */
        rclcpp_action::Server<ControlAction>::SharedPtr action_server_;
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ControlAction::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleControlAction> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandleControlAction> goal_handle);
        std::shared_ptr<GoalHandleControlAction> active_goal_; // use this to ensure that only one goal is executing at a time
        std::mutex active_goal_mutex_;

        /**
         * control publisher
        */
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<ackermann_msgs::msg::AckermannDriveStamped>> ackermann_publisher_;

        /**
         * control algorithm registry
        */
       std::shared_ptr<ControllerInterface> controller; 

    };

} // controller
#endif