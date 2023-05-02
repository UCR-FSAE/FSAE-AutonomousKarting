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
        bool canExecute(std::shared_ptr<const ControlAction::Goal> goal);
        rclcpp::TimerBase::SharedPtr execution_timer;
        void execution_callback();


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

        /**
         * Helper functions
        */
        ackermann_msgs::msg::AckermannDriveStamped p_controlResultToAckermannStamped(ControlResult controlResult);



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

        float closeness_threshold = 1.0;
        std::string frame_id;
    };

} // controller
#endif