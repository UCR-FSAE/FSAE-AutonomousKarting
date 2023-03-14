// Copyright 2023 michael. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#ifndef SIMPLE_LOCAL_PLANNER_SIMPLE_LOCAL_PLANNER_NODE_HPP_
#define SIMPLE_LOCAL_PLANNER_SIMPLE_LOCAL_PLANNER_NODE_HPP_

#include <memory>
#include <string>

#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_interfaces/action/control.hpp"

namespace local_planner
{

    class SimpleLocalPlannerNode : public nav2_util::LifecycleNode
    {
    public:
        using ControlAction = control_interfaces::action::Control;
        using GoalHandleControlAction = rclcpp_action::ClientGoalHandle<ControlAction>;
        SimpleLocalPlannerNode();
        ~SimpleLocalPlannerNode();

        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

    private:
        void execute();
        void onLatestWaypointReceived(geometry_msgs::msg::Pose::SharedPtr msg);
        void onLatestOdomReceived(nav_msgs::msg::Odometry::SharedPtr msg);
        bool is_all_data_synced();
        void send_goal(const nav_msgs::msg::Path::SharedPtr path, float target_spd);
        nav_msgs::msg::Path::SharedPtr find_trajectory();
        void result_callback(const GoalHandleControlAction::WrappedResult &result);
        void feedback_callback(GoalHandleControlAction::SharedPtr future, const std::shared_ptr<const ControlAction::Feedback> feedback);
        void goal_response_callback(std::shared_future<GoalHandleControlAction::SharedPtr> future);
        bool shouldUpdateTrajectory();

        // start of variable section
        std::shared_ptr<geometry_msgs::msg::Pose> latest_waypoint_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::ConstSharedPtr
            next_waypoint_sub_;
        std::mutex waypoint_mutex;

        std::shared_ptr<nav_msgs::msg::Odometry> latest_odom;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr
            odom_sub_;
        std::mutex odom_mutex_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> path_publisher_;
        bool shouldExecute;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        rclcpp_action::Client<ControlAction>::SharedPtr control_action_client_;

        int num_goals_processing = 0;
        // TODO: add costmap receiving
        };

} // namespace local_planner

#endif // SIMPLE_LOCAL_PLANNER_SIMPLE_LOCAL_PLANNER_NODE_HPP_
