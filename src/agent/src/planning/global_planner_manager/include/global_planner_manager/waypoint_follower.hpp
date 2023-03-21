// Copyright 2023 michael. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#ifndef GLOBAL_PLANNER_MANAGER__WAYPOINT_FOLLOWER_HPP_
#define GLOBAL_PLANNER_MANAGER__WAYPOINT_FOLLOWER_HPP_

#include <mutex>
#include <string>
#include <vector>
#include <nav_msgs/msg/odometry.hpp>
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace gokart_planner
{
    /**
     * @brief An action server that follows a set of global waypoints.
     *
     * This class implements an action server that receives a set of global waypoints and follows them using odometry information.
     * The action server provides feedback to the client about the progress of the goal and returns a result when the goal is completed.
     */
    class WaypointFollowerServer : public rclcpp::Node
    {
        public:
            using WaypointFollowerAction = nav2_msgs::action::FollowWaypoints;
            using GoalHandleWaypointFollower = rclcpp_action::ServerGoalHandle<WaypointFollowerAction>;
            /**
             * @brief Construct a new WaypointFollowerServer object.
             *
             * @param options The options to use when constructing the node.
             */
            WaypointFollowerServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
            /**
             * @brief Destroy the WaypointFollowerServer object.
             */
            ~WaypointFollowerServer();
        protected:
            /**
             * @brief Handle a new goal request.
             *
             * This function is called when the action server receives a new goal request. It is responsible for deciding whether to accept or reject the goal. In this implementation, the goal is accepted if an odometry message has been received, and rejected otherwise.
             *
             * @param uuid The UUID of the goal request.
             * @param goal The goal request.
             * @return A response indicating whether the goal was accepted or rejected.
             */
            rclcpp_action::GoalResponse handle_goal(
                const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const WaypointFollowerAction::Goal> goal);

            /**
             * @brief Handle a request to cancel a goal.
             *
             * This function is called when the action client sends a request to cancel a goal. It is responsible for deciding whether to accept or reject the request.
             *
             * @param goal_handle The handle of the goal to be cancelled.
             * @return A response indicating whether the cancel request was accepted or rejected.
             */
            rclcpp_action::CancelResponse handle_cancel(
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<WaypointFollowerAction>> goal_handle);
            /**
             * @brief Handle an accepted goal.
             *
             * This function is called when the action server accepts a goal request. It is responsible for starting the execution of the goal.
             *
             * @param goal_handle The handle of the accepted goal.
             */
            void handle_accepted(const std::shared_ptr<GoalHandleWaypointFollower> goal_handle);
            /**
             * @brief Execute an accepted goal.
             *
             * This function is called to execute an accepted goal. It is responsible for sending feedback to the action client and returning a result when the goal is completed.
             *
             * @param goal_handle The handle of the accepted goal.
             */
            void execute(const std::shared_ptr<GoalHandleWaypointFollower> goal_handle);
            /**
             * @brief Handle a request to set parameters.
             *
             * This function is called when the action server receives a request to set parameters. It is responsible for applying the new parameter values and returning a result indicating whether the request was successful.
             *
             * @param parameters The new parameter values.
             * @return A result indicating whether the request was successful.
             */
            rcl_interfaces::msg::SetParametersResult parametersCallback(
                const std::vector<rclcpp::Parameter> &parameters);

            /**
             * @brief Calculate the Euclidean distance between two poses.
             *
             * This function calculates the Euclidean distance between two poses in 3D space.
             *
             * @param pose1 The first pose.
             * @param pose2 The second pose.
             * @return The Euclidean distance between the two poses.
             */
            double getDistance(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2);
            /**
             * @brief Handle an incoming odometry message.
             *
             * This function is called whenever an odometry message is received. It updates the latest odometry message stored in the class.
             *
             * @param msg The incoming odometry message.
             */
            double get_current_speed(const geometry_msgs::msg::Twist &current_speed);
            /**
             * @brief Handle an incoming odometry message.
             *
             * This function is called whenever an odometry message is received. It updates the latest odometry message stored in the class.
             *
             * @param msg The incoming odometry message.
             */
            void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

            // variables
            OnSetParametersCallbackHandle::SharedPtr callback_handle_;
            rclcpp_action::Server<WaypointFollowerAction>::SharedPtr action_server_;
            float lookahead_dist;

            std::string file_path;
            std::string line;
            int low_bound;
            int high_bound;
            int dist_from_file;
            int pos_for_file;
            std::string token_for_file;
            std::vector<std::vector<int>> speed_bounds_and_lookahead_distance;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_odom_sub_;
            std::shared_ptr<nav_msgs::msg::Odometry> latest_odom_msg;
            std::mutex odom_mutex_;
            visualization_msgs::msg::Marker vis_marker_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_marker_publisher_;
            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    };

} // namespace gokart_planner

#endif // GLOBAL_PLANNER_MANAGER__WAYPOINT_FOLLOWER_HPP_