// Copyright 2023 michael. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include "global_planner_manager/waypoint_follower.hpp"

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>
#include <cmath>

namespace gokart_planner
{
    WaypointFollowerServer::WaypointFollowerServer(const rclcpp::NodeOptions &options)
        : Node("waypoint_follower_server", options)
    {
        using namespace std::placeholders;

        RCLCPP_INFO(get_logger(), "Initializing");

        // Initialize the action server
        this->action_server_ = rclcpp_action::create_server<WaypointFollowerServer::WaypointFollowerAction>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "/global_waypoints",
            std::bind(&WaypointFollowerServer::handle_goal, this, _1, _2),
            std::bind(&WaypointFollowerServer::handle_cancel, this, _1),
            std::bind(&WaypointFollowerServer::handle_accepted, this, _1));

        this->declare_parameter("lookahead_dist", 5.0);
        this->lookahead_dist = this->get_parameter("lookahead_dist").as_double();

        this->vehicle_odom_sub_=this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&WaypointFollowerServer::odomCallback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "Listening for odom message on [%s]", this->vehicle_odom_sub_->get_topic_name());

        this->declare_parameter("loop_rate", 1.0);
    }

    WaypointFollowerServer::~WaypointFollowerServer()
    {
        RCLCPP_INFO(get_logger(), "Destroying");
    }

    void WaypointFollowerServer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        this->latest_odom_msg = msg;
    }
    

    rcl_interfaces::msg::SetParametersResult WaypointFollowerServer::parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        RCLCPP_INFO(get_logger(), "Look ahead distance updated");
        return result;
    }
    rclcpp_action::GoalResponse WaypointFollowerServer::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const gokart_planner::WaypointFollowerServer::WaypointFollowerAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid;
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse WaypointFollowerServer::handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<gokart_planner::WaypointFollowerServer::WaypointFollowerAction>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void WaypointFollowerServer::handle_accepted(const std::shared_ptr<gokart_planner::WaypointFollowerServer::GoalHandleWaypointFollower> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&WaypointFollowerServer::execute, this, _1), goal_handle}.detach();
    }
    void WaypointFollowerServer::execute(const std::shared_ptr<gokart_planner::WaypointFollowerServer::GoalHandleWaypointFollower> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(this->get_parameter("loop_rate").as_double());
        RCLCPP_INFO(this->get_logger(), "Loop rate: %f", this->get_parameter("loop_rate").as_double());

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<nav2_msgs::action::FollowWaypoints::Feedback>();
        auto result = std::make_shared<nav2_msgs::action::FollowWaypoints::Result>();

        while (true)
        {
            if (!this->latest_odom_msg)
            {
                // if odom msg not received
                RCLCPP_INFO(this->get_logger(), "Odom msg not received, waiting...");
                loop_rate.sleep();
                continue;
            }
            if (goal_handle->is_canceling())
            {
                // return the rest of the waypoint indexes
                int num_waypoint_missed = goal->poses.size() - feedback->current_waypoint;
                std::vector<int> missed_waypoints(num_waypoint_missed); // create an empty vector of num_waypoint_missed ints
                std::iota(missed_waypoints.begin(), missed_waypoints.end(), feedback->current_waypoint); // fill the vector with the numbers starting from feedback->current_waypoint
                result->missed_waypoints = missed_waypoints;
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "goal canceled");
                return;
            }

            // find the next waypoint
            uint32_t next_index = feedback->current_waypoint;
            geometry_msgs::msg::Pose vehicle_pose;
            vehicle_pose = this->latest_odom_msg->pose.pose;
            for (; next_index < goal->poses.size(); next_index++)
            {
                double dist = this->getDistance(vehicle_pose, goal->poses[next_index].pose);

                if (dist > this->lookahead_dist) // TODO: increase lookahead distance as speed increases
                {
                    break;
                }
            }

            if (next_index == goal->poses.size())
            {
                // reached the end
                std::vector<int> missed_waypoints(0); // create an empty vector of num_waypoint_missed ints
                result->missed_waypoints=missed_waypoints;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                return; 
            }
            // publish the next waypoint index
            feedback->current_waypoint = next_index;
            goal_handle->publish_feedback(feedback);
            // sleep
            loop_rate.sleep();
        }
    }
    double WaypointFollowerServer::getDistance(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2)
    {

        return sqrt(pow(pose1.position.x - pose2.position.x, 2) + pow(pose1.position.y - pose2.position.y, 2) + pow(pose1.position.z - pose2.position.z, 2));
    }

} // namespace nav2_waypoint_follower

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gokart_planner::WaypointFollowerServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
