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
#include <sstream>
#include <experimental/filesystem>


namespace fs = std::experimental::filesystem;

namespace gokart_planner
{
    WaypointFollowerServer::WaypointFollowerServer(const rclcpp::NodeOptions &options)
        : Node("waypoint_follower_server", options)
    {
        using namespace std::placeholders;

        RCLCPP_INFO(get_logger(), "Initializing");
        // Initialize te visualization maker publisher
        this->vis_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/waypoint_follower/next_waypoint_visualization", 10);
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

        this->declare_parameter("speed_zone_and_lookahead_distance", "./agent/src/planning/global_planner_manager/params/lookahead_distance_vs_speed.txt");
        this->declare_parameter("debug", true);
        this->declare_parameter("lookahead_dist", 5.0);

        this->lookahead_dist = this->get_parameter("lookahead_dist").as_double();
        this->vehicle_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&WaypointFollowerServer::odomCallback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "Listening for odom message on [%s]", this->vehicle_odom_sub_->get_topic_name());
        
        this->declare_parameter("loop_rate", 1.0);
        file_path = this->get_parameter("speed_zone_and_lookahead_distance").get_parameter_value().get<std::string>();
        RCLCPP_INFO(get_logger(), "lookahead_distance_and_current_speed_file_path: %s", file_path.c_str());
        fs::path p(file_path.c_str());
        if (!fs::exists(p))
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open file");
        }
        std::ifstream file(p.c_str());
        if (file.is_open())
        {
            while (std::getline(file, line))
            {
                std::string token;
                while ((pos_for_file = line.find(",")) != std::string::npos)
                {
                    token_for_file = line.substr(0, pos_for_file);
                    low_bound = std::stof(token_for_file);
                    line.erase(0, pos_for_file + 1);
                    pos_for_file = line.find(",");
                    token_for_file = line.substr(0, pos_for_file);
                    high_bound = std::stof(token_for_file);
                    line.erase(0, pos_for_file + 1);
                    dist_from_file = std::stof(line);
                    speed_bounds_and_lookahead_distance.push_back({low_bound, high_bound, dist_from_file});
                    
                }
            }
            file.close();
        }
        else
        {
            std::cerr << "Failed to open file" << std::endl;
        }
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
            double current_speed = this->get_current_speed(this->latest_odom_msg->twist.twist);
            for (int i = 0; i < speed_bounds_and_lookahead_distance.size(); i++)
            {
                if (speed_bounds_and_lookahead_distance[i][0] <= current_speed < speed_bounds_and_lookahead_distance[i][1])
                {
                    this->lookahead_dist = speed_bounds_and_lookahead_distance[i][2];
                    break;
                }
            }
            for (; next_index < goal->poses.size(); next_index++)
            {

                double dist = this->getDistance(vehicle_pose, goal->poses[next_index].pose);

                if (dist > this->lookahead_dist)
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
            // publish the visualization marker
            geometry_msgs::msg::PoseStamped next_waypoint_pose = goal->poses[next_index];
            next_waypoint_pose.header.frame_id = "map"; 
            geometry_msgs::msg::PoseStamped transformed_waypoint_pose;
            try {
                transformed_waypoint_pose = tf_buffer_->transform(next_waypoint_pose, this->latest_odom_msg->child_frame_id, tf2::durationFromSec(2.0));
            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR(this->get_logger(), "Failed to transform waypoint pose to local frame: %s", ex.what());
                return; // or handle the error as needed
            }
            visualization_msgs::msg::Marker vis_marker_;
            vis_marker_.type = visualization_msgs::msg::Marker::SPHERE;
            vis_marker_.action = visualization_msgs::msg::Marker::ADD;
            vis_marker_.pose = transformed_waypoint_pose.pose;

            // Set the marker scale
            vis_marker_.scale.x = 2.0;
            vis_marker_.scale.y = 2.0;
            vis_marker_.scale.z = 2.0;

            // Set the marker color
            vis_marker_.color.r = 1.0;
            vis_marker_.color.g = 1.0;
            vis_marker_.color.b = 0.0;
            vis_marker_.color.a = 1.0;

            vis_marker_.lifetime = rclcpp::Duration(std::chrono::seconds(5));
            std_msgs::msg::Header header;
            header.frame_id = this->latest_odom_msg->child_frame_id;
            header.stamp = this->get_clock()->now();
            vis_marker_.header = header;

            if (this->get_parameter("debug").as_bool())
            {
                this->vis_marker_publisher_->publish(vis_marker_);
            }
            // publish the next waypoint index
            feedback->current_waypoint = next_index;
            RCLCPP_INFO(get_logger(), "Waypoint published: %d", next_index);
            goal_handle->publish_feedback(feedback);
            // sleep
            loop_rate.sleep();
        }
    }
    double WaypointFollowerServer::getDistance(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2)
    {

        return sqrt(pow(pose1.position.x - pose2.position.x, 2) + pow(pose1.position.y - pose2.position.y, 2) + pow(pose1.position.z - pose2.position.z, 2));
    }

    double WaypointFollowerServer::get_current_speed(const geometry_msgs::msg::Twist &current_speed)
    {

        return sqrt(pow(current_speed.linear.x, 2) + pow(current_speed.linear.y, 2) + pow(current_speed.linear.z, 2));
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
