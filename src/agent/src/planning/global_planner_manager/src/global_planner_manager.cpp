// Copyright 2023 michael. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include <cstdio>
#include "global_planner_manager/global_planner_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include <experimental/filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <memory>

namespace fs = std::experimental::filesystem;
namespace gokart_planner
{
  GlobalPlannerManager::GlobalPlannerManager() : LifecycleNode("global_planner_manager", "", true)
  {
    RCLCPP_INFO(get_logger(), "Creating GlobalPlannerManager Node");
    this->declare_parameter("waypoint_file_path", "./data/recording.txt");
        this->declare_parameter("debug", false);
    
    if (this->get_parameter("debug").as_bool())
    {
        auto ret = rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG); // enable or disable debug
    }
    RCLCPP_INFO(this->get_logger(), "GlobalPlannerManager initialized with Debug Mode = [%s]", this->get_parameter("debug").as_bool() ? "YES" : "NO");

    this->current_manager_status_ = ManagerStatus::NoGoal;
  }
  
  GlobalPlannerManager::~GlobalPlannerManager()
  {
    RCLCPP_INFO(get_logger(), "Destroying GlobalPlannerManager");
  }

  nav2_util::CallbackReturn
  GlobalPlannerManager::on_configure(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Configuring");
    this->next_waypoint_visualization_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/next_waypoint_visualization", 10);    

    this->next_waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/next_waypoint", 10);
    this->global_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);
    this->global_path_publisher_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&GlobalPlannerManager::global_path_timer_callback, this));
    // read from file
    this->follow_waypoint_client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
        this,
        "global_waypoints");
    std::string file_path =
        this->get_parameter("waypoint_file_path").get_parameter_value().get<std::string>();
    RCLCPP_INFO(get_logger(), "waypoint_file_path: %s", file_path.c_str());

    fs::path p(file_path.c_str());
    if (!fs::exists(p)){
      return nav2_util::CallbackReturn::FAILURE;
    }

    this->waypoints_ = std::make_shared<nav_msgs::msg::Path>(this->read_path_from_file(p));
    RCLCPP_INFO(get_logger(), "[%d] waypoints read.", this->waypoints_->poses.size());
    // send goal
    this->send_goal(*this->waypoints_);
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  GlobalPlannerManager::on_activate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Activating");
    this->next_waypoint_publisher_->on_activate();
    this->next_waypoint_visualization_publisher_->on_activate();
    this->global_path_publisher_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  GlobalPlannerManager::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_DEBUG(get_logger(), "Deactivating");

    this->next_waypoint_visualization_publisher_->on_deactivate();
    
    this->next_waypoint_publisher_->on_deactivate();
    this->global_path_publisher_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  GlobalPlannerManager::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_DEBUG(get_logger(), "Cleaning up");
    // nav2_waypoint_follower_->cleanup();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  GlobalPlannerManager::on_shutdown(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_DEBUG(get_logger(), "Shutting Down");
    // nav2_waypoint_follower_->shutdown();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  void GlobalPlannerManager::send_goal(const nav_msgs::msg::Path &p)
  {
    using namespace std::placeholders;
    auto goal_msg = WaypointFollowerAction::Goal();
    goal_msg.poses = p.poses;
    auto send_goal_options = rclcpp_action::Client<WaypointFollowerAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&GlobalPlannerManager::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&GlobalPlannerManager::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&GlobalPlannerManager::result_callback, this, _1);
    this->follow_waypoint_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void GlobalPlannerManager::goal_response_callback(std::shared_future<GoalHandleWaypointFollower::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server, transition back to unconfigured state");
      this->cleanup();
      this->current_manager_status_ = ManagerStatus::NoGoal;
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "Goal accepted by waypoint follower server, manager is ready to transition to Activate State");
      this->current_manager_status_ = ManagerStatus::GoalOk;
    }
  }
  void GlobalPlannerManager::p_publish_debug_marker(const geometry_msgs::msg::Pose pose)
  {
    if (this->get_parameter("debug").as_bool())
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "global_planner_manager";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = pose;
      marker.scale.x = 10.0;
      marker.scale.y = 10.0;
      marker.scale.z = 10.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.lifetime = rclcpp::Duration(0, 1000000000); // 1 second
      this->next_waypoint_visualization_publisher_->publish(marker);
    }
  }

  void GlobalPlannerManager::feedback_callback(GoalHandleWaypointFollower::SharedPtr ptr, const std::shared_ptr<const WaypointFollowerAction::Feedback> feedback)
  {
    uint32_t waypoint_index = feedback->current_waypoint;
    if (waypoint_index < this->waypoints_->poses.size())
    {
      geometry_msgs::msg::Pose pose = this->waypoints_->poses[waypoint_index].pose;
      this->next_waypoint_publisher_->publish(pose);

      this->p_publish_debug_marker(pose);
      
      return;
    } 
    RCLCPP_ERROR(get_logger(), "Abnormal waypoint index: [%d]", waypoint_index);
  }

  void GlobalPlannerManager::result_callback(const GoalHandleWaypointFollower::WrappedResult &result)
  {
    RCLCPP_INFO(get_logger(), "result received");
  }

  nav_msgs::msg::Path GlobalPlannerManager::read_path_from_file(fs::path &p)
  {
    RCLCPP_INFO(get_logger(), "Reading waypoints from %s", p.c_str());
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = this->now();
    std::ifstream file(p.c_str());
    if (file.is_open())
    {
      std::string line;
      while (std::getline(file, line))
      {
        std::vector<std::string> splitted = this->split(line, ",");
        if (splitted.size() == 3)
        {
          geometry_msgs::msg::PoseStamped ps;
          ps.pose.position.x = std::stof(splitted[0]);
          ps.pose.position.y = std::stof(splitted[1]);
          ps.pose.position.z = std::stof(splitted[2]);
          path.poses.push_back(ps);
        }
        
      }
      file.close();
    }
    return path;
  }

  std::vector<std::string> GlobalPlannerManager::split(std::string s, std::string delimiter)
  {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos)
    {
      token = s.substr(pos_start, pos_end - pos_start);
      pos_start = pos_end + delim_len;
      res.push_back(token);
    }

    res.push_back(s.substr(pos_start));
    return res;
  }

  void GlobalPlannerManager::global_path_timer_callback()
  {
    if (this->waypoints_ && this->global_path_publisher_->is_activated())
    {
      this->global_path_publisher_->publish(*this->waypoints_);
    }
  }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gokart_planner::GlobalPlannerManager>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}