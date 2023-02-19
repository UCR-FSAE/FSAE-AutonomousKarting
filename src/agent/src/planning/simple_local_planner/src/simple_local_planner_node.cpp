// Copyright 2023 michael. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include "simple_local_planner/simple_local_planner_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <string>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <math.h>
using namespace std::placeholders;

namespace local_planner
{
  SimpleLocalPlannerNode::SimpleLocalPlannerNode() : nav2_util::LifecycleNode("simple_local_planner_node", "", true)
  {
    RCLCPP_INFO(get_logger(), "Initializing");
    this->declare_parameter("loop_rate", 30.0);
    this->declare_parameter("target_spd", 2.0);
  } 
  SimpleLocalPlannerNode::~SimpleLocalPlannerNode()
  {
      RCLCPP_INFO(get_logger(), "Destroying");
  }

  nav2_util::CallbackReturn SimpleLocalPlannerNode::on_configure(const rclcpp_lifecycle::State &state) 
  {
    this->next_waypoint_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/next_waypoint", rclcpp::SystemDefaultsQoS(),
        std::bind(&SimpleLocalPlannerNode::onLatestWaypointReceived, this, std::placeholders::_1));

    this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/carla/ego_vehicle/odometry", rclcpp::SystemDefaultsQoS(),
        std::bind(&SimpleLocalPlannerNode::onLatestOdomReceived, this, std::placeholders::_1));
    this->path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/local_planner/path", 10);
    this->control_action_client_ = rclcpp_action::create_client<ControlAction>(
        this,
        "pid_control");
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn SimpleLocalPlannerNode::on_activate(const rclcpp_lifecycle::State &state) 
  {
    this->path_publisher_->on_activate();
    this->shouldExecute = true;
    std::thread{std::bind(&SimpleLocalPlannerNode::execute, this)}.detach();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn SimpleLocalPlannerNode::on_deactivate(const rclcpp_lifecycle::State &state) 
  {
    this->shouldExecute = false;
    this->path_publisher_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn SimpleLocalPlannerNode::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn SimpleLocalPlannerNode::on_shutdown(const rclcpp_lifecycle::State &state) 
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }
  void SimpleLocalPlannerNode::onLatestWaypointReceived(geometry_msgs::msg::Pose::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(waypoint_mutex);
    this->latest_waypoint_ = msg;
  }
  void SimpleLocalPlannerNode::onLatestOdomReceived(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    this->latest_odom = msg;
  }

  void SimpleLocalPlannerNode::execute()
  {
    RCLCPP_INFO(this->get_logger(), "running simple local planner");
    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::Rate loop_rate(this->get_parameter("loop_rate").as_double());
    RCLCPP_INFO(this->get_logger(), "Loop rate: %f", this->get_parameter("loop_rate").as_double());

    while (this->shouldExecute)
    {
      if (this->is_all_data_synced()) {
        nav_msgs::msg::Path::SharedPtr path = this->find_trajectory();
        if (path != nullptr) {
          this->path_publisher_->publish(*path);

          // TODO: change spd as needed
          // TODO: figure out if change spd should happen at local planner level?
          this->send_goal(path, this->get_parameter("target_spd").as_double()); 
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "Odom or waypoint msg not arrived");
      }

      loop_rate.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "Finished execution");
  }
  void SimpleLocalPlannerNode::send_goal(const nav_msgs::msg::Path::SharedPtr path, float target_spd)
  {
    if (!this->control_action_client_->wait_for_action_server())
    {
      RCLCPP_ERROR(this->get_logger(), "Control action server not available, not sending goal");
      return;
    }
      auto goal_msg = ControlAction::Goal();
      goal_msg.path = *path;
      goal_msg.target_spd = target_spd;

      RCLCPP_INFO(this->get_logger(), "Sending goal: target_spd: %f", target_spd);

      auto send_goal_options = rclcpp_action::Client<ControlAction>::SendGoalOptions();
      send_goal_options.goal_response_callback =
          std::bind(&SimpleLocalPlannerNode::goal_response_callback, this, _1);
      send_goal_options.feedback_callback =
          std::bind(&SimpleLocalPlannerNode::feedback_callback, this, _1, _2);
      send_goal_options.result_callback =
          std::bind(&SimpleLocalPlannerNode::result_callback, this, _1);
      this->control_action_client_->async_send_goal(goal_msg, send_goal_options);
    }
  void SimpleLocalPlannerNode::result_callback(const GoalHandleControlAction::WrappedResult &result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
  }
  void SimpleLocalPlannerNode::feedback_callback(GoalHandleControlAction::SharedPtr, const std::shared_ptr<const ControlAction::Feedback> feedback)
  {

  }
  void SimpleLocalPlannerNode::goal_response_callback(std::shared_future<GoalHandleControlAction::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }
  nav_msgs::msg::Path::SharedPtr SimpleLocalPlannerNode::find_trajectory()
  {
    // TODO: implement more advanced path finding algorithm here

    // get the transform from map to ego_vehicle
    geometry_msgs::msg::TransformStamped t;
    std::string fromFrameRel = this->latest_odom->header.frame_id;
    std::string toFrameRel = this->latest_odom->child_frame_id;
    try
    {
      t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      return nullptr;
    }
    // construct the message
    std_msgs::msg::Header header;
    header.frame_id = toFrameRel;
    header.stamp = this->get_clock()->now();

    nav_msgs::msg::Path path;
    path.header = header;

    geometry_msgs::msg::PoseStamped child_pose;
    geometry_msgs::msg::PoseStamped ps;
    ps.header = header;
    ps.pose = *this->latest_waypoint_;
    tf2::doTransform(ps, child_pose, t);

    path.poses.push_back(child_pose);
    return std::make_shared<nav_msgs::msg::Path>(path);
  }

  bool SimpleLocalPlannerNode::is_all_data_synced()
  {
    if(this->latest_odom != nullptr && this->latest_waypoint_ != nullptr) {
      return true;
    }
    return false; 
  }

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<local_planner::SimpleLocalPlannerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
