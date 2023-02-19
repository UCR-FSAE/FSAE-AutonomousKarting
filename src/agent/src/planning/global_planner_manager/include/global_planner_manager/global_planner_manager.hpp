// Copyright 2023 michael. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.



#include "nav2_util/lifecycle_node.hpp"
#include "global_planner_manager/waypoint_follower.hpp"
#include "nav_msgs/msg/path.hpp"
#include <experimental/filesystem>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#ifndef GOKART_GLOBAL_PLANNER_MANAGER_HPP_
#define GOKART_GLOBAL_PLANNER_MANAGER_HPP_
namespace fs = std::experimental::filesystem;

namespace gokart_planner
{
    enum class ManagerStatus
    {
        NoGoal = 0,
        GoalOk = 1,
    };

    class GlobalPlannerManager : public nav2_util::LifecycleNode
    {
        public:
            using WaypointFollowerAction = nav2_msgs::action::FollowWaypoints;
            using GoalHandleWaypointFollower = rclcpp_action::ClientGoalHandle<WaypointFollowerAction>;

            /**
             * @brief Constructor for gokart_planner::GlobalPlannerManager
             */
            GlobalPlannerManager();
            /**
             * @brief Destructor for gokart_planner::GlobalPlannerManager
             */
            ~GlobalPlannerManager();

        protected:
            /**
             * @brief Configures controller parameters and member variables
             *
             * Configures costmap;
             * @param state LifeCycle Node's state
             * @return Success or Failure
             * @throw pluginlib::PluginlibException When failed to initialize controller
             * plugin
             */
            nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
            /**
             * @brief Activates member variables
             *
             * Activates costmap
             * server
             * @param state LifeCycle Node's state
             * @return Success or Failure
             */
            nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
            /**
             * @brief Deactivates member variables
             *
             * Deactivates costmap
             * @param state LifeCycle Node's state
             * @return Success or Failure
             */
            nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
            /**
             * @brief Calls clean up states and resets member variables.
             *
             * Costmap clean up state is called, and resets rest of the
             * variables
             * @param state LifeCycle Node's state
             * @return Success or Failure
             */
            nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
            /**
             * @brief Called when in Shutdown state
             * @param state LifeCycle Node's state
             * @return Success or Failure
             */
            nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

            /**
             * @brief given a waypoint file path, read and output nav_msgs/path
             * @param p target file path to read
             * @return waypoint path
            */
            nav_msgs::msg::Path read_path_from_file(fs::path &p);
            std::vector<std::string> split(std::string s, std::string delimiter);

            void send_goal(const nav_msgs::msg::Path &p);
            void goal_response_callback(std::shared_future<GoalHandleWaypointFollower::SharedPtr> future);
            void feedback_callback(
                GoalHandleWaypointFollower::SharedPtr ptr,
                const std::shared_ptr<const WaypointFollowerAction::Feedback> feedback);
            void result_callback(const GoalHandleWaypointFollower::WrappedResult &result);
            void global_path_timer_callback();
            void autoStartOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

            void p_publish_debug_marker(const geometry_msgs::msg::Pose pose);

                // variables
                std::shared_ptr<nav_msgs::msg::Path> waypoints_;
            ManagerStatus current_manager_status_;

            rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoint_client_ptr_;
            std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose>> next_waypoint_publisher_;
            std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>> next_waypoint_visualization_publisher_;
            std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_publisher_;
            rclcpp::TimerBase::SharedPtr global_path_publisher_timer_;
    };
}
#endif 