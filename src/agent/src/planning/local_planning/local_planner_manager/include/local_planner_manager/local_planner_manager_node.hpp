#ifndef LOCAL_PLANNER_MANAGER_NODE_HPP_
#define LOCAL_PLANNER_MANAGER_NODE_HPP_

#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <nav2_msgs/msg/costmap_meta_data.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <trajectory_generator/trajectory_generator_ros.hpp>
#include <trajectory_picker/trajectory_picker_ros.hpp>
#include "planning_interfaces/action/trajectory_generation.hpp"
#include "planning_interfaces/srv/trajectory_scoring.hpp"
#include "control_interfaces/action/control.hpp"

namespace local_planning
{
    class LocalPlannerManagerNode : public nav2_util::LifecycleNode
    {
        using TrajectoryGeneration = planning_interfaces::action::TrajectoryGeneration;
        using GoalHandleTrajectoryGeneration = rclcpp_action::ClientGoalHandle<TrajectoryGeneration>;

        using ControlAction = control_interfaces::action::Control;
        using GoalHandleControlAction = rclcpp_action::ClientGoalHandle<ControlAction>;
    public:
        LocalPlannerManagerNode();
        ~LocalPlannerManagerNode();

        void execute();


    protected:
        // implement the lifecycle interface
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;


        /* execution */
        void p_execute();
        rclcpp::TimerBase::SharedPtr execute_timer;
        int num_execution = 0;
        bool didReceiveAllMessages();
        bool canExecute();

        /* Waypoint */
        std::shared_ptr<geometry_msgs::msg::PoseStamped> latest_waypoint_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::ConstSharedPtr
            next_waypoint_sub_;
        std::mutex waypoint_mutex;
        void onLatestWaypointReceived(geometry_msgs::msg::Pose::SharedPtr msg);

        /* Odometry */
        std::shared_ptr<nav_msgs::msg::Odometry> latest_odom;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr
            odom_sub_;
        std::mutex odom_mutex_;
        void onLatestOdomReceived(nav_msgs::msg::Odometry::SharedPtr msg);

        /* Costmap */
        std::shared_ptr<nav2_msgs::msg::Costmap> p_GetLatestCostmap();
        std::shared_ptr<rclcpp::Node>
            costmap_node_;
        rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
        void p_PrintCostMapInfo(const nav2_msgs::msg::Costmap::SharedPtr msg);
        std::shared_ptr<nav2_msgs::msg::Costmap> latest_costmap_;

        /* Trajectory Generator */
        std::shared_ptr<local_planning::TrajectoryGeneratorROS>
            trajectory_generator_node_;
        std::unique_ptr<nav2_util::NodeThread> trajectory_generator_thread_;
        rclcpp_action::Client<TrajectoryGeneration>::SharedPtr trajectory_generator_client;
        void trajectory_generator_goal_response_callback(std::shared_future<GoalHandleTrajectoryGeneration::SharedPtr> future);
        void trajectory_generator_feedback_callback(
            GoalHandleTrajectoryGeneration::SharedPtr,
            const std::shared_ptr<const TrajectoryGeneration::Feedback> feedback);
        void trajectory_generator_result_callback(const GoalHandleTrajectoryGeneration::WrappedResult &result);
        void send_trajectory_generator_action(
            const nav2_msgs::msg::Costmap::SharedPtr costmap,
            const nav_msgs::msg::Odometry::SharedPtr odom,
            const geometry_msgs::msg::PoseStamped::SharedPtr next_waypoint);
        void register_generators();
        int num_generator_execution = 0;

        /* Trajectory Picker */
        std::shared_ptr<local_planning::TrajectoryPickerROS> trajectory_picker_node_;
        std::unique_ptr<nav2_util::NodeThread> trajectory_picker_thread_;
        void on_best_trajectory_publication_received(const planning_interfaces::msg::Trajectory::SharedPtr msg);
        rclcpp::Subscription<planning_interfaces::msg::Trajectory>::SharedPtr best_trajectory_subscriber_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<planning_interfaces::msg::Trajectory>> possible_trajectory_publisher_;

        /* control client */
        rclcpp_action::Client<ControlAction>::SharedPtr control_action_client_;
        std::string controllerServerRoute;
        void control_action_goal_response_callback(std::shared_future<GoalHandleControlAction::SharedPtr> future);
        void control_action_feedback_callback(GoalHandleControlAction::SharedPtr future, const std::shared_ptr<const ControlAction::Feedback> feedback);
        void control_action_result_callback(const GoalHandleControlAction::WrappedResult &result);
        void control_send_goal(const nav_msgs::msg::Path::SharedPtr path, float target_spd);

    };
} // local_planning
#endif