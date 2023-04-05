#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <nav2_msgs/msg/costmap_meta_data.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <trajectory_generator/trajectory_generator_ros.hpp>
#include <trajectory_scorer/trajectory_scorer_ros.hpp>
#include <trajectory_picker/trajectory_picker_ros.hpp>


#ifndef LOCAL_PLANNER_MANAGER_NODE_HPP_
#define LOCAL_PLANNER_MANAGER_NODE_HPP_

namespace local_planning
{
    class LocalPlannerManagerNode : public nav2_util::LifecycleNode
    {
    public:
        LocalPlannerManagerNode();
        ~LocalPlannerManagerNode();

    protected:
        // implement the lifecycle interface
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;


        /* execution */
        void execute();
        
        rclcpp::TimerBase::SharedPtr execute_timer;
        int num_execution = 0;
        bool didReceiveAllMessages();
        bool canExecute();

        /* Waypoint */
        std::shared_ptr<geometry_msgs::msg::Pose> latest_waypoint_;
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

        /* Trajectory Generator */
        std::shared_ptr<local_planning::TrajectoryGeneratorROS>
            trajectory_generator_node_;
        std::unique_ptr<nav2_util::NodeThread> trajectory_generator_thread_;

        /* Trajectory Scorer */
        std::shared_ptr<local_planning::TrajectoryScorerROS> trajectory_scorer_node_;
        std::unique_ptr<nav2_util::NodeThread> trajectory_scorer_thread_;

        /* Trajectory Picker */
        std::shared_ptr<local_planning::TrajectoryPickerROS> trajectory_picker_node_;
        std::unique_ptr<nav2_util::NodeThread> trajectory_picker_thread_;
    };
} // local_planning
#endif