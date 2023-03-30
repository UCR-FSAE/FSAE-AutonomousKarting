#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <nav2_msgs/msg/costmap_meta_data.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>

#ifndef LOCAL_PLANNER_MANAGER_NODE_HPP_
#define LOCAL_PLANNER_MANAGER_NODE_HPP_
using GetCostmap = nav2_msgs::srv::GetCostmap;

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

        void execute();


        /* timer */
        void costMapUpdateTimerCallback();
        rclcpp::TimerBase::SharedPtr costmap_update_timer;

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
        std::shared_ptr<rclcpp::Node> costmap_client_node;

        rclcpp::Client<GetCostmap>::SharedPtr costmap_client;
        void getLatestCostmap();
        void costmapCallback(const std::shared_future<GetCostmap::Response> &future_response);
        std::shared_ptr<nav2_msgs::msg::Costmap> latest_costmap;
        std::mutex costmap_mutex;
    };
} // local_planning
#endif