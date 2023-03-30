#include "local_planner_manager/local_planner_manager_node.hpp"

namespace local_planning
{
  LocalPlannerManagerNode::LocalPlannerManagerNode() : LifecycleNode("manager_node", "local_planner", true)
  {
    this->declare_parameter("loop_rate", 0.5);
  }

  LocalPlannerManagerNode ::~LocalPlannerManagerNode()
  {
  }

  nav2_util::CallbackReturn LocalPlannerManagerNode::on_configure(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(this->get_logger(), "on_configure");
    this->next_waypoint_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/next_waypoint", rclcpp::SystemDefaultsQoS(),
        std::bind(&LocalPlannerManagerNode::onLatestWaypointReceived, this, std::placeholders::_1));
    this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/carla/ego_vehicle/odometry", rclcpp::SystemDefaultsQoS(),
        std::bind(&LocalPlannerManagerNode::onLatestOdomReceived, this, std::placeholders::_1));
    costmap_client_node = rclcpp::Node::make_shared("costmap_client");
    costmap_client = costmap_client_node->create_client<GetCostmap>("/local_costmap/get_costmap");

    costmap_update_timer = create_wall_timer(std::chrono::milliseconds(int(this->get_parameter("loop_rate").as_double()*1000)), 
    std::bind(&LocalPlannerManagerNode::costMapUpdateTimerCallback, this));

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn LocalPlannerManagerNode::on_activate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(this->get_logger(), "on_activate");

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn LocalPlannerManagerNode::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating");

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn LocalPlannerManagerNode::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(this->get_logger(), "on_cleanup");

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn LocalPlannerManagerNode::on_shutdown(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(this->get_logger(), "on_shutdown");

    return nav2_util::CallbackReturn::SUCCESS;
  }

  void LocalPlannerManagerNode::onLatestWaypointReceived(geometry_msgs::msg::Pose::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(waypoint_mutex);
    this->latest_waypoint_ = msg;
  }

  void LocalPlannerManagerNode::onLatestOdomReceived(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    this->latest_odom = msg;
  }

  void LocalPlannerManagerNode::costmapCallback(const std::shared_future<GetCostmap::Response> &future_response)
  {
    std::lock_guard<std::mutex> lock(costmap_mutex);
    auto response = future_response.get();
    latest_costmap = std::make_shared<nav2_msgs::msg::Costmap>(response.map);
    RCLCPP_INFO(this->get_logger(), "Response");
  }

  void LocalPlannerManagerNode::getLatestCostmap()
  {
    auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
    request->specs.layer = "obstacle_layer";
    request->specs.resolution = 1.0;
    request->specs.size_x = 50;
    request->specs.size_y = 80;
    request->specs.origin = this->latest_odom->pose.pose;
    request->specs.update_time = this->get_clock()->now();
    auto future_result = costmap_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(costmap_client_node, future_result, std::chrono::nanoseconds(500)) !=
        rclcpp::FutureReturnCode::SUCCESS)
        {
            // failed
            RCLCPP_INFO(this->get_logger(), "Failed");
        } else {
            // success
            RCLCPP_INFO(this->get_logger(), "Success");
        }
  }
  void LocalPlannerManagerNode::costMapUpdateTimerCallback()
  {
    this->getLatestCostmap();
  }

} // local_planning

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<local_planning::LocalPlannerManagerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}