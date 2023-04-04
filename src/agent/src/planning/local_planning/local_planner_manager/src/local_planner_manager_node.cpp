#include "local_planner_manager/local_planner_manager_node.hpp"

namespace local_planning
{
  LocalPlannerManagerNode::LocalPlannerManagerNode() : LifecycleNode("manager", "local_planner", true)
  {
    this->declare_parameter("manager_rate", 0.5);
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
    this->costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/local_costmap/costmap", rclcpp::SystemDefaultsQoS(),
        std::bind(&LocalPlannerManagerNode::onLatestCostmapReceived, this, std::placeholders::_1));

    trajectory_generator_node_ = std::make_shared<local_planning::TrajectoryGeneratorROS>("generator", std::string{get_namespace()}, "trajectory");
    trajectory_generator_thread_ = std::make_unique<nav2_util::NodeThread>(trajectory_generator_node_);
    trajectory_generator_node_->configure();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn LocalPlannerManagerNode::on_activate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(this->get_logger(), "on_activate");
    double loop_rate = this->get_parameter("manager_rate").as_double();
    RCLCPP_INFO(this->get_logger(), "loop_rate: %.3f", loop_rate);
    execute_timer = create_wall_timer(std::chrono::milliseconds(int(loop_rate * 1000)),
                                      std::bind(&LocalPlannerManagerNode::execute, this));
    trajectory_generator_node_->activate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn LocalPlannerManagerNode::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(this->get_logger(), "on_deactivate");
    execute_timer->cancel();
    trajectory_generator_node_->deactivate();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn LocalPlannerManagerNode::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(this->get_logger(), "on_cleanup");
    this->next_waypoint_sub_ = nullptr;
    this->odom_sub_ = nullptr;
    this->costmap_sub_ = nullptr;
    trajectory_generator_node_->cleanup();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn LocalPlannerManagerNode::on_shutdown(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(this->get_logger(), "on_shutdown");
    trajectory_generator_node_->shutdown();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  void LocalPlannerManagerNode::onLatestWaypointReceived(geometry_msgs::msg::Pose::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(waypoint_mutex);
    this->latest_waypoint_ = msg;
    // RCLCPP_INFO(this->get_logger(), "latest_waypoint_ received");
  }

  void LocalPlannerManagerNode::onLatestOdomReceived(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    this->latest_odom = msg;
    // RCLCPP_INFO(this->get_logger(), "odom received");
  }

  void LocalPlannerManagerNode::onLatestCostmapReceived(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(occu_map_mutex);
    this->latest_occu_map = msg;
  }
  void LocalPlannerManagerNode::execute()
  {
    std::lock_guard<std::mutex> occumap_lock(occu_map_mutex);
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    std::lock_guard<std::mutex> waypoint_lock(waypoint_mutex);

    if (this->didReceiveAllMessages())
    {
      num_execution += 1;
      // RCLCPP_INFO(this->get_logger(), "STEPPING");
    }

    num_execution -= 1;

  }

  bool LocalPlannerManagerNode::canExecute()
  {
    if (this->didReceiveAllMessages() && num_execution < 1)
    {
      return true; 
    } 
    return false;
  }

  bool LocalPlannerManagerNode::didReceiveAllMessages()
  {
    return this->latest_occu_map != nullptr && this->latest_odom != nullptr && this->latest_waypoint_ != nullptr; 
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