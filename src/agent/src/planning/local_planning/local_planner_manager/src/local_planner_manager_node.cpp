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
   

    trajectory_generator_node_ = std::make_shared<local_planning::TrajectoryGeneratorROS>("generator", std::string{get_namespace()}, "trajectory");
    trajectory_generator_thread_ = std::make_unique<nav2_util::NodeThread>(trajectory_generator_node_);
    trajectory_generator_node_->configure();

    trajectory_scorer_node_ = std::make_shared<local_planning::TrajectoryScorerROS>("scorer", std::string{get_namespace()}, "trajectory");
    trajectory_scorer_thread_ = std::make_unique<nav2_util::NodeThread>(trajectory_scorer_node_);
    trajectory_scorer_node_->configure();

    trajectory_picker_node_ = std::make_shared<local_planning::TrajectoryPickerROS>("picker", std::string{get_namespace()}, "trajectory");
    trajectory_picker_thread_ = std::make_unique<nav2_util::NodeThread>(trajectory_picker_node_);
    trajectory_picker_node_->configure();

    costmap_node_ = rclcpp::Node::make_shared("get_costmap_node");
    costmap_client_ = costmap_node_->create_client<nav2_msgs::srv::GetCostmap>("/local_costmap/get_costmap");
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
    trajectory_scorer_node_->activate();
    trajectory_picker_node_->activate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn LocalPlannerManagerNode::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(this->get_logger(), "on_deactivate");
    execute_timer->cancel();
    trajectory_generator_node_->deactivate();
    trajectory_scorer_node_->deactivate();
    trajectory_picker_node_->deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn LocalPlannerManagerNode::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(this->get_logger(), "on_cleanup");
    this->next_waypoint_sub_ = nullptr;
    this->odom_sub_ = nullptr;
    trajectory_generator_node_->cleanup();
    trajectory_scorer_node_->cleanup();
    trajectory_picker_node_->cleanup();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn LocalPlannerManagerNode::on_shutdown(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(this->get_logger(), "on_shutdown");
    trajectory_generator_node_->shutdown();
    trajectory_scorer_node_->shutdown();
    trajectory_picker_node_->shutdown();
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

  void LocalPlannerManagerNode::execute()
  {
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    std::lock_guard<std::mutex> waypoint_lock(waypoint_mutex);

    if (this->didReceiveAllMessages())
    {
      num_execution += 1;
      nav2_msgs::msg::Costmap::SharedPtr curr_costmap = this->p_GetLatestCostmap();
      if (curr_costmap != nullptr)
      {
        this->p_PrintCostMapInfo(curr_costmap);
      }
    }
    num_execution -= 1;

  }

  std::shared_ptr<nav2_msgs::msg::Costmap> LocalPlannerManagerNode::p_GetLatestCostmap()
  {
    if (costmap_client_ == nullptr ||  costmap_node_ == nullptr)
    {
      return nullptr;
    }
    
    auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
    auto result = costmap_client_->async_send_request(request);
    // Wait for the result.
    RCLCPP_INFO(get_logger(), "sending request");

    if (rclcpp::spin_until_future_complete(costmap_node_, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      this->p_PrintCostMapInfo(std::make_shared<nav2_msgs::msg::Costmap>(result.get()->map));
      return std::make_shared<nav2_msgs::msg::Costmap>(result.get()->map);
      // RCLCPP_INFO(get_logger(), "fetched");
    }
    else
    {
      // RCLCPP_INFO(get_logger(), "failed");
      return nullptr;
    }
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
    return this->latest_odom != nullptr && this->latest_waypoint_ != nullptr; 
  }
  void LocalPlannerManagerNode::p_PrintCostMapInfo(const nav2_msgs::msg::Costmap::SharedPtr msg)
  {
    auto map_metadata = msg->metadata;
    RCLCPP_INFO(this->get_logger(), "Map metadata:");
    RCLCPP_INFO(this->get_logger(), "  resolution: %f", map_metadata.resolution);
    RCLCPP_INFO(this->get_logger(), "  size_x: %d", map_metadata.size_x);
    RCLCPP_INFO(this->get_logger(), "  size_y: %d", map_metadata.size_y);
    RCLCPP_INFO(this->get_logger(), "  layer: %s", map_metadata.layer.c_str());
    RCLCPP_INFO(this->get_logger(), "  origin:");
    RCLCPP_INFO(this->get_logger(), "    position:");
    RCLCPP_INFO(this->get_logger(), "      x: %f", map_metadata.origin.position.x);
    RCLCPP_INFO(this->get_logger(), "      y: %f", map_metadata.origin.position.y);
    RCLCPP_INFO(this->get_logger(), "      z: %f", map_metadata.origin.position.z);
    RCLCPP_INFO(this->get_logger(), "    orientation:");
    RCLCPP_INFO(this->get_logger(), "      x: %f", map_metadata.origin.orientation.x);
    RCLCPP_INFO(this->get_logger(), "      y: %f", map_metadata.origin.orientation.y);
    RCLCPP_INFO(this->get_logger(), "      z: %f", map_metadata.origin.orientation.z);
    RCLCPP_INFO(this->get_logger(), "      w: %f", map_metadata.origin.orientation.w);
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