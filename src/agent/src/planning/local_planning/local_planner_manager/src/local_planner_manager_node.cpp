#include "local_planner_manager/local_planner_manager_node.hpp"
#include "trajectory_generator/dummy_trajectory_generator.hpp"

namespace local_planning
{
  LocalPlannerManagerNode::LocalPlannerManagerNode() : LifecycleNode("manager", "local_planner", true)
  {
    this->declare_parameter("manager_rate", 0.5);

    this->declare_parameter("debug", false);
    
    if (this->get_parameter("debug").as_bool())
    {
        auto ret = rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG); // enable or disable debug
    }

    RCLCPP_INFO(this->get_logger(), "LocalPlannerManagerNode initialized with Debug Mode = [%s]", this->get_parameter("debug").as_bool() ? "YES" : "NO");
  }
  LocalPlannerManagerNode ::~LocalPlannerManagerNode()
  {
  }

  /**
   * lifecycle
  */
  nav2_util::CallbackReturn LocalPlannerManagerNode::on_configure(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_DEBUG(this->get_logger(), "on_configure");

    this->next_waypoint_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/next_waypoint", rclcpp::SystemDefaultsQoS(),
        std::bind(&LocalPlannerManagerNode::onLatestWaypointReceived, this, std::placeholders::_1));
    this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/carla/ego_vehicle/odometry", rclcpp::SystemDefaultsQoS(),
        std::bind(&LocalPlannerManagerNode::onLatestOdomReceived, this, std::placeholders::_1));
   

    trajectory_generator_node_ = std::make_shared<local_planning::TrajectoryGeneratorROS>("generator", std::string{get_namespace()}, "trajectory");
    trajectory_generator_node_->set_parameter(rclcpp::Parameter("debug", this->get_parameter("debug").as_bool()));
    trajectory_generator_thread_ = std::make_unique<nav2_util::NodeThread>(trajectory_generator_node_);
    this->register_generators();
    trajectory_generator_node_->configure();

    trajectory_picker_node_ = std::make_shared<local_planning::TrajectoryPickerROS>("picker", std::string{get_namespace()}, "trajectory");
    trajectory_picker_thread_ = std::make_unique<nav2_util::NodeThread>(trajectory_picker_node_);
    trajectory_picker_node_->configure();

    this->possible_trajectory_publisher_ = this->create_publisher<planning_interfaces::msg::Trajectory>(
      std::string{get_namespace()} + "/trajectory/picker" + "/possible_trajectory", 10);

    costmap_node_ = rclcpp::Node::make_shared("get_costmap_node");
    costmap_client_ = costmap_node_->create_client<nav2_msgs::srv::GetCostmap>("/local_costmap/get_costmap");


    this->control_action_client_ = rclcpp_action::create_client<ControlAction>(
        this,
        "/controller/manager");
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn LocalPlannerManagerNode::on_activate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_DEBUG(this->get_logger(), "on_activate");
    double loop_rate = this->get_parameter("manager_rate").as_double();
    RCLCPP_DEBUG(this->get_logger(), "loop_rate: %.3f", loop_rate);
    execute_timer = create_wall_timer(std::chrono::milliseconds(int(loop_rate * 1000)),
                                      std::bind(&LocalPlannerManagerNode::execute, this));
    trajectory_generator_node_->activate();
    trajectory_picker_node_->activate();

    this->trajectory_generator_client = rclcpp_action::create_client<TrajectoryGeneration>(this,"trajectory/trajectory_generation");

    this->best_trajectory_subscriber_ = this->create_subscription<planning_interfaces::msg::Trajectory>(
          std::string(this->get_namespace())  + "/trajectory/picker"+ "/best_trajectory", 
            10,
            std::bind(&LocalPlannerManagerNode::on_best_trajectory_publication_received, this, std::placeholders::_1));
    this->possible_trajectory_publisher_->on_activate();

    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn LocalPlannerManagerNode::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_DEBUG(this->get_logger(), "on_deactivate");
    execute_timer->cancel();
    trajectory_generator_node_->deactivate();
    trajectory_picker_node_->deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn LocalPlannerManagerNode::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_DEBUG(this->get_logger(), "on_cleanup");
    this->next_waypoint_sub_ = nullptr;
    this->odom_sub_ = nullptr;
    trajectory_generator_node_->cleanup();
    trajectory_picker_node_->cleanup();

    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn LocalPlannerManagerNode::on_shutdown(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_DEBUG(this->get_logger(), "on_shutdown");
    trajectory_generator_node_->shutdown();
    trajectory_picker_node_->shutdown();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  void LocalPlannerManagerNode::onLatestWaypointReceived(geometry_msgs::msg::Pose::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(waypoint_mutex);
    // TODO: change global waypoint to be PoseStamped
    geometry_msgs::msg::PoseStamped ps;
    ps.pose = *msg;
    std_msgs::msg::Header header; 
    header.frame_id = "map";
    header.stamp = this->get_clock()->now();
    ps.header = header; 
    this->latest_waypoint_ = std::make_shared<geometry_msgs::msg::PoseStamped>(ps);
  }
  void LocalPlannerManagerNode::onLatestOdomReceived(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    this->latest_odom = msg;
  }
  
  void LocalPlannerManagerNode::execute()
  {
    this->p_execute();
  }

  void LocalPlannerManagerNode::p_execute()
  {
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    std::lock_guard<std::mutex> waypoint_lock(waypoint_mutex);

    if (this->canExecute()) // only one request at a time
    {
      num_execution += 1;
      num_generator_execution += 1;
      latest_costmap_ = this->p_GetLatestCostmap();
      if (latest_costmap_ != nullptr)
      {
        this->send_trajectory_generator_action(latest_costmap_, this->latest_odom, this->latest_waypoint_);
      }
    }
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

    if (rclcpp::spin_until_future_complete(costmap_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      // TODO: error on timeout
      // this->p_PrintCostMapInfo(std::make_shared<nav2_msgs::msg::Costmap>(result.get()->map));
      return std::make_shared<nav2_msgs::msg::Costmap>(result.get()->map);
    }
    else
    {
      return nullptr;
    }
  }

  /**
   * trajectory generation
  */
  void LocalPlannerManagerNode::register_generators()
  {
    std::shared_ptr<local_planning::DummyTrajectoryGenerator> dummy_generator = std::make_shared<local_planning::DummyTrajectoryGenerator>();
    this->trajectory_generator_node_->registerTrajectoryGenerator(dummy_generator);
  }
  void LocalPlannerManagerNode::send_trajectory_generator_action(
      const nav2_msgs::msg::Costmap::SharedPtr costmap,
      const nav_msgs::msg::Odometry::SharedPtr odom,
      const geometry_msgs::msg::PoseStamped::SharedPtr next_waypoint)
  {
    using namespace std::placeholders;
    RCLCPP_DEBUG(get_logger(), "sending goal");

    planning_interfaces::action::TrajectoryGeneration_Goal goal_msg = TrajectoryGeneration::Goal();
    goal_msg.costmap = *costmap;
    goal_msg.next_waypoint = *next_waypoint;
    goal_msg.odom = *odom;
    
    auto send_goal_options = rclcpp_action::Client<TrajectoryGeneration>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&LocalPlannerManagerNode::trajectory_generator_goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&LocalPlannerManagerNode::trajectory_generator_feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&LocalPlannerManagerNode::trajectory_generator_result_callback, this, _1);
    this->trajectory_generator_client->async_send_goal(goal_msg, send_goal_options);
  }
  
  void LocalPlannerManagerNode::trajectory_generator_goal_response_callback(std::shared_future<GoalHandleTrajectoryGeneration::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "[trajectory_generator_goal_response_callback] was rejected by server");
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "[trajectory_generator_goal_response_callback] accepted by server, waiting for result");
    }  
  }

  void LocalPlannerManagerNode::trajectory_generator_feedback_callback(
      GoalHandleTrajectoryGeneration::SharedPtr goal_handle,
      const std::shared_ptr<const TrajectoryGeneration::Feedback> feedback)
  {
    RCLCPP_DEBUG(get_logger(), "Feedback contains trajectory of length [%d] and raw_score: [%.3f]", feedback->trajectory.trajectory.poses.size(), feedback->trajectory.score.raw_score);
    this->possible_trajectory_publisher_->publish(feedback.get()->trajectory);
  }
  void LocalPlannerManagerNode::trajectory_generator_result_callback(const GoalHandleTrajectoryGeneration::WrappedResult &result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_DEBUG(this->get_logger(), "trajectory_generator result received");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_DEBUG(this->get_logger(), "trajectory_generator goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_DEBUG(this->get_logger(), "trajectory_generator goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "trajectory_generator unknown result code");
        break;
    }
    this->num_generator_execution -= 1;
  }
  void LocalPlannerManagerNode::on_best_trajectory_publication_received(const planning_interfaces::msg::Trajectory::SharedPtr msg)
  {
    RCLCPP_DEBUG(get_logger(), "on_best_trajectory_publication_received");
    // TODO: terminate the curren trajectory computation immediately because best trajectory is already published
    RCLCPP_DEBUG(get_logger(), "cancelling generator goals");
    this->trajectory_generator_client->async_cancel_all_goals();

    this->control_send_goal(std::make_shared<nav_msgs::msg::Path>(msg->trajectory), 10.0);
  }

  /**
  * Controller interaction
  */
  void
  LocalPlannerManagerNode::control_send_goal(const nav_msgs::msg::Path::SharedPtr path, float target_spd)
  {
    RCLCPP_DEBUG(get_logger(), "sending goal to controller");

    using namespace std::placeholders;

    if (!this->control_action_client_->wait_for_action_server())
    {
      RCLCPP_ERROR(this->get_logger(), "Control action server not available, not sending goal");
      return;
    }
    auto goal_msg = ControlAction::Goal();
    goal_msg.path = *path;
    goal_msg.target_spd = target_spd;

    RCLCPP_DEBUG(this->get_logger(), "Sending goal: target_spd: %f", target_spd);

    auto send_goal_options = rclcpp_action::Client<ControlAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&LocalPlannerManagerNode::control_action_goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&LocalPlannerManagerNode::control_action_feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&LocalPlannerManagerNode::control_action_result_callback, this, _1);
    this->control_action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void LocalPlannerManagerNode::control_action_goal_response_callback(std::shared_future<GoalHandleControlAction::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "[control_action_goal_response_callback] was rejected by server");
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "[control_action_goal_response_callback] accepted by server, waiting for result");
    }  
  }
  void LocalPlannerManagerNode::control_action_feedback_callback(GoalHandleControlAction::SharedPtr future, const std::shared_ptr<const ControlAction::Feedback> feedback)
  {
    // Left empty on purpose
  }
  void LocalPlannerManagerNode::control_action_result_callback(const GoalHandleControlAction::WrappedResult &result)
  {
    this->num_execution -= 1;
  }
  /**
   * helper methods
  */
  bool LocalPlannerManagerNode::canExecute()
  {
    if (this->didReceiveAllMessages() && num_generator_execution < 1)
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