#include "local_planner_manager/local_planner_manager_node.hpp"
#include "trajectory_generator/dummy_trajectory_generator.hpp"

namespace local_planning
{
  LocalPlannerManagerNode::LocalPlannerManagerNode() : LifecycleNode("manager", "local_planner", true)
  {
    this->declare_parameter("manager_rate", 0.5);
    this->declare_parameter("min_dist", 5.0);
    this->declare_parameter("debug", false);

    if (this->get_parameter("debug").as_bool())
    {
      auto ret = rcutils_logging_set_logger_level(get_logger().get_name(),
                                                  RCUTILS_LOG_SEVERITY_DEBUG); // enable or disable debug
    }
    this->declare_parameter("controller_route", "/roar/controller_manager");
    this->controllerServerRoute = this->get_parameter("controller_route").as_string();

    RCLCPP_INFO(this->get_logger(), "LocalPlannerManagerNode initialized with Debug Mode = [%s]",
                this->get_parameter("debug").as_bool() ? "YES" : "NO");
    RCLCPP_INFO(this->get_logger(), "controller route: [%s]", this->controllerServerRoute.c_str());
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

    this->next_waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        std::string{get_namespace()} + "/next_waypoint", 10);

    this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry", rclcpp::SystemDefaultsQoS(),
        std::bind(&LocalPlannerManagerNode::onLatestOdomReceived, this, std::placeholders::_1));

    this->footprint_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
        "/footprint", rclcpp::SystemDefaultsQoS(),
        std::bind(&LocalPlannerManagerNode::onLatestFootprintReceived, this, std::placeholders::_1));

    this->global_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/global_path", rclcpp::SystemDefaultsQoS(),
        std::bind(&LocalPlannerManagerNode::onLatestGlobalPlanReceived, this, std::placeholders::_1));

    trajectory_generator_node_ = std::make_shared<local_planning::TrajectoryGeneratorROS>(
        "generator", std::string{get_namespace()}, "trajectory", this->get_parameter("debug").as_bool());
    trajectory_generator_thread_ = std::make_unique<nav2_util::NodeThread>(trajectory_generator_node_);
    this->register_generators();
    trajectory_generator_node_->configure();

    trajectory_picker_node_ = std::make_shared<local_planning::TrajectoryPickerROS>(
        "picker", std::string{get_namespace()}, "trajectory", this->get_parameter("debug").as_bool());
    trajectory_picker_thread_ = std::make_unique<nav2_util::NodeThread>(trajectory_picker_node_);
    trajectory_picker_node_->configure();

    this->possible_trajectory_publisher_ = this->create_publisher<planning_interfaces::msg::Trajectory>(
        std::string{get_namespace()} + "/trajectory/picker" + "/possible_trajectory", 10);

    costmap_node_ = rclcpp::Node::make_shared("get_costmap_node");
    costmap_client_ = costmap_node_->create_client<nav2_msgs::srv::GetCostmap>("/local_costmap/get_costmap");

    this->control_action_client_ = rclcpp_action::create_client<ControlAction>(this, this->controllerServerRoute);

    // diagnostic server
    diagnostic_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10);

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

    this->trajectory_generator_client =
        rclcpp_action::create_client<TrajectoryGeneration>(this, "trajectory/trajectory_generation");

    this->best_trajectory_subscriber_ = this->create_subscription<planning_interfaces::msg::Trajectory>(
        std::string(this->get_namespace()) + "/trajectory/picker" + "/best_trajectory", 10,
        std::bind(&LocalPlannerManagerNode::on_best_trajectory_publication_received, this, std::placeholders::_1));
    this->possible_trajectory_publisher_->on_activate();
    this->diagnostic_pub_->on_activate();

    this->next_waypoint_publisher_->on_activate();

    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn LocalPlannerManagerNode::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_DEBUG(this->get_logger(), "on_deactivate");
    execute_timer->cancel();
    trajectory_generator_node_->deactivate();
    trajectory_picker_node_->deactivate();
    diagnostic_pub_->on_deactivate();
    this->next_waypoint_publisher_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn LocalPlannerManagerNode::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_DEBUG(this->get_logger(), "on_cleanup");
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
  void LocalPlannerManagerNode::execute()
  {
    this->p_execute();
  }

  void LocalPlannerManagerNode::p_execute()
  {
    RCLCPP_DEBUG(this->get_logger(), "-----LocalPlannerManagerNode-----");
    if (this->canExecute()) // only one request at a time
    {
      geometry_msgs::msg::PoseStamped::SharedPtr next_waypoint = this->findNextWaypoint(float(this->get_parameter("min_dist").as_double()));
      if (next_waypoint == nullptr)
      {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "next_waypoint is null, not executing...");
        return;
      }
      RCLCPP_DEBUG_STREAM(this->get_logger(), "next waypoint: x:" << next_waypoint->pose.position.x << " y: " << next_waypoint->pose.position.y << " z: " << next_waypoint->pose.position.z);

      this->next_waypoint_publisher_->publish(*next_waypoint);

      // construct goal
      planning_interfaces::action::TrajectoryGeneration_Goal goal_msg = TrajectoryGeneration::Goal();
      // goal_msg.costmap = *this->latest_costmap_;
      goal_msg.odom = *this->latest_odom;
      goal_msg.footprint = *this->latest_footprint_;
      goal_msg.next_waypoint = *next_waypoint;
      goal_msg.global_path = *this->global_plan_;
      RCLCPP_DEBUG(get_logger(), "goal constructed");

      // send goal
      auto send_goal_options = rclcpp_action::Client<TrajectoryGeneration>::SendGoalOptions();
      send_goal_options.goal_response_callback =
          std::bind(&LocalPlannerManagerNode::trajectory_generator_goal_response_callback, this, std::placeholders::_1);
      send_goal_options.feedback_callback =
          std::bind(&LocalPlannerManagerNode::trajectory_generator_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
      send_goal_options.result_callback =
          std::bind(&LocalPlannerManagerNode::trajectory_generator_result_callback, this, std::placeholders::_1);
      this->trajectory_generator_client->async_send_goal(goal_msg, send_goal_options);
      RCLCPP_DEBUG(get_logger(), "goal sent");

      num_execution += 1;
      num_generator_execution += 1;
    }
  }

  std::shared_ptr<nav2_msgs::msg::Costmap> LocalPlannerManagerNode::p_GetLatestCostmap()
  {
    if (costmap_client_ == nullptr || costmap_node_ == nullptr)
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
    this->trajectory_generator_node_->registerTrajectoryGenerator(
        std::make_shared<local_planning::DummyTrajectoryGenerator>());
    // this->trajectory_generator_node_->registerTrajectoryGenerator(std::make_shared<local_planning::AStar>());
  }
  void LocalPlannerManagerNode::send_trajectory_generator_action(
      const nav2_msgs::msg::Costmap::SharedPtr costmap, const nav_msgs::msg::Odometry::SharedPtr odom,
      const geometry_msgs::msg::PoseStamped::SharedPtr next_waypoint,
      const geometry_msgs::msg::PolygonStamped::SharedPtr footprint)
  {
    using namespace std::placeholders;
    RCLCPP_DEBUG(get_logger(), "sending goal");

    planning_interfaces::action::TrajectoryGeneration_Goal goal_msg = TrajectoryGeneration::Goal();
    // goal_msg.costmap = *costmap;
    goal_msg.next_waypoint = *next_waypoint;
    // goal_msg.odom = *odom;
    // goal_msg.footprint = *footprint;
  }

  void LocalPlannerManagerNode::trajectory_generator_goal_response_callback(
      std::shared_future<GoalHandleTrajectoryGeneration::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "[trajectory_generator_goal_response_"
                   "callback] was rejected by server");
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(),
                   "[trajectory_generator_goal_response_callback] accepted "
                   "by server, waiting for result");
    }
  }

  void LocalPlannerManagerNode::trajectory_generator_feedback_callback(
      GoalHandleTrajectoryGeneration::SharedPtr goal_handle,
      const std::shared_ptr<const TrajectoryGeneration::Feedback> feedback)
  {
    RCLCPP_DEBUG(get_logger(), "Feedback contains trajectory of length [%d] and raw_score: [%.3f]",
                 feedback->trajectory.trajectory.poses.size(), feedback->trajectory.score.raw_score);
    this->possible_trajectory_publisher_->publish(feedback.get()->trajectory);
  }
  void LocalPlannerManagerNode::trajectory_generator_result_callback(
      const GoalHandleTrajectoryGeneration::WrappedResult &result)
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
  void LocalPlannerManagerNode::on_best_trajectory_publication_received(
      const planning_interfaces::msg::Trajectory::SharedPtr msg)
  {
    RCLCPP_DEBUG(get_logger(), "on_best_trajectory_publication_received");
    RCLCPP_DEBUG(get_logger(), "cancelling generator goals");
    this->trajectory_generator_client->async_cancel_all_goals();

    this->control_send_goal(std::make_shared<nav_msgs::msg::Path>(msg->trajectory), 10.0);
  }

  /**
   * Controller interaction
   */
  void LocalPlannerManagerNode::control_send_goal(const nav_msgs::msg::Path::SharedPtr path, float target_spd)
  {
    RCLCPP_DEBUG(get_logger(), "sending goal to controller at [%s]", this->controllerServerRoute.c_str());

    using namespace std::placeholders;

    if (!this->control_action_client_->wait_for_action_server())
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Control action server not available, "
                   "not sending goal to controller");
      return;
    }
    auto goal_msg = ControlAction::Goal();
    goal_msg.path = *path;
    goal_msg.target_spd = target_spd;
    goal_msg.overwrite_previous = true;

    // RCLCPP_INFO(this->get_logger(), "Executing local path - len(path)=[%d] |
    // target_spd=[%f]",path->poses.size() ,target_spd);

    auto send_goal_options = rclcpp_action::Client<ControlAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&LocalPlannerManagerNode::control_action_goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&LocalPlannerManagerNode::control_action_feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&LocalPlannerManagerNode::control_action_result_callback, this, _1);
    this->control_action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void LocalPlannerManagerNode::control_action_goal_response_callback(
      std::shared_future<GoalHandleControlAction::SharedPtr> future)
  {
    auto goal_handle = future.get();
    auto diag_array_msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
    diag_array_msg->header.stamp = this->now();
    diagnostic_msgs::msg::DiagnosticStatus::SharedPtr diag_status_msg = std::make_shared<diagnostic_msgs::msg::DiagnosticStatus>();
    diag_status_msg->name = std::string(this->get_namespace()) + "/" +
                            std::string(this->get_name());

    if (!goal_handle)
    {
      diag_status_msg->level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diag_status_msg->message = "Goal was rejected by server";
      diag_array_msg->status.push_back(*diag_status_msg);
      diagnostic_pub_->publish(std::move(diag_array_msg));
    }
    else
    {
      diag_status_msg->level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      diag_status_msg->message = "Goal accepted by server, waiting for result";
      diag_array_msg->status.push_back(*diag_status_msg);
      diagnostic_pub_->publish(std::move(diag_array_msg));
    }
  }
  void LocalPlannerManagerNode::control_action_feedback_callback(
      GoalHandleControlAction::SharedPtr future, const std::shared_ptr<const ControlAction::Feedback> feedback)
  {
    // Left empty on purpose
    RCLCPP_DEBUG(get_logger(), "Currently at waypoint index: [{%d}]", feedback->curr_index);
  }
  void LocalPlannerManagerNode::control_action_result_callback(const GoalHandleControlAction::WrappedResult &result)
  {
    this->num_execution -= 1;

    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(this->get_logger(), "control_action result success received");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_DEBUG(this->get_logger(), "control_action goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(this->get_logger(), "control_action goal was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "control_action unknown result code");
      break;
    }
  }
  /**
   * helper methods
   */
  bool LocalPlannerManagerNode::canExecute()
  {
    if (this->didReceiveAllMessages() == false)
    {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "not executing: "
                                                  << "didReceiveAllMessages() == false");
      return false;
    }

    if (num_generator_execution >= 1)
    {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "not executing: "
                                                  << "num_generator_execution >= 1");
      return false;
    }
    return true;
  }
  bool LocalPlannerManagerNode::didReceiveAllMessages()
  {
    if (this->latest_odom == nullptr)
    {
      RCLCPP_DEBUG(this->get_logger(), "odom not received, not executing...");
    }
    if (this->latest_footprint_ == nullptr)
    {
      RCLCPP_DEBUG(this->get_logger(), "latest_footprint_ not received, not executing...");
    }
    if (this->global_plan_ == nullptr)
    {
      RCLCPP_DEBUG(this->get_logger(), "global_plan_ not received, not executing...");
    }
    return this->latest_odom != nullptr && this->latest_footprint_ != nullptr && this->global_plan_ != nullptr;
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

  geometry_msgs::msg::PoseStamped::SharedPtr LocalPlannerManagerNode::findNextWaypoint(const float next_waypoint_min_dist)
  {
    if (this->global_plan_ == nullptr)
    {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "global_plan_ is null, not finding waypoint");
      return nullptr;
    }

    RCLCPP_DEBUG(this->get_logger(), "finding waypoint");

    // find closest waypoint
    // find the closest waypoint to the current position
    double min_distance = std::numeric_limits<double>::max();
    size_t closest_waypoint_index = 0;
    for (size_t i = 0; i < global_plan_->poses.size(); i++)
    {
      double distance = std::sqrt(std::pow(this->latest_odom->pose.pose.position.x - global_plan_->poses[i].pose.position.x, 2) +
                                  std::pow(this->latest_odom->pose.pose.position.y - global_plan_->poses[i].pose.position.y, 2) +
                                  std::pow(this->latest_odom->pose.pose.position.z - global_plan_->poses[i].pose.position.z, 2));
      if (distance < min_distance)
      {
        min_distance = distance;
        closest_waypoint_index = i;
      }
    }
    RCLCPP_DEBUG_STREAM(this->get_logger(), "closest waypoint index: " << closest_waypoint_index << ", distance: " << min_distance);

    // find the next waypoint, including looping back to the beginning
    // double next_waypoint_dist = cte_and_lookahead.second;
    double next_waypoint_dist = next_waypoint_min_dist;
    size_t next_waypoint_index = closest_waypoint_index;
    for (size_t i = 0; i < global_plan_->poses.size(); i++)
    {
      size_t next_index = (closest_waypoint_index + i) % global_plan_->poses.size();
      double distance = std::sqrt(std::pow(this->latest_odom->pose.pose.position.x - global_plan_->poses[next_index].pose.position.x, 2) +
                                  std::pow(this->latest_odom->pose.pose.position.y - global_plan_->poses[next_index].pose.position.y, 2) +
                                  std::pow(this->latest_odom->pose.pose.position.z - global_plan_->poses[next_index].pose.position.z, 2));
      if (distance > next_waypoint_dist)
      {
        next_waypoint_index = next_index;
        break;
      }
    }
    RCLCPP_DEBUG_STREAM(this->get_logger(), "next waypoint index: " << next_waypoint_index << ", next_waypoint_dist: " << next_waypoint_dist);
    geometry_msgs::msg::PoseStamped next_waypoint = this->global_plan_->poses[next_waypoint_index];

    return std::make_shared<geometry_msgs::msg::PoseStamped>(next_waypoint);
  }

} // namespace local_planning

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<local_planning::LocalPlannerManagerNode>();

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}