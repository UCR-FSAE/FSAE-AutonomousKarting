#include <controller_manager/controller_manager_ros.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace controller
{
  ControllerManagerNode::ControllerManagerNode() : LifecycleNode("manager", "controller", true)
  {
    this->declare_parameter("debug", false);
    
    if (this->get_parameter("debug").as_bool())
    {
        auto ret = rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG); // enable or disable debug
    }

    RCLCPP_INFO(this->get_logger(), "ControllerManagerNode initialized with Debug Mode = [%s]", this->get_parameter("debug").as_bool() ? "YES" : "NO");
  }
  ControllerManagerNode ::~ControllerManagerNode()
  {

  }


  /**
   * Lifecycles
  */

  nav2_util::CallbackReturn ControllerManagerNode::on_configure(const rclcpp_lifecycle::State &state) 
  {
    RCLCPP_DEBUG(get_logger(), "on_configure");
    this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/carla/ego_vehicle/odometry", rclcpp::SystemDefaultsQoS(),
        std::bind(&ControllerManagerNode::onLatestOdomReceived, this, std::placeholders::_1));

    this->action_server_ = rclcpp_action::create_server<ControlAction>(
      this,
      "fibonacci",
      std::bind(&ControllerManagerNode::handle_goal, this, std::placeholders::_1,std::placeholders:: _2),
      std::bind(&ControllerManagerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&ControllerManagerNode::handle_accepted, this, std::placeholders::_1));
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn ControllerManagerNode::on_activate(const rclcpp_lifecycle::State &state) 
  {
    RCLCPP_DEBUG(get_logger(), "on_activate");
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn ControllerManagerNode::on_deactivate(const rclcpp_lifecycle::State &state) 
  {
    RCLCPP_DEBUG(get_logger(), "on_deactivate");
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn ControllerManagerNode::on_cleanup(const rclcpp_lifecycle::State &state) 
  {
    RCLCPP_DEBUG(get_logger(), "on_cleanup");
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn ControllerManagerNode::on_shutdown(const rclcpp_lifecycle::State &state) 
  {
    RCLCPP_DEBUG(get_logger(), "on_shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
  }
  /**
   * subscribers 
  */

  void ControllerManagerNode::onLatestOdomReceived(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    this->latest_odom = msg;
  }


  /**
   * Action server
  */
  rclcpp_action::GoalResponse ControllerManagerNode::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ControlAction::Goal> goal)
  {
    RCLCPP_DEBUG(get_logger(), "handle_goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse ControllerManagerNode::handle_cancel(const std::shared_ptr<GoalHandleControlAction> goal_handle)
  {
    RCLCPP_DEBUG(get_logger(), "handle_cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void ControllerManagerNode::handle_accepted(const std::shared_ptr<GoalHandleControlAction> goal_handle)
  {
    RCLCPP_DEBUG(get_logger(), "handle_accepted");
  }

  /**
   * main execution loops
  */
  void ControllerManagerNode::p_execute()
  {

  }
  bool ControllerManagerNode::canExecute()
  {
    return true;
  }

}
