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
