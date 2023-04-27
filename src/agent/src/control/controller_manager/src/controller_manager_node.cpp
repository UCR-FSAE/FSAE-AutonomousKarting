#include "controller_manager/controller_manager_node.hpp"
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
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<controller::ControllerManagerNode>();
  
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}