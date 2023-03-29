#include "trajectory_picker/trajectory_picker_node.hpp"

namespace local_planning
{
  TrajectoryPickerNode::TrajectoryPickerNode() : LifecycleNode("trajectory_generator_node", "", true)
  {
  }

  TrajectoryPickerNode ::~TrajectoryPickerNode()
  {
  }

  nav2_util::CallbackReturn TrajectoryPickerNode::on_configure(const rclcpp_lifecycle::State &state)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn TrajectoryPickerNode::on_activate(const rclcpp_lifecycle::State &state)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn TrajectoryPickerNode::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn TrajectoryPickerNode::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn TrajectoryPickerNode::on_shutdown(const rclcpp_lifecycle::State &state)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }
} // local_planning

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<local_planning::TrajectoryPickerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}