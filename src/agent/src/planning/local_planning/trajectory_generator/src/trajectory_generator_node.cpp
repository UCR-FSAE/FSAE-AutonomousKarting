#include "trajectory_generator/trajectory_generator_node.hpp"

namespace local_planning
{
  TrajectoryGeneratorNode::TrajectoryGeneratorNode() : LifecycleNode("trajectory_generator_node", "", true)
  {

  }

  TrajectoryGeneratorNode ::~TrajectoryGeneratorNode()
  {

  }

  nav2_util::CallbackReturn TrajectoryGeneratorNode::on_configure(const rclcpp_lifecycle::State &state)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn TrajectoryGeneratorNode::on_activate(const rclcpp_lifecycle::State &state)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn TrajectoryGeneratorNode::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn TrajectoryGeneratorNode::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn TrajectoryGeneratorNode::on_shutdown(const rclcpp_lifecycle::State &state)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }
} // local_planning

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<local_planning::TrajectoryGeneratorNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}