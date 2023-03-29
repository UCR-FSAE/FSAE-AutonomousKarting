#include "trajectory_scorer/trajectory_scorer_node.hpp"

namespace local_planning
{
  TrajectoryScorerNode::TrajectoryScorerNode() : LifecycleNode("trajectory_generator_node", "", true)
  {
  }

  TrajectoryScorerNode ::~TrajectoryScorerNode()
  {
  }

  nav2_util::CallbackReturn TrajectoryScorerNode::on_configure(const rclcpp_lifecycle::State &state)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn TrajectoryScorerNode::on_activate(const rclcpp_lifecycle::State &state)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn TrajectoryScorerNode::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn TrajectoryScorerNode::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn TrajectoryScorerNode::on_shutdown(const rclcpp_lifecycle::State &state)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }
} // local_planning

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<local_planning::TrajectoryScorerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}