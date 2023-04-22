#include "trajectory_scorer/trajectory_scorer_ros.hpp"


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<local_planning::TrajectoryScorerROS>("trajectory_scorer");
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}