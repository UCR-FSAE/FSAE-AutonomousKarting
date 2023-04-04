#include "trajectory_generator/trajectory_generator_ros.hpp"


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<local_planning::TrajectoryGeneratorROS>("trajectory_generator");
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}