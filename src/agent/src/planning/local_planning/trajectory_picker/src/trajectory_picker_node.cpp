#include "trajectory_picker/trajectory_picker_ros.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<local_planning::TrajectoryPickerROS>("trajectory_picker");
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}