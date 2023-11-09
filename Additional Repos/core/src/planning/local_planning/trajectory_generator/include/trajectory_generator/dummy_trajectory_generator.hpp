#ifndef DUMMY_TRAJECTORY_GENERATOR_HPP
#define DUMMY_TRAJECTORY_GENERATOR_HPP
#include "trajectory_generator/trajectory_generator_interface.hpp"
#include <memory>
namespace local_planning {
class DummyTrajectoryGenerator : public TrajectoryGeneratorInterface {
  public:
    DummyTrajectoryGenerator() { this->name = "DummyTrajectoryGenerator"; }
    nav_msgs::msg::Path computeTrajectory(
        const std::shared_ptr<
            const planning_interfaces::action::TrajectoryGeneration_Goal>
            request) override {
        nav_msgs::msg::Path path;
        // dummy trajectory generator just directly output path to
        // next_waypoint
        path.poses.push_back(request->next_waypoint);
        return path;
    }

    void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
                   std::shared_ptr<tf2_ros::Buffer> tf) override {
        RCLCPP_INFO(rclcpp::get_logger(this->name), "configuring");
    }

    void cleanup() override {}

    void activate() override {}

    void deactivate() override {}

  private:
};
} // namespace local_planning
#endif // DUMMY_TRAJECTORY_GENERATOR_HPP
