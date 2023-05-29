#ifndef A_STAR_HPP_
#define A_STAR_HPP_
#include "trajectory_generator/trajectory_generator_interface.hpp"
#include <memory>

/* start of migrated imports from smac planner */
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "smac_planner/a_star.hpp"
#include "smac_planner/costmap_downsampler.hpp"
#include "smac_planner/smoother.hpp"
#include "tf2/utils.h"
/* end of migrated imports from smac planner */
#include <smac_planner/constants.hpp>
#include <smac_planner/types.hpp>

#define DEBUG false
#define BENCHMARK_TESTING

namespace local_planning {
class AStar : public TrajectoryGeneratorInterface {
  public:
    AStar() {
        this->name = "A_star_algo";
        if (DEBUG) {
            auto ret = rcutils_logging_set_logger_level(
                _logger.get_name(),
                RCUTILS_LOG_SEVERITY_DEBUG); // enable or disable debug
        }
    }

    void p_bridgeSMACConfigure();

    nav_msgs::msg::Path computeTrajectory(
        const std::shared_ptr<
            const planning_interfaces::action::TrajectoryGeneration_Goal>
            request) override;

    void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
                   std::shared_ptr<tf2_ros::Buffer> tf) override;

    void cleanup() override;

    void activate() override;

    void deactivate();

  private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;

  protected:
    /* start migration from smac planner */
    std::unique_ptr<smac_planner::AStarAlgorithm<smac_planner::NodeSE2>>
        _a_star;
    std::unique_ptr<smac_planner::Smoother> _smoother;
    rclcpp::Clock::SharedPtr _clock;
    rclcpp::Logger _logger{rclcpp::get_logger("A_star_algo")};
    // TODO: add this back in
    // std::unique_ptr<smac_planner::CostmapDownsampler> _costmap_downsampler;

    std::string _ego_frame, _name, _global_frame;
    float _tolerance;
    unsigned int _angle_quantizations;
    double _angle_bin_size;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
        _raw_plan_publisher;
    smac_planner::SmootherParams _smoother_params;
    smac_planner::OptimizerParams _optimizer_params;
    double _max_planning_time;
    /* end migration from smac planner*/

    smac_planner::SearchInfo search_info;
    smac_planner::MotionModel motion_model;
    bool allow_unknown;
    int max_iterations;
    int max_on_approach_iterations = std::numeric_limits<int>::max();
    int angle_quantizations;

    bool smooth_path;
    std::string motion_model_for_search;

    void fillCostmapFromMsg(nav2_costmap_2d::Costmap2D *costmap,
                            const nav2_msgs::msg::Costmap *msg) {
        if (costmap == nullptr || msg == nullptr) {
            throw std::runtime_error("undefined costmap or costmap msg");
        }
        const std::vector<uint8_t> &data = msg->data;
        unsigned int width = msg->metadata.size_x;
        unsigned int height = msg->metadata.size_y;

        costmap->resizeMap(width, height, msg->metadata.resolution,
                           msg->metadata.origin.position.x,
                           msg->metadata.origin.position.y);

        for (unsigned int y = 0; y < height; ++y) {
            for (unsigned int x = 0; x < width; ++x) {
                unsigned char cost = data[y * width + x];
                costmap->setCost(x, y, cost);
            }
        }
    }

    Eigen::Vector2d getWorldCoords(const float &mx, const float &my,
                                   const nav2_costmap_2d::Costmap2D *costmap) {
        // mx, my are in continuous grid coordinates, must convert to world
        // coordinates
        double world_x = static_cast<double>(costmap->getOriginX()) +
                         (mx + 0.5) * costmap->getResolution();
        double world_y = static_cast<double>(costmap->getOriginY()) +
                         (my + 0.5) * costmap->getResolution();
        return Eigen::Vector2d(world_x, world_y);
    }

    geometry_msgs::msg::Quaternion getWorldOrientation(const float &theta) {
        // theta is in continuous bin coordinates, must convert to world
        // orientation
        tf2::Quaternion q;
        q.setEuler(0.0, 0.0, theta * static_cast<double>(_angle_bin_size));
        return tf2::toMsg(q);
    }

    void removeHook(std::vector<Eigen::Vector2d> &path) {
        // Removes the end "hooking" since goal is locked in place
        Eigen::Vector2d interpolated_second_to_last_point;
        interpolated_second_to_last_point =
            (path.end()[-3] + path.end()[-1]) / 2.0;
        if (smac_planner::squaredDistance(path.end()[-2], path.end()[-1]) >
            smac_planner::squaredDistance(interpolated_second_to_last_point,
                                          path.end()[-1])) {
            path.end()[-2] = interpolated_second_to_last_point;
        }
    }

    void p_debugSearchInfo(smac_planner::SearchInfo searchInfo) {
        RCLCPP_INFO(_logger, "SearchInfo values:");
        RCLCPP_INFO(_logger, "  Minimum Turning Radius: %.2f",
                    searchInfo.minimum_turning_radius);
        RCLCPP_INFO(_logger, "  Non-straight Penalty: %.2f",
                    searchInfo.non_straight_penalty);
        RCLCPP_INFO(_logger, "  Change Penalty: %.2f",
                    searchInfo.change_penalty);
        RCLCPP_INFO(_logger, "  Reverse Penalty: %.2f",
                    searchInfo.reverse_penalty);
        RCLCPP_INFO(_logger, "  Cost Penalty: %.2f", searchInfo.cost_penalty);
        RCLCPP_INFO(_logger, "  Analytic Expansion Ratio: %.2f",
                    searchInfo.analytic_expansion_ratio);
    }
    void p_debugCostMapMsg(const nav2_msgs::msg::Costmap *msg) {
        RCLCPP_DEBUG(_logger, "Received Costmap message");
        RCLCPP_DEBUG(_logger, "  Width: %d", msg->metadata.size_x);
        RCLCPP_DEBUG(_logger, "  Height: %d", msg->metadata.size_y);
        RCLCPP_DEBUG(_logger, "  Resolution: %.3f", msg->metadata.resolution);
        RCLCPP_DEBUG(_logger, "  origin_x: %.3f",
                     msg->metadata.origin.position.x);
        RCLCPP_DEBUG(_logger, "  origin_y: %.3f",
                     msg->metadata.origin.position.y);
    }

    void p_debugCostMap(nav2_costmap_2d::Costmap2D *costmap_nav2) {
        RCLCPP_DEBUG(_logger, "costmap_nav2: ");
        RCLCPP_DEBUG(_logger, "  x_size: %d", costmap_nav2->getSizeInCellsX());
        RCLCPP_DEBUG(_logger, "  y_size: %d", costmap_nav2->getSizeInCellsY());
        RCLCPP_DEBUG(_logger, "  Resolution: %.3f",
                     costmap_nav2->getResolution());
        RCLCPP_DEBUG(_logger, "  origin_x: %.3f", costmap_nav2->getOriginX());
        RCLCPP_DEBUG(_logger, "  origin_y: %.3f", costmap_nav2->getOriginY());
    }

    void p_defaultSolution(nav_msgs::msg::Path *plan,
                           const geometry_msgs::msg::PoseStamped *next_waypoint,
                           const std::string reason) {
        RCLCPP_WARN(_logger, "Default solution emitted. Reason: [%s]",
                    reason.c_str());

        plan->poses.push_back(*next_waypoint); // default just emit the next
    }

    geometry_msgs::msg::PoseStamped::SharedPtr
    pConvertToEgoFov(geometry_msgs::msg::Pose original,
                     nav_msgs::msg::Odometry odom,
                     std::shared_ptr<tf2_ros::Buffer> tf_buffer_,
                     rclcpp::Clock::SharedPtr clock) {
        geometry_msgs::msg::TransformStamped t;
        std::string fromFrameRel = odom.header.frame_id;
        std::string toFrameRel = odom.child_frame_id;
        try {
            t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel,
                                            tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(_logger, "Could not transform [%s] to [%s]: [%s]",
                        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            return nullptr;
        }
        std_msgs::msg::Header header;
        header.frame_id = toFrameRel;
        header.stamp = clock->now();
        geometry_msgs::msg::PoseStamped child_pose;
        geometry_msgs::msg::PoseStamped ps;
        ps.header = header;
        ps.pose = original;
        tf2::doTransform(ps, child_pose, t);
        return std::make_shared<geometry_msgs::msg::PoseStamped>(child_pose);
    }
};
} // namespace local_planning
#endif // A_STAR_HPP_
