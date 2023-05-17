#include "a_star/a_star.hpp"
#include <rclcpp/logging.hpp>

using namespace std::chrono; // NOLINT
#define BENCHMARK_TESTING

namespace local_planning {
void AStar::configure(rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
                      std::shared_ptr<tf2_ros::Buffer> tf) {
    this->parent_node = parent;
    this->tf_buffer = tf;
    this->name = name;

    p_bridgeSMACConfigure();

    RCLCPP_INFO(_logger, "Configured");
}

void AStar::cleanup() {
    this->parent_node = nullptr;
    this->tf_buffer = nullptr;
    RCLCPP_INFO(_logger, "cleanup");
}

void AStar::activate() {
    // Implementation details for activation
    RCLCPP_INFO(rclcpp::get_logger(this->name), "activate");
}

void AStar::deactivate() {
    // Implementation details for deactivation
    RCLCPP_INFO(rclcpp::get_logger(this->name), "deactivate");
}

void AStar::p_bridgeSMACConfigure() {
    // General planner params
    nav2_util::declare_parameter_if_not_declared(
        this->parent_node, name + ".tolerance", rclcpp::ParameterValue(0.125));
    _tolerance = static_cast<float>(
        this->parent_node->get_parameter(name + ".tolerance").as_double());

    nav2_util::declare_parameter_if_not_declared(this->parent_node,
                                                 name + ".downsampling_factor",
                                                 rclcpp::ParameterValue(1));

    nav2_util::declare_parameter_if_not_declared(
        this->parent_node, name + ".angle_quantization_bins",
        rclcpp::ParameterValue(72));
    this->parent_node->get_parameter(name + ".angle_quantization_bins",
                                     angle_quantizations);
    _angle_bin_size = 2.0 * M_PI / angle_quantizations;
    _angle_quantizations = static_cast<unsigned int>(angle_quantizations);

    nav2_util::declare_parameter_if_not_declared(this->parent_node,
                                                 name + ".allow_unknown",
                                                 rclcpp::ParameterValue(true));
    this->parent_node->get_parameter(name + ".allow_unknown", allow_unknown);
    nav2_util::declare_parameter_if_not_declared(this->parent_node,
                                                 name + ".max_iterations",
                                                 rclcpp::ParameterValue(-1));
    this->parent_node->get_parameter(name + ".max_iterations", max_iterations);
    nav2_util::declare_parameter_if_not_declared(this->parent_node,
                                                 name + ".smooth_path",
                                                 rclcpp::ParameterValue(false));
    this->parent_node->get_parameter(name + ".smooth_path", smooth_path);

    nav2_util::declare_parameter_if_not_declared(
        this->parent_node, name + ".minimum_turning_radius",
        rclcpp::ParameterValue(0.2));
    this->parent_node->get_parameter(name + ".minimum_turning_radius",
                                     search_info.minimum_turning_radius);

    nav2_util::declare_parameter_if_not_declared(this->parent_node,
                                                 name + ".reverse_penalty",
                                                 rclcpp::ParameterValue(2.0));
    this->parent_node->get_parameter(name + ".reverse_penalty",
                                     search_info.reverse_penalty);
    nav2_util::declare_parameter_if_not_declared(this->parent_node,
                                                 name + ".change_penalty",
                                                 rclcpp::ParameterValue(0.5));
    this->parent_node->get_parameter(name + ".change_penalty",
                                     search_info.change_penalty);
    nav2_util::declare_parameter_if_not_declared(this->parent_node,
                                                 name + ".non_straight_penalty",
                                                 rclcpp::ParameterValue(1.05));
    this->parent_node->get_parameter(name + ".non_straight_penalty",
                                     search_info.non_straight_penalty);
    nav2_util::declare_parameter_if_not_declared(
        this->parent_node, name + ".cost_penalty", rclcpp::ParameterValue(1.2));
    this->parent_node->get_parameter(name + ".cost_penalty",
                                     search_info.cost_penalty);
    nav2_util::declare_parameter_if_not_declared(
        this->parent_node, name + ".analytic_expansion_ratio",
        rclcpp::ParameterValue(2.0));
    this->parent_node->get_parameter(name + ".analytic_expansion_ratio",
                                     search_info.analytic_expansion_ratio);

    nav2_util::declare_parameter_if_not_declared(
        this->parent_node, name + ".max_planning_time_ms",
        rclcpp::ParameterValue(5000.0));
    this->parent_node->get_parameter(name + ".max_planning_time_ms",
                                     _max_planning_time);

    nav2_util::declare_parameter_if_not_declared(
        this->parent_node, name + ".motion_model_for_search",
        rclcpp::ParameterValue(std::string("DUBIN")));
    this->parent_node->get_parameter(name + ".motion_model_for_search",
                                     motion_model_for_search);
    motion_model = smac_planner::fromString(motion_model_for_search);
    if (motion_model == smac_planner::MotionModel::UNKNOWN) {
        RCLCPP_WARN(rclcpp::get_logger(this->name),
                    "Unable to get MotionModel search type. Given '%s', "
                    "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP.",
                    motion_model_for_search.c_str());
    }

    if (max_on_approach_iterations <= 0) {
        RCLCPP_INFO(rclcpp::get_logger(this->name),
                    "On approach iteration selected as <= 0, "
                    "disabling tolerance and on approach iterations.");
        max_on_approach_iterations = std::numeric_limits<int>::max();
    }

    if (max_iterations <= 0) {
        RCLCPP_INFO(rclcpp::get_logger(this->name),
                    "maximum iteration selected as <= 0, disabling maximum "
                    "iterations.");
        max_iterations = std::numeric_limits<int>::max();
    }
    // convert to grid coordinates
    const double minimum_turning_radius_global_coords =
        search_info.minimum_turning_radius;

    if (smooth_path) {
        _smoother = std::make_unique<smac_planner::Smoother>();
        _optimizer_params.get(this->parent_node.get(), name);
        _smoother_params.get(this->parent_node.get(), name);
        _smoother_params.max_curvature =
            1.0f / minimum_turning_radius_global_coords;
        _smoother->initialize(_optimizer_params);
    }

    _raw_plan_publisher =
        this->parent_node->create_publisher<nav_msgs::msg::Path>(
            "unsmoothed_plan", 1);

    _a_star =
        std::make_unique<smac_planner::AStarAlgorithm<smac_planner::NodeSE2>>(
            motion_model, search_info);

    _a_star->initialize(allow_unknown, max_iterations,
                        max_on_approach_iterations);

    _raw_plan_publisher->on_activate();

    RCLCPP_INFO(
        rclcpp::get_logger(this->name),
        "Configured plugin %s of type SmacPlanner with "
        "tolerance %.2f, maximum iterations %i, "
        "max on approach iterations %i, and %s. Using motion model: %s.",
        _name.c_str(), _tolerance, max_iterations, max_on_approach_iterations,
        allow_unknown ? "allowing unknown traversal"
                      : "not allowing unknown traversal",
        toString(motion_model).c_str());
}

nav_msgs::msg::Path AStar::computeTrajectory(
    const nav2_msgs::msg::Costmap::SharedPtr costmap,
    const nav_msgs::msg::Odometry::SharedPtr odom,
    const geometry_msgs::msg::PoseStamped::SharedPtr next_waypoint) {

    nav_msgs::msg::Path plan;
    // Implementation details for trajectory computation using A*

    RCLCPP_INFO(rclcpp::get_logger(this->name), "minimum_turning_radius = %d",
                search_info.minimum_turning_radius);

    search_info.minimum_turning_radius =
        search_info.minimum_turning_radius / (costmap->metadata.resolution * 1);

    // TODO: set footprint somehow
    // _a_star->setFootprint(costmap_ros.get()->getRobotFootprint(), false);
    steady_clock::time_point a = steady_clock::now();

    nav2_costmap_2d::Costmap2D *costmap_nav2 = new nav2_costmap_2d::Costmap2D(
        costmap.get()->metadata.size_x, costmap.get()->metadata.size_y,
        costmap.get()->metadata.resolution,
        costmap.get()->metadata.origin.position.x,
        costmap.get()->metadata.origin.position.y);
    if (this->fillCostmapFromMsg(costmap_nav2, costmap)) {
        plan.poses.push_back(*next_waypoint); // default just emit the next
                                              // waypoint, dummy generator
        return plan;
    }

    _a_star->createGraph(costmap.get()->metadata.size_x,
                         costmap.get()->metadata.size_y, _angle_quantizations,
                         costmap_nav2);

    // Set starting point, in A* bin search coordinates
    unsigned int mx, my;

    costmap_nav2->worldToMap(odom->pose.pose.position.x,
                             odom->pose.pose.position.y, mx, my);
    double orientation_bin =
        tf2::getYaw(odom->pose.pose.orientation) / _angle_bin_size;
    while (orientation_bin < 0.0) {
        orientation_bin += static_cast<float>(_angle_quantizations);
    }
    unsigned int orientation_bin_id =
        static_cast<unsigned int>(floor(orientation_bin));
    _a_star->setStart(mx, my, orientation_bin_id);

    // Set goal point, in A* bin search coordinates
    costmap_nav2->worldToMap(next_waypoint->pose.position.x,
                             next_waypoint->pose.position.y, mx, my);
    orientation_bin =
        tf2::getYaw(next_waypoint->pose.orientation) / _angle_bin_size;
    while (orientation_bin < 0.0) {
        orientation_bin += static_cast<float>(_angle_quantizations);
    }
    orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));
    _a_star->setGoal(mx, my, orientation_bin_id);

    // Setup message
    plan.header.stamp = _clock->now();
    plan.header.frame_id = _global_frame;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    // Compute plan
    smac_planner::NodeSE2::CoordinateVector path;
    int num_iterations = 0;
    std::string error;
    try {
        if (!_a_star->createPath(
                path, num_iterations,
                _tolerance /
                    static_cast<float>(costmap_nav2->getResolution()))) {
            if (num_iterations < _a_star->getMaxIterations()) {
                error = std::string("no valid path found");
            } else {
                error = std::string("exceeded maximum iterations");
            }
        }
    } catch (const std::runtime_error &e) {
        error = "invalid use: ";
        error += e.what();
    }

    if (!error.empty()) {
        RCLCPP_WARN(_logger, "%s: failed to create plan, %s.", _name.c_str(),
                    error.c_str());
        return plan;
    }

    // Convert to world coordinates and downsample path for smoothing if
    // necesssary We're going to downsample by 4x to give terms room to move.
    const int downsample_ratio = 4;
    std::vector<Eigen::Vector2d> path_world;
    path_world.reserve(path.size());
    plan.poses.reserve(path.size());

    for (int i = path.size() - 1; i >= 0; --i) {
        path_world.push_back(
            getWorldCoords(path[i].x, path[i].y, costmap_nav2));
        pose.pose.position.x = path_world.back().x();
        pose.pose.position.y = path_world.back().y();
        pose.pose.orientation = getWorldOrientation(path[i].theta);
        plan.poses.push_back(pose);
    }

    // Publish raw path for debug
    if (_raw_plan_publisher->get_subscription_count() > 0) {
        _raw_plan_publisher->publish(plan);
    }

    // If not smoothing or too short to smooth, return path
    if (!_smoother || path_world.size() < 4) {
#ifdef BENCHMARK_TESTING
        steady_clock::time_point b = steady_clock::now();
        duration<double> time_span = duration_cast<duration<double>>(b - a);
        std::cout << "It took " << time_span.count() * 1000
                  << " milliseconds with " << num_iterations << " iterations."
                  << std::endl;
#endif
        return plan;
    }

    // Find how much time we have left to do smoothing
    steady_clock::time_point b = steady_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(b - a);
    double time_remaining =
        _max_planning_time - static_cast<double>(time_span.count());
    _smoother_params.max_time =
        std::min(time_remaining, _optimizer_params.max_time);

    // Smooth plan
    if (!_smoother->smooth(path_world, costmap_nav2, _smoother_params)) {
        RCLCPP_WARN(_logger,
                    "%s: failed to smooth plan, Ceres could not find a usable "
                    "solution to optimize.",
                    _name.c_str());
        return plan;
    }

    removeHook(path_world);

    // populate final path
    // TODO(stevemacenski): set orientation to tangent of path
    for (unsigned int i = 0; i != path_world.size(); i++) {
        pose.pose.position.x = path_world[i][0];
        pose.pose.position.y = path_world[i][1];
        plan.poses[i] = pose;
    }

    return plan;
}
} // namespace local_planning
