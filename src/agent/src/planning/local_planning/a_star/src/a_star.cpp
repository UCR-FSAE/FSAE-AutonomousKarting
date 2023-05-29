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

    _clock = parent->get_clock();

    p_bridgeSMACConfigure();

    RCLCPP_DEBUG(_logger, "Configured");
}

void AStar::cleanup() {
    this->parent_node = nullptr;
    this->tf_buffer = nullptr;
    RCLCPP_DEBUG(_logger, "cleanup");
}

void AStar::activate() {
    // Implementation details for activation
    RCLCPP_DEBUG(_logger, "activate");
}

void AStar::deactivate() {
    // Implementation details for deactivation
    RCLCPP_DEBUG(_logger, "deactivate");
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
        RCLCPP_WARN(_logger,
                    "Unable to get MotionModel search type. Given '%s', "
                    "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP.",
                    motion_model_for_search.c_str());
    }

    if (max_on_approach_iterations <= 0) {
        RCLCPP_DEBUG(_logger,
                     "On approach iteration selected as <= 0, "
                     "disabling tolerance and on approach iterations.");
        max_on_approach_iterations = std::numeric_limits<int>::max();
    }

    if (max_iterations <= 0) {
        RCLCPP_DEBUG(_logger,
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

    p_debugSearchInfo(search_info);
    RCLCPP_DEBUG(
        _logger,
        "Configured plugin %s of type SmacPlanner with "
        "tolerance %.2f, maximum iterations %i, "
        "max on approach iterations %i, and %s. Using motion model: %s.",
        _name.c_str(), _tolerance, max_iterations, max_on_approach_iterations,
        allow_unknown ? "allowing unknown traversal"
                      : "not allowing unknown traversal",
        toString(motion_model).c_str());
}

nav_msgs::msg::Path AStar::computeTrajectory(
    const std::shared_ptr<
        const planning_interfaces::action::TrajectoryGeneration_Goal>
        request) {
    RCLCPP_DEBUG(_logger, "odom - x: [%.3f] | y: [% .3f]",
                 request->odom.pose.pose.position.x,
                 request->odom.pose.pose.position.y);
    RCLCPP_DEBUG(_logger, "next_waypoint - x: [%.3f] | y: [%.3f] | ",
                 request->next_waypoint.pose.position.x,
                 request->next_waypoint.pose.position.y);

    _global_frame = request->odom.header.frame_id;
    _ego_frame = request->odom.child_frame_id;
    nav_msgs::msg::Path plan_global;
    // Implementation details for trajectory computation using A*

    // TODO: set footprint somehow
    steady_clock::time_point a = steady_clock::now();

    /* set costmap and initiate graph for A* search */
    nav2_costmap_2d::Costmap2D *costmap_nav2 = new nav2_costmap_2d::Costmap2D(
        request->costmap.metadata.size_x, request->costmap.metadata.size_y,
        request->costmap.metadata.resolution,
        request->costmap.metadata.origin.position.x,
        request->costmap.metadata.origin.position.y);
    // _a_star->setFootprint(costmap_nav2->getRo, false);
    // p_debugCostMapMsg(&request->costmap);
    // p_debugCostMap(costmap_nav2);

    try {
        this->fillCostmapFromMsg(costmap_nav2, &request->costmap);
    } catch (const std::exception &e) {
        p_defaultSolution(&plan_global, &request->next_waypoint,
                          "unable to fill costmap from costmap message");
        return plan_global;
    }

    _a_star->createGraph(request->costmap.metadata.size_x,
                         request->costmap.metadata.size_y, _angle_quantizations,
                         costmap_nav2);

    /* set start */
    unsigned int mx, my;

    if (!costmap_nav2->worldToMap(0, 0, mx, my)) {

        RCLCPP_WARN(_logger, "start.x: [%.3f] | start.y: [%.3f]",
                    request->odom.pose.pose.position.x,
                    request->odom.pose.pose.position.y);
        p_defaultSolution(&plan_global, &request->next_waypoint,
                          "Failed to "
                          "convert robot position to map coord");
        return plan_global;
    }

    double orientation_bin =
        tf2::getYaw(request->odom.pose.pose.orientation) / _angle_bin_size;
    while (orientation_bin < 0.0) {
        orientation_bin += static_cast<float>(_angle_quantizations);
    }
    unsigned int orientation_bin_id =
        0; // static_cast<unsigned int>(floor(orientation_bin));
    _a_star->setStart(mx, my, 0); // orientation_bin_id);

    RCLCPP_DEBUG(_logger,
                 "start - mx: [%d] | my: [%d] | orientation_bin_id: [%d]", mx,
                 my, orientation_bin_id);
    /**
     * set goal location
     * calc distance from goal to ego so that we can find world->occupancy map
     * NOTE: we have reversed the coordinate here due to
     */
    float goal_x = request->next_waypoint.pose.position.y -
                   request->odom.pose.pose.position.y;
    float goal_y = request->next_waypoint.pose.position.x -
                   request->odom.pose.pose.position.x;

    RCLCPP_DEBUG(_logger,
                 "goal - x: [%.3f] | y: [%.3f] | orientation_bin_id: [%d]",
                 goal_x, goal_y, orientation_bin_id);

    if (!costmap_nav2->worldToMap(goal_x, goal_y, mx, my)) {
        p_defaultSolution(&plan_global, &request->next_waypoint,
                          "Failed to convert waypoint position to map coord");
        return plan_global;
    }

    orientation_bin =
        tf2::getYaw(request->next_waypoint.pose.orientation) / _angle_bin_size;

    while (orientation_bin < 0.0) {
        orientation_bin += static_cast<float>(_angle_quantizations);
    }

    orientation_bin_id =
        0; // static_cast<unsigned int>(floor(orientation_bin)); TODO: figure
           // out what is orientation bin

    RCLCPP_DEBUG(_logger,
                 "goal - mx: [%d] | my: [%d] | orientation_bin_id: [%d]", mx,
                 my, orientation_bin_id);

    _a_star->setGoal(mx, my, 0);
    RCLCPP_DEBUG(_logger, "goal set success");
    // Setup message
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan_global.header;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    plan_global.header.stamp = _clock->now();
    plan_global.header.frame_id = _global_frame;
    // Compute plan
    smac_planner::NodeSE2::CoordinateVector path;
    int num_iterations = 0;
    std::string error;
    try {
        RCLCPP_DEBUG(_logger, "Trying to create path...");
        RCLCPP_DEBUG(
            _logger,
            "num_iteration: [%d] | tolerance: [%.3f] | resolution: [%.3f]",
            num_iterations, _tolerance,
            static_cast<float>(costmap_nav2->getResolution()));
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
    } catch (...) {
        p_defaultSolution(&plan_global, &request->next_waypoint,
                          "unable to fill costmap from costmap message");
        RCLCPP_WARN(_logger, "%s: failed to create plan due to unknown "
                             "reasons, using default plan");
        return plan_global;
    }

    if (!error.empty()) {
        RCLCPP_WARN(_logger, "%s: failed to create plan, %s.", _name.c_str(),
                    error.c_str());
        return plan_global;
    }
    RCLCPP_DEBUG(_logger, "Plan creation success");

    // Convert to world coordinates and down sample path for smoothing if
    // necesssary We're going to downsample by 4x to give terms room to move.
    std::vector<Eigen::Vector2d> path_ego;

    path_ego.reserve(path.size());
    plan_global.poses.reserve(path.size());
    RCLCPP_DEBUG(_logger, "Coordinates on ego vehicle frame");
    for (int i = path.size() - 1; i >= 0; --i) {
        path_ego.push_back(getWorldCoords(path[i].x, path[i].y, costmap_nav2));
        pose.pose.position.x =
            path_ego.back().y() + request->odom.pose.pose.position.x;

        pose.pose.position.y =
            path_ego.back().x() + request->odom.pose.pose.position.y;

        pose.pose.orientation = getWorldOrientation(path[i].theta);
        plan_global.poses.push_back(pose);
    }

    // debug
    for (size_t i = 0; i < plan_global.poses.size(); i++) {
        RCLCPP_DEBUG(_logger, "i: [%d] | x: [%.3f] | y: [%.3f]", i,
                     plan_global.poses[i].pose.position.x,
                     plan_global.poses[i].pose.position.y);
    }

    // Publish raw path for debug, in ego frame
    if (_raw_plan_publisher->get_subscription_count() > 0) {
        nav_msgs::msg::Path plan_ego;
        plan_ego.header.frame_id = _ego_frame;

        for (size_t i = 0; i < plan_global.poses.size(); i++) {
            std::shared_ptr<geometry_msgs::msg::PoseStamped> ps;

            ps =
                this->pConvertToEgoFov(plan_global.poses[i].pose, request->odom,
                                       this->tf_buffer, this->_clock);

            if (ps == nullptr) {
                break;
            }

            plan_ego.poses.push_back(*ps.get());
        }

        _raw_plan_publisher->publish(plan_ego);
    }

    // If not smoothing or too short to smooth, return path
    if (!_smoother || path_ego.size() < 4) {
#ifdef BENCHMARK_TESTING
        steady_clock::time_point b = steady_clock::now();
        duration<double> time_span = duration_cast<duration<double>>(b - a);
#endif
        RCLCPP_DEBUG(_logger, "---------------");

        return plan_global;
    }

    // Find how much time we have left to do smoothing
    steady_clock::time_point b = steady_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(b - a);
    double time_remaining =
        _max_planning_time - static_cast<double>(time_span.count());
    _smoother_params.max_time =
        std::min(time_remaining, _optimizer_params.max_time);

    // Smooth plan
    if (!_smoother->smooth(path_ego, costmap_nav2, _smoother_params)) {
        RCLCPP_WARN(_logger,
                    "%s: failed to smooth plan, Ceres could not find a usable "
                    "solution to optimize.",
                    _name.c_str());
        RCLCPP_DEBUG(_logger, "---------------");
        return plan_global;
    }

    removeHook(path_ego);

    // populate final path
    for (unsigned int i = 0; i != path_ego.size(); i++) {
        pose.pose.position.x = path_ego[i][0];
        pose.pose.position.y = path_ego[i][1];
        plan_global.poses[i] = pose;
    }
    RCLCPP_DEBUG(_logger, "---------------");
    return plan_global;
}

} // namespace local_planning
