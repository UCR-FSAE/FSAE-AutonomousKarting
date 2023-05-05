#include "trajectory_generator/trajectory_generator_ros.hpp"
#include <nav2_util/node_utils.hpp>


namespace local_planning
{

    TrajectoryGeneratorROS::TrajectoryGeneratorROS(const std::string &name)
        : TrajectoryGeneratorROS(name, "/", name) {}

    TrajectoryGeneratorROS::TrajectoryGeneratorROS(
        const std::string &name,
        const std::string &parent_namespace,
        const std::string &local_namespace
        )
        : LifecycleNode(name, "", true,
                        rclcpp::NodeOptions().arguments({"--ros-args", "-r", std::string("__ns:=") + nav2_util::add_namespaces(parent_namespace, local_namespace),
                                                         "--ros-args", "-r", name + ":" + std::string("__node:=") + name})),
          name_(name),
          parent_namespace_(parent_namespace)
    {
        this->declare_parameter("debug", true);
        
        if (this->get_parameter("debug").as_bool())
        {
            auto ret = rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG); // enable or disable debug
        }
        RCLCPP_INFO(this->get_logger(), "TrajectoryGeneratorROS initialized with Debug Mode = [%s]", this->get_parameter("debug").as_bool() ? "YES" : "NO");
    }
    TrajectoryGeneratorROS::TrajectoryGeneratorROS(
        const std::string &name,
        const std::string &parent_namespace,
        const std::string &local_namespace,
        const bool is_debug
        )
        : LifecycleNode(name, "", true,
                        rclcpp::NodeOptions().arguments({"--ros-args", "-r", std::string("__ns:=") + nav2_util::add_namespaces(parent_namespace, local_namespace),
                                                         "--ros-args", "-r", name + ":" + std::string("__node:=") + name})),
          name_(name),
          parent_namespace_(parent_namespace)
    {
        this->declare_parameter("debug", is_debug);
        
        if (this->get_parameter("debug").as_bool())
        {
            auto ret = rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG); // enable or disable debug
        }
        RCLCPP_INFO(this->get_logger(), "TrajectoryGeneratorROS initialized with Debug Mode = [%s]", this->get_parameter("debug").as_bool() ? "YES" : "NO");
    }

    TrajectoryGeneratorROS ::~TrajectoryGeneratorROS()
    {
    }

    void TrajectoryGeneratorROS::registerTrajectoryGenerator(const std::shared_ptr<TrajectoryGeneratorInterface> generator)
    {
        // Add the generator to the vector of trajectory generators
        this->trajectory_generators.push_back(generator);
    }

    /* Lifecycle */
    nav2_util::CallbackReturn
    TrajectoryGeneratorROS::on_configure(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_DEBUG(this->get_logger(), "on_configure");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn TrajectoryGeneratorROS::on_activate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_DEBUG(this->get_logger(), "on_activate");
        this->action_server_ = rclcpp_action::create_server<TrajectoryGeneration>(
            this,
            "trajectory_generation",
            std::bind(&TrajectoryGeneratorROS::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TrajectoryGeneratorROS::handle_cancel, this, std::placeholders::_1),
            std::bind(&TrajectoryGeneratorROS::handle_accepted, this, std::placeholders::_1));
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn TrajectoryGeneratorROS::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_DEBUG(this->get_logger(), "on_deactivate");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn TrajectoryGeneratorROS::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_DEBUG(this->get_logger(), "on_cleanup");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn TrajectoryGeneratorROS::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_DEBUG(this->get_logger(), "on_shutdown");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    /* Action server */
    rclcpp_action::GoalResponse
    TrajectoryGeneratorROS::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const TrajectoryGeneration::Goal> goal)
    {
        RCLCPP_DEBUG(get_logger(), "received goal - UUID=[%s]", rclcpp_action::to_string(uuid).c_str());
        if (this->canExecute())
        {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        return rclcpp_action::GoalResponse::REJECT;
        
    }

    rclcpp_action::CancelResponse
    TrajectoryGeneratorROS::handle_cancel(const std::shared_ptr<GoalHandleTrajectoryGeneration> goal_handle)
    {
        RCLCPP_DEBUG(get_logger(), "handle_cancel");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void TrajectoryGeneratorROS::handle_accepted(const std::shared_ptr<GoalHandleTrajectoryGeneration> goal_handle)
    {
        RCLCPP_DEBUG(get_logger(), "handle_accepted - goal uuid: [%s]", rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
        std::thread{std::bind(&TrajectoryGeneratorROS::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void TrajectoryGeneratorROS::execute(const std::shared_ptr<GoalHandleTrajectoryGeneration> goal_handle)
    {
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        active_goal_ = goal_handle;
        // prep for result
        std::shared_ptr<planning_interfaces::msg::Trajectories> all_trajectories = std::make_shared<planning_interfaces::msg::Trajectories>();

        // extract information
        std::shared_ptr<const planning_interfaces::action::TrajectoryGeneration_Goal>
            goal = goal_handle->get_goal();
        std::shared_ptr<nav2_msgs::msg::Costmap> costmap =
            std::make_shared<nav2_msgs::msg::Costmap>(goal->costmap);

        std::shared_ptr<nav_msgs::msg::Odometry> odom =
            std::make_shared<nav_msgs::msg::Odometry>(goal->odom);

        std::shared_ptr<geometry_msgs::msg::PoseStamped> next_waypoint =
            std::make_shared<geometry_msgs::msg::PoseStamped>(goal->next_waypoint);
        auto result = std::make_shared<TrajectoryGeneration::Result>();

        // TODO: each generator will have its own thread 
        // pass into generators
        for (const auto &generator: this->trajectory_generators) {
            planning_interfaces::msg::Trajectory trajectory; 

            std_msgs::msg::Header header;
            header.frame_id = "ego_vehicle"; // TODO: change this
            header.stamp = this->get_clock()->now();
            trajectory.header = header;

            nav_msgs::msg::Path path = generator->computeTrajectory(costmap, odom, next_waypoint);
            trajectory.trajectory = path;

            // TODO: implement scoring
            planning_interfaces::msg::TrajectoryScore score;
            score.name = "default";
            score.raw_score = 1.0;
            score.scale = 1.0;
            trajectory.score = score; 

            auto feedback = std::make_shared<TrajectoryGeneration::Feedback>();
            feedback->trajectory = trajectory;
            goal_handle->publish_feedback(feedback);
            all_trajectories->trajectories.push_back(trajectory);

            if (goal_handle->is_canceling())
            {
                goal_handle->canceled(result);
                active_goal_ = nullptr; // release the active goal checker
                RCLCPP_DEBUG(this->get_logger(), "Goal canceled");
                return;
            }
            
        }

        result->trajectories = *all_trajectories;
        goal_handle->succeed(result);
        active_goal_ = nullptr; // release the active goal checker
        RCLCPP_DEBUG(this->get_logger(), "Goal reached");
    }

    bool TrajectoryGeneratorROS::canExecute()
    {
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        if (active_goal_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Another goal is already active");
            return false;
        }
        return true;
    }

} // local_planning