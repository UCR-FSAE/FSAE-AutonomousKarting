#include <controller_manager/controller_manager_ros.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <controller_manager/pid_controller.hpp>

namespace controller
{
  ControllerManagerNode::ControllerManagerNode() : LifecycleNode("manager", "controller", true)
  {
    this->declare_parameter("debug", false);
    
    if (this->get_parameter("debug").as_bool())
    {
        auto ret = rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG); // enable or disable debug
    }

    RCLCPP_INFO(this->get_logger(), "ControllerManagerNode initialized with Debug Mode = [%s]", this->get_parameter("debug").as_bool() ? "YES" : "NO");
    this->stop_flag = true;


    // TODO: load config from parameters
    std::map<const std::string, boost::any> dict = {{"key1", 42}, {"key2", std::string("hello")}};
    this->registerControlAlgorithm(PID, dict); 
  }
  ControllerManagerNode ::~ControllerManagerNode()
  {
    this->stop_flag = true;
  }


  /**
   * Lifecycles
  */
  nav2_util::CallbackReturn ControllerManagerNode::on_configure(const rclcpp_lifecycle::State &state) 
  {
    RCLCPP_DEBUG(get_logger(), "on_configure");
    this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/carla/ego_vehicle/odometry", rclcpp::SystemDefaultsQoS(),
        std::bind(&ControllerManagerNode::onLatestOdomReceived, this, std::placeholders::_1));

    this->action_server_ = rclcpp_action::create_server<ControlAction>(
      this,
      std::string(this->get_namespace()) + "/" + std::string(this->get_name()),
      std::bind(&ControllerManagerNode::handle_goal, this, std::placeholders::_1,std::placeholders:: _2),
      std::bind(&ControllerManagerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&ControllerManagerNode::handle_accepted, this, std::placeholders::_1));
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn ControllerManagerNode::on_activate(const rclcpp_lifecycle::State &state) 
  {
    RCLCPP_DEBUG(get_logger(), "on_activate");
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn ControllerManagerNode::on_deactivate(const rclcpp_lifecycle::State &state) 
  {
    RCLCPP_DEBUG(get_logger(), "on_deactivate");
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn ControllerManagerNode::on_cleanup(const rclcpp_lifecycle::State &state) 
  {
    RCLCPP_DEBUG(get_logger(), "on_cleanup");
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn ControllerManagerNode::on_shutdown(const rclcpp_lifecycle::State &state) 
  {
    RCLCPP_DEBUG(get_logger(), "on_shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  /**
   * subscriber
  */
  void ControllerManagerNode::onLatestOdomReceived(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    this->latest_odom = msg;
  }

  /**
   * Action server
  */
  rclcpp_action::GoalResponse ControllerManagerNode::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ControlAction::Goal> goal)
  {
    RCLCPP_DEBUG(get_logger(), "received goal - goal uuid: [%s]", rclcpp_action::to_string(uuid).c_str());

    
    if (this->canExecute())
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    if (goal->overwrite_previous && active_goal_ != nullptr)
    {
        auto result = std::make_shared<ControlAction::Result>();
        result->status = result->NORMAL;
        active_goal_->abort(result);
        active_goal_ = nullptr; // release the active goal checker
        stop_flag = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    return rclcpp_action::GoalResponse::REJECT;
  }
  rclcpp_action::CancelResponse ControllerManagerNode::handle_cancel(const std::shared_ptr<GoalHandleControlAction> goal_handle)
  {
    RCLCPP_DEBUG(get_logger(), "handle_cancel - goal uuid: [%s]", rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void ControllerManagerNode::handle_accepted(const std::shared_ptr<GoalHandleControlAction> goal_handle)
  {
    RCLCPP_DEBUG(get_logger(), "handle_accepted - goal uuid: [%s]", rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
    stop_flag = false;
    std::thread{std::bind(&ControllerManagerNode::p_execute, this, std::placeholders::_1), goal_handle}.detach();

  }

  /**
   * main execution loops
  */
  void ControllerManagerNode::p_execute(const std::shared_ptr<GoalHandleControlAction> goal_handle)
  {

      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_goal_ = goal_handle;
      auto result = std::make_shared<ControlAction::Result>();

      while (!this->stop_flag)
      {
        if (goal_handle->is_canceling())
        {
            result->status = control_interfaces::action::Control::Result::NORMAL;
            goal_handle->canceled(result);
            RCLCPP_DEBUG(get_logger(), "goal canceled");
            return;
        }

        result->status = control_interfaces::action::Control::Result::NORMAL;
        goal_handle->succeed(result);
        active_goal_ = nullptr; // release lock so that next iteration can execute
        RCLCPP_DEBUG(get_logger(), "goal reached");
        return;
      }
      result->status = -1;
      goal_handle->canceled(result);
      RCLCPP_DEBUG(get_logger(), "execution loop exited abnormally");
      
  }
  bool ControllerManagerNode::canExecute()
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    if (active_goal_)
    {
        RCLCPP_WARN(this->get_logger(), "Another goal is already active, please cancel [%s] before sending a new goal", rclcpp_action::to_string(active_goal_->get_goal_id()).c_str());
        return false;
    }
    return true;
  }

  void ControllerManagerNode::registerControlAlgorithm(const Algorithms algo, const std::map<const std::string, boost::any> configs)
  {
    switch (algo)
    {
      case PID:
        this->controller = std::make_shared<controller::PIDController>();
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unable to match control algorithm");
        break;
    }
    if (this->controller != nullptr)
    {
      this->controller->setup(configs);
    }
    
  }

}
