#include "pid_control/pid_control_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <math.h>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>
namespace pid_controller
{
    PIDControlActionServer::PIDControlActionServer(const rclcpp::NodeOptions &options) : LifecycleNode("pid_control_action_server", "",true)
    {
        RCLCPP_INFO(get_logger(), "Creating PID Control Server Node");
        this->is_server_running = false;
        this->declare_parameter("debug", false);
        this->declare_parameter("loop_rate", 10.0);
    }
    PIDControlActionServer::~PIDControlActionServer()
    {
        RCLCPP_INFO(this->get_logger(), "Destroying PIDControlActionServer");
        ackermann_msgs::msg::AckermannDriveStamped msg;
        msg.drive.jerk = 0;
        this->ackermann_publisher_->publish(msg);
    }

    nav2_util::CallbackReturn
        PIDControlActionServer::on_configure(const rclcpp_lifecycle::State &state)
    {
        this->is_server_running = false;
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/carla/ego_vehicle/odometry", rclcpp::SystemDefaultsQoS(),
            std::bind(&PIDControlActionServer::onLatestOdomReceived, this, std::placeholders::_1));
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_drive", 10);
        next_waypoint_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/pid_controller/next_waypoint_visualization", 10);

        action_server_ = rclcpp_action::create_server<ControlAction>(
            get_node_base_interface(),
            get_node_clock_interface(),
            get_node_logging_interface(),
            get_node_waitables_interface(),
            "pid_control",
            std::bind(&PIDControlActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PIDControlActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&PIDControlActionServer::handle_accepted, this, std::placeholders::_1));
        longitudinal_pid_node_ = rclcpp::Node::make_shared("longitudinal_pid_node");
        lateral_pid_node_ = rclcpp::Node::make_shared("lateral_pid_node");

        longitudinal_pid = std::shared_ptr<control_toolbox::PidROS>(new control_toolbox::PidROS(this->longitudinal_pid_node_, "longitudinal_pid_controller"));
        lateral_pid = std::shared_ptr<control_toolbox::PidROS>(new control_toolbox::PidROS(this->lateral_pid_node_, "lateral_pid_controller"));

        // TODO: look into what is antiwindup
        // TODO: have another service dynamically tune kp, kd, ki
        longitudinal_pid->initPid(0.05, 0, 0.1, 5, 0, false);
        lateral_pid->initPid(     0.05, 0, 0, 1, 0, false);
        RCLCPP_INFO(this->get_logger(), "PID Control Server Configured");

        return nav2_util::CallbackReturn::SUCCESS;
    }
    nav2_util::CallbackReturn PIDControlActionServer::on_activate(const rclcpp_lifecycle::State &state)
    {

        this->ackermann_publisher_->on_activate();
        this->next_waypoint_pub_->on_activate();
        this->is_server_running = true;
        RCLCPP_INFO(this->get_logger(), "PID Control Server Activated");

        return nav2_util::CallbackReturn::SUCCESS;
    }
    nav2_util::CallbackReturn PIDControlActionServer::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        this->is_server_running = false;
        RCLCPP_INFO(this->get_logger(), "PID Control Server Deactivated");

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn PIDControlActionServer::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(this->get_logger(), "PID Control Server Shutdown");

        return nav2_util::CallbackReturn::SUCCESS;
    }

    rclcpp_action::GoalResponse PIDControlActionServer::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ControlAction::Goal> goal)
    {
        // RCLCPP_INFO(this->get_logger(), "Received goal request with [%d] waypoints", goal->path.poses.size());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse PIDControlActionServer::handle_cancel(
        const std::shared_ptr<GoalHandleControlAction> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        this->is_thread_running = false;
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void PIDControlActionServer::handle_accepted(const std::shared_ptr<GoalHandleControlAction> goal_handle)
    {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        if (this->is_thread_running)
        {
            this->is_thread_running = false;
            rclcpp::WallRate loop_rate(this->get_parameter("loop_rate").as_double());
            loop_rate.sleep();
        }
        
        std::thread{std::bind(&PIDControlActionServer::execute, this, _1), goal_handle}.detach();
    }
    void PIDControlActionServer::onLatestOdomReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        this->latest_odom = msg;
    }

    float PIDControlActionServer::calculateThrottleError(nav_msgs::msg::Odometry::SharedPtr odom, float target_spd)
    {
        float current_spd = this->p_spd_from_odom(odom);
        if (this->get_parameter("debug").as_bool())
        {
            RCLCPP_INFO(this->get_logger(), "target_spd: %f | curr_spd: %f", target_spd, current_spd);
        }
        return target_spd - current_spd;
    }
    float PIDControlActionServer::calculateSteeringError(nav_msgs::msg::Odometry::SharedPtr odom, geometry_msgs::msg::Pose::SharedPtr next_waypoint)
    {
        float tx = next_waypoint->position.x;
        float ty = next_waypoint->position.y;
        float desired_yaw = -1*std::atan2(ty, tx);

        float steering_error = desired_yaw; // after mathmatical deduction, steering error = desired yaw
        if (this->get_parameter("debug").as_bool())
        {
            RCLCPP_INFO(this->get_logger(), "desired_yaw: %f ", desired_yaw);
        }

            
        return steering_error;
    }

    double PIDControlActionServer::yawFromOdom(nav_msgs::msg::Odometry::SharedPtr odom)
    {
        tf2::Quaternion quat(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
                             odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
        tf2::Matrix3x3 mat(quat);
        double yaw = atan2(mat[1][0], mat[0][0]);
        return yaw;
    }

    void PIDControlActionServer::execute(const std::shared_ptr<GoalHandleControlAction> goal_handle)
    {
        // RCLCPP_INFO(this->get_logger(), "Executing...");
        this->is_thread_running = true;
        rclcpp::WallRate loop_rate(this->get_parameter("loop_rate").as_double());
        rclcpp::Time start_time = this->get_clock()->now();
        while (rclcpp::ok())
        {
            if (this->is_thread_running == false)
            {
                break;
            }
            if (this->is_server_running == false)
            {
                RCLCPP_INFO(this->get_logger(), "Server is not running, not executing...");
                break;
            }

            if (this->latest_odom == nullptr)
            {
                loop_rate.sleep();
                continue;
            }
            

            // find the immediate next point to follow
            float target_spd = goal_handle->get_goal().get()->target_spd;
            std::shared_ptr<nav_msgs::msg::Odometry> odom = this->latest_odom;
            nav_msgs::msg::Path path = goal_handle->get_goal().get()->path;
            std::shared_ptr<geometry_msgs::msg::Pose> next_waypoint = std::make_shared<geometry_msgs::msg::Pose>(this->findNextWaypoint(path, odom));

            if (this->get_parameter("debug").as_bool())
            {
                this->p_publish_visualization_from_pose(next_waypoint);
            }

            // Run PID controller
            rclcpp::Time end_time = this->get_clock()->now();
            rclcpp::Duration dt = end_time - start_time;
            float throttle_error = this->calculateThrottleError(odom, target_spd);
            float steering_error = this->calculateSteeringError(odom, next_waypoint);
            float throttle_cmd = std::max(0.0f, std::min(1.0f, float(this->longitudinal_pid->computeCommand(throttle_error, dt))));
            float steering_cmd = std::max(-1.0f, std::min(1.0f, float(this->lateral_pid->computeCommand(steering_error, dt))));
            if (this->get_parameter("debug").as_bool())
            {
                RCLCPP_INFO(this->get_logger(), "target_spd: %f | throttle_error: %f | throttle_cmd: %f", target_spd, throttle_error, throttle_cmd);
                RCLCPP_INFO(this->get_logger(), "steering_error: %f | steering_cmd: %f", steering_error, steering_cmd);
            }                

            this->p_publish_ackermann_msg(steering_error,
                                            steering_cmd,
                                            throttle_cmd,
                                            odom->header.frame_id);
 
            start_time = end_time;
            loop_rate.sleep();
        }
    }
    float PIDControlActionServer::p_spd_from_odom(nav_msgs::msg::Odometry::SharedPtr odom)
    {
        geometry_msgs::msg::Vector3 linear_velocities = odom->twist.twist.linear;
        double vx = linear_velocities.x;
        double vy = linear_velocities.y;
        double vz = linear_velocities.z;
        float current_spd = std::sqrt(vx * vx + vy * vy + vz * vz);
        return current_spd;
    }

    void PIDControlActionServer::p_publish_visualization_from_pose(geometry_msgs::msg::Pose::SharedPtr pose)
    {
        if (this->latest_odom)
        {
            visualization_msgs::msg::Marker marker;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = *pose;
            // marker.pose.position.x = 5.0;
            // marker.pose.position.y = 0.0;
            // marker.pose.position.z = 1.0;
            // Set the marker scale
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;

            // Set the marker color
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            marker.lifetime = rclcpp::Duration(std::chrono::seconds(2));

            std_msgs::msg::Header header;

            header.frame_id = this->latest_odom->child_frame_id;
            header.stamp = this->get_clock()->now();

            // Set the header in the marker message
            marker.header = header;
            this->next_waypoint_pub_->publish(marker);
        }
    }

    void PIDControlActionServer::p_publish_ackermann_msg(float steering_error, float steering_cmd, float throttle_cmd, std::string frame_id)
    {
        ackermann_msgs::msg::AckermannDriveStamped msg;
        msg.header.frame_id = frame_id;
        msg.header.stamp = this->get_clock()->now();
        msg.drive.jerk = std::max(-1.0f, std::min(1.0f, throttle_cmd));
        msg.drive.steering_angle = steering_error;
        msg.drive.steering_angle_velocity = std::max(-1.0f, std::min(1.0f, steering_cmd));
        // TODO: add in reverse here
        ackermann_publisher_->publish(msg);
    }

    geometry_msgs::msg::Pose PIDControlActionServer::findNextWaypoint(const nav_msgs::msg::Path path, const std::shared_ptr<nav_msgs::msg::Odometry> odom)
    {
        // TODO: Better strategy to find the next immediate point to follow 
        return path.poses[0].pose;
    }

    // TODO: implement function that set gains for long and lat pid controller
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pid_controller::PIDControlActionServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
