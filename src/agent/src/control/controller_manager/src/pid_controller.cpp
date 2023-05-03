#include <controller_manager/pid_controller.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace controller 
{
    void PIDController::configure(const std::map<std::string, boost::any> configuration, rclcpp_lifecycle::LifecycleNode *parent)
    {
        this->parent = parent;

        longitudinal_pid_configs = new std::map<float, control_toolbox::Pid::Gains>;
        lateral_pid_configs = new std::map<float, control_toolbox::Pid::Gains>;
        
        longitudinal_pid_node_ = rclcpp::Node::make_shared("longitudinal_pid_node");
        lateral_pid_node_ = rclcpp::Node::make_shared("lateral_pid_node");

        longitudinal_pid = std::shared_ptr<control_toolbox::PidROS>(new control_toolbox::PidROS(this->longitudinal_pid_node_, "longitudinal_pid_controller"));
        lateral_pid = std::shared_ptr<control_toolbox::PidROS>(new control_toolbox::PidROS(this->lateral_pid_node_, "lateral_pid_controller"));

        // TODO: read config from file
    }

    void PIDController::setTarget(const nav_msgs::msg::Path::SharedPtr trajectory, const float targetSpeed)
    {
        this->trajectory = trajectory;
        this->target_speed = target_speed;
        pid_last_calc_timestamp = this->parent->get_clock()->now();
    }

    ControlResult 
    PIDController::compute(const nav_msgs::msg::Odometry::SharedPtr odom,
                           std::mutex& odom_mutex,
                           const std::map<const std::string, boost::any> extra)
    {
        rclcpp::Duration dt = this->parent->get_clock()->now() - pid_last_calc_timestamp;
        ControlResult result;

        ackermann_msgs::msg::AckermannDrive msg;
        // find the next waypoint
        int next_waypoint_index = this->p_findNextWaypoint(this->trajectory, odom);

        // run pid controller 
        float throttle_error = this->p_calculateThrottleError(odom, this->target_speed);
        float steering_error = this->p_calculateSteeringError(odom, this->trajectory->poses[next_waypoint_index]);

        float current_spd = this->p_SpeedFromOdom(odom);
        this->p_updateLongAndLatPID(current_spd);

        float throttle_cmd = std::max(0.0f, std::min(1.0f, float(this->longitudinal_pid->computeCommand(throttle_error, dt))));
        float steering_cmd = std::max(MAX_LEFT_STEERING_ANGLE, 
                                      std::min(MAX_RIGHT_STEERING_ANGLE, 
                                      float(this->lateral_pid->computeCommand(steering_error, dt))));
        
        RCLCPP_INFO(rclcpp::get_logger("pid_controller"), "lateral pid:");
        this->printGains(this->lateral_pid->getGains());
        RCLCPP_INFO(rclcpp::get_logger("pid_controller"), "longitudinal pid:");
        this->printGains(this->lateral_pid->getGains());
        // output cmd 
        msg.jerk = throttle_cmd;
        msg.steering_angle = steering_cmd;
        result.drive = msg;
        result.waypoint_index = next_waypoint_index;

        // update dt for next computation
        pid_last_calc_timestamp = this->parent->get_clock()->now();
        return result;
    }

    float PIDController::p_calculateThrottleError(nav_msgs::msg::Odometry::SharedPtr odom, float target_spd)
    {
        float current_spd = this->p_SpeedFromOdom(odom);
        return target_spd - current_spd;
    }

    float PIDController::p_calculateSteeringError(nav_msgs::msg::Odometry::SharedPtr odom, geometry_msgs::msg::PoseStamped next_waypoint)
    {
        float tx = next_waypoint.pose.position.x;
        float ty = next_waypoint.pose.position.y;
        float desired_yaw = -1 * std::atan2(ty, tx);

        float steering_error = desired_yaw; // after mathmatical deduction, steering error = desired yaw
        return steering_error;
    }

    double PIDController::p_yawFromOdom(nav_msgs::msg::Odometry::SharedPtr odom)
    {
        tf2::Quaternion quat(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
                             odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
        tf2::Matrix3x3 mat(quat);
        double yaw = atan2(mat[1][0], mat[0][0]);
        return yaw;
    }

    float PIDController::p_SpeedFromOdom(nav_msgs::msg::Odometry::SharedPtr odom)
    {
        geometry_msgs::msg::Vector3 linear_velocities = odom->twist.twist.linear;
        double vx = linear_velocities.x;
        double vy = linear_velocities.y;
        double vz = linear_velocities.z;
        float current_spd = std::sqrt(vx * vx + vy * vy + vz * vz);
        return current_spd;
    }

    int PIDController::p_findNextWaypoint(const std::shared_ptr<nav_msgs::msg::Path> path, const std::shared_ptr<nav_msgs::msg::Odometry> odom)
    {
        // TODO: Better strategy to find the next immediate point to follow 
        return 0;
    }
    

    void PIDController::p_updateLongAndLatPID(float spd)
    {
        // control_toolbox::Pid::Gains long_gains = this->pGainsFromSpeed(spd, this->longitudinal_pid_configs);
        // control_toolbox::Pid::Gains lat_gains = this->pGainsFromSpeed(spd, this->lateral_pid_configs);

        // longitudinal_pid->setGains(long_gains);
        // lateral_pid->setGains(lat_gains);
    }

    control_toolbox::Pid::Gains PIDController::p_GainsFromSpeed(float spd, std::map<float, control_toolbox::Pid::Gains> *configs)
    {
        // Check if map is empty
        if (configs->empty())
        {
            RCLCPP_WARN(rclcpp::get_logger("pid_controller"), "Empty gains configuration, using default gains");
            return control_toolbox::Pid::Gains(DEFAULT_K_P, DEFAULT_K_I, DEFAULT_K_D, 0.0, 0.0, false);
        }

        float closest_speed = configs->begin()->first;
        control_toolbox::Pid::Gains gains = configs->begin()->second;
        for (const auto &pair : *configs)
        {
            if (std::abs(pair.first - spd) < std::abs(closest_speed - spd))
            {
                gains = pair.second;
                closest_speed = pair.first;
            }
        }
        return gains;
    }


} // controller