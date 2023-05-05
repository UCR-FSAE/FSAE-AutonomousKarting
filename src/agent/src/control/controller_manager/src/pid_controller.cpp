#include <controller_manager/pid_controller.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace controller 
{
    bool PIDController::configure(const std::map<std::string, boost::any> configuration, rclcpp_lifecycle::LifecycleNode *parent)
    {
        this->parent = parent;

        longitudinal_pid_configs = new std::map<float, control_toolbox::Pid::Gains>;
        lateral_pid_configs = new std::map<float, control_toolbox::Pid::Gains>;
        
        longitudinal_pid_node_ = rclcpp::Node::make_shared("longitudinal_pid_node");
        lateral_pid_node_ = rclcpp::Node::make_shared("lateral_pid_node");

        longitudinal_pid = std::shared_ptr<control_toolbox::PidROS>(new control_toolbox::PidROS(this->longitudinal_pid_node_, "longitudinal_pid_controller"));
        lateral_pid = std::shared_ptr<control_toolbox::PidROS>(new control_toolbox::PidROS(this->lateral_pid_node_, "lateral_pid_controller"));
        
        // TODO: this should be passed in from config dictionary
        bool pid_read_status = this->pReadPidFromFile("/home/michael/Desktop/projects/roar-gokart-ws/src/agent/src/control/controller_manager/params/carla_pid.json");
        if (!pid_read_status)
        {
            RCLCPP_ERROR(this->parent->get_logger(), "Error reading pid configuration");
            return false;
        }

        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->parent->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        return true;
    }

    void PIDController::setTarget(const nav_msgs::msg::Path::SharedPtr trajectory, const float target_speed)
    {
        this->trajectory = trajectory;
        this->targetSpeed = target_speed;
        pid_last_calc_timestamp = this->parent->get_clock()->now();
        // RCLCPP_INFO(rclcpp::get_logger("pid_controller"), "Target speed: [%.3f]", this->targetSpeed);
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

        geometry_msgs::msg::Pose::SharedPtr next_waypoint_ego_fov = this->pConvertToEgoFov(this->trajectory->poses[next_waypoint_index].pose, odom);
        if (next_waypoint_ego_fov == nullptr){
            RCLCPP_ERROR(rclcpp::get_logger("pid_controller"), "unable to get transform");
            return result;
        }

        // run pid controller 
        float throttle_error = this->p_calculateThrottleError(odom, this->targetSpeed);
        float steering_error = this->p_calculateSteeringError(odom, next_waypoint_ego_fov);

        float current_spd = this->p_SpeedFromOdom(odom);
        this->p_updateLongAndLatPID(current_spd);

        float throttle_cmd = std::max(0.0f, std::min(1.0f, float(this->longitudinal_pid->computeCommand(throttle_error, dt))));
        float steering_cmd = std::max(MAX_LEFT_STEERING_ANGLE, 
                                      std::min(MAX_RIGHT_STEERING_ANGLE, 
                                      float(this->lateral_pid->computeCommand(steering_error, dt))));
        this->p_printOdom(odom);
        this->p_printPose(std::make_shared<geometry_msgs::msg::Pose>(this->trajectory->poses[0].pose));
        RCLCPP_DEBUG(rclcpp::get_logger("pid_controller"), "spd: %.3f | steering_error: %3f | steering cmd: %3f | lateral pid:", current_spd, steering_error, steering_cmd);
        this->printGains(this->lateral_pid->getGains());
        // RCLCPP_DEBUG(rclcpp::get_logger("pid_controller"), "target spd: %.3f | spd: %.3f | longitudinal cmd: [%.3f] | pid:", this->targetSpeed, current_spd, throttle_cmd);
        // this->printGains(this->longitudinal_pid->getGains());

        RCLCPP_DEBUG(rclcpp::get_logger("pid_controller"), "------------");

        // output cmd 
        msg.jerk = throttle_cmd;

        msg.steering_angle = steering_cmd;
        msg.steering_angle_velocity = std::max(MAX_LEFT_STEERING_ANGLE, std::min(MAX_RIGHT_STEERING_ANGLE, steering_cmd));

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

    float PIDController::p_calculateSteeringError(nav_msgs::msg::Odometry::SharedPtr odom, 
                                                  geometry_msgs::msg::Pose::SharedPtr next_waypoint)
    {
        float tx = next_waypoint->position.x;
        float ty = next_waypoint->position.y;
        float desired_yaw = -1 * std::atan2(ty, tx);

        float steering_error = desired_yaw;
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
        control_toolbox::Pid::Gains long_gains = this->p_GainsFromSpeed(spd, this->longitudinal_pid_configs);
        control_toolbox::Pid::Gains lat_gains = this->p_GainsFromSpeed(spd, this->lateral_pid_configs);

        longitudinal_pid->setGains(long_gains);
        lateral_pid->setGains(lat_gains);
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

    /**
     * Reading from PID config file
    */
    bool
    PIDController::pReadPidFromFile(const std::string pid_config_file_path)
    {
        RCLCPP_INFO(rclcpp::get_logger("pid_controller"), "Loading PID configuration from: [%s]", pid_config_file_path.c_str());

        std::ifstream file(pid_config_file_path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(rclcpp::get_logger("pid_controller"), "Unable to open file %s", pid_config_file_path.c_str());
            std::cerr << "Failed to open file." << std::endl;
            return false;
        }

        // Parse the JSON data from the file
        rapidjson::IStreamWrapper isw(file);
        rapidjson::Document doc;
        doc.ParseStream(isw);
        if (doc.HasMember("longitudinal_controller"))
        {
            const rapidjson::Value &configs = doc["longitudinal_controller"];
            bool status = this->pLoadConfigs(configs, this->longitudinal_pid_configs);
            RCLCPP_INFO(rclcpp::get_logger("pid_controller"), "Longitudinal PIDs:");

            this->pPrettyPrintMap(this->longitudinal_pid_configs);
            if (!status)
            {
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("pid_controller"), "JSON does not container key [longitudinal_controller]");
            return false;
        }

        if (doc.HasMember("latitudinal_controller"))
        {
            const rapidjson::Value &configs = doc["latitudinal_controller"];
            bool status = this->pLoadConfigs(configs, this->lateral_pid_configs);
            RCLCPP_INFO(rclcpp::get_logger("pid_controller"), "Lateral PIDs:");

            this->pPrettyPrintMap(this->lateral_pid_configs);
            if (!status)
            {
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("pid_controller"), "JSON does not container key [latitudinal_controller]");
            return false;
        }
        return true;
    }

    bool 
    PIDController::pLoadConfigs(const rapidjson::Value &config, std::map<float, control_toolbox::Pid::Gains> *configs)
    {
        if (!config.IsObject())
        {
            return false;
        }

        for (auto itr = config.MemberBegin(); itr != config.MemberEnd(); ++itr)
        {
            std::string key = itr->name.GetString();
            float f = std::stof(key);

            const rapidjson::Value &pid_config = itr->value;
            (*configs)[f] = this->pConvertJsonToPid(pid_config);
        }

        return true;
    }

    control_toolbox::Pid::Gains
    PIDController::pConvertJsonToPid(const rapidjson::Value &config)
    {
        control_toolbox::Pid::Gains gains(0, 0, 0, 0, 0, false);
        if (!config.IsObject())
        {
            return gains;
        }
        if (config.HasMember("k_p") && config["k_p"].IsNumber())
        {
            gains.p_gain_ = config["k_p"].GetFloat();
        }
        if (config.HasMember("k_i") && config["k_i"].IsNumber())
        {
            gains.i_gain_ = config["k_i"].GetFloat();
        }
        if (config.HasMember("k_d") && config["k_d"].IsNumber())
        {
            gains.d_gain_ = config["k_d"].GetFloat();
        }
        if (config.HasMember("i_clamp_min") && config["i_clamp_min"].IsNumber())
        {
            gains.i_min_ = config["i_clamp_min"].GetFloat();
        }
        if (config.HasMember("i_clamp_max") && config["i_clamp_max"].IsNumber())
        {
            gains.i_max_ = config["i_clamp_max"].GetFloat();
        }
        if (config.HasMember("antiwindup") && config["antiwindup"].IsBool())
        {
            gains.antiwindup_ = config["antiwindup"].GetBool();
        }
        return gains;
    }

    void 
    PIDController::pPrettyPrintMap(const std::map<float, control_toolbox::Pid::Gains> *configs)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2); // set precision to 2 decimal places
        oss << "\n";
        for (const auto &pair : *configs)
        {
            oss << "  Below " << pair.first << " m/s"
                << ": { p: " << pair.second.p_gain_
                << ", i: " << pair.second.i_gain_
                << ", d: " << pair.second.d_gain_
                << ", i_min_: " << pair.second.i_min_
                << ", i_max_: " << pair.second.i_max_
                << " }\n";
        }
        RCLCPP_INFO(rclcpp::get_logger("pid_controller"), oss.str());
    }

    geometry_msgs::msg::Pose::SharedPtr PIDController::pConvertToEgoFov(geometry_msgs::msg::Pose original, nav_msgs::msg::Odometry::SharedPtr latest_odom)
    {
        geometry_msgs::msg::TransformStamped t;
        std::string fromFrameRel = latest_odom->header.frame_id;
        std::string toFrameRel = latest_odom->child_frame_id;
        try
        {
          t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
          RCLCPP_INFO(
              rclcpp::get_logger("pid_controller"), "Could not transform %s to %s: %s",
              toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return nullptr;
        }
        std_msgs::msg::Header header;
        header.frame_id = toFrameRel;
        header.stamp = this->parent->get_clock()->now();
        geometry_msgs::msg::PoseStamped child_pose;
        geometry_msgs::msg::PoseStamped ps;
        ps.header = header;
        ps.pose = original;
        tf2::doTransform(ps, child_pose, t);
        return std::make_shared<geometry_msgs::msg::Pose>(child_pose.pose);
    }



} // controller