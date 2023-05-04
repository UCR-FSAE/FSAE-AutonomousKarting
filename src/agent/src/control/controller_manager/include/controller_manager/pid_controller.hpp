#include "controller_manager/controller_interface.hpp"
#include <control_toolbox/pid_ros.hpp>
#include <rclcpp/clock.hpp>
#include <iostream>
#include <fstream>
#include <iomanip> // for std::setprecision
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"

namespace controller 
{
    class PIDController : public ControllerInterface
    {   
        public:
            static constexpr float DEFAULT_K_P = 0.05f;
            static constexpr float DEFAULT_K_D = 0.0f;
            static constexpr float DEFAULT_K_I = 0.0f;
            
            bool configure(const std::map<std::string, boost::any> configuration, rclcpp_lifecycle::LifecycleNode *parent);
            void setTarget(const nav_msgs::msg::Path::SharedPtr trajectory, const float target_speed)  override;
            ControlResult compute(const nav_msgs::msg::Odometry::SharedPtr odom, 
                                std::mutex& odom_mutex,
                                const std::map<const std::string, boost::any> extra) override;
        private:
            std::shared_ptr<nav_msgs::msg::Path> trajectory;
            float targetSpeed = 1.0;
            float MAX_RIGHT_STEERING_ANGLE = 1.0;
            float MAX_LEFT_STEERING_ANGLE = -1.0;
            rclcpp::Time pid_last_calc_timestamp;

            std::shared_ptr<control_toolbox::PidROS> longitudinal_pid;
            std::shared_ptr<rclcpp::Node> longitudinal_pid_node_;

            std::shared_ptr<control_toolbox::PidROS> lateral_pid;
            std::shared_ptr<rclcpp::Node> lateral_pid_node_;

            std::map<float, control_toolbox::Pid::Gains> *longitudinal_pid_configs;
            std::map<float, control_toolbox::Pid::Gains> *lateral_pid_configs;


            float p_calculateThrottleError(nav_msgs::msg::Odometry::SharedPtr odom, float target_spd);
            float p_calculateSteeringError(nav_msgs::msg::Odometry::SharedPtr odom, geometry_msgs::msg::PoseStamped next_waypoint);
            double p_yawFromOdom(nav_msgs::msg::Odometry::SharedPtr odom);
            float p_SpeedFromOdom(nav_msgs::msg::Odometry::SharedPtr odom);
            int p_findNextWaypoint(const std::shared_ptr<nav_msgs::msg::Path>, const std::shared_ptr<nav_msgs::msg::Odometry> odom);
            void p_updateLongAndLatPID(float spd);

            control_toolbox::Pid::Gains p_GainsFromSpeed(float spd, std::map<float, control_toolbox::Pid::Gains> *configs);


            void
            printGains(const control_toolbox::Pid::Gains &gains)
            {
                RCLCPP_DEBUG(rclcpp::get_logger("pid_controller"), "Gains: P=%.2f, I=%.2f, D=%.2f, i_max=%.2f, i_min=%.2f, antiwindup=%d",
                            gains.p_gain_, gains.i_gain_, gains.d_gain_, gains.i_max_, gains.i_min_, gains.antiwindup_);
            }

            bool pReadPidFromFile(const std::string pid_config_file_path);
            bool pLoadConfigs(const rapidjson::Value &config, std::map<float, control_toolbox::Pid::Gains> *configs);
            void pPrettyPrintMap(const std::map<float, control_toolbox::Pid::Gains> *configs);
            control_toolbox::Pid::Gains pConvertJsonToPid(const rapidjson::Value &config);

    };
}