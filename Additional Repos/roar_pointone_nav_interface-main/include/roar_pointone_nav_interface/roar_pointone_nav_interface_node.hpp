#ifndef POINT_ONE_NAV_INTERFACE_NODE_HPP
#define POINT_ONE_NAV_INTERFACE_NODE_HPP
#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include "rclcpp/rclcpp.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_ros/transform_broadcaster.h>

namespace ROAR
{
    namespace PointoneNavInterface
    {
        struct GeodeticPosition
        {
            double latitude;
            double longitude;
            double altitude;
        };
        struct CartesianPosition
        {
            double x;
            double y;
            double z;
        };
        class PointoneNavInterfaceNode : public rclcpp::Node
        {
        public:
            PointoneNavInterfaceNode();
            ~PointoneNavInterfaceNode();

        private:
            void parse_datum();
            void gps_callback(const gps_msgs::msg::GPSFix::SharedPtr msg);
            void convert_gnss_to_local_cartesian(GeodeticPosition inputGeoPosition, CartesianPosition &outputCartesianPosition);
            void publishTransformFromOdom(const nav_msgs::msg::Odometry::SharedPtr odom);

            GeodeticPosition map_origin_;
            GeographicLib::LocalCartesian proj;

            // subscribers
            rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr gps_subscriber_;

            // publishers
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

            std::unique_ptr<tf2_ros::TransformBroadcaster>
                tf_broadcaster_;
        };
    } // namespace PointoneNavInterface
}

#endif // POINT_ONE_NAV_INTERFACE_NODE_HPP