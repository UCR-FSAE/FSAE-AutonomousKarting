// Copyright 2023 michael. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#ifndef GOKART_WAYPOINT_RECORDER_HPP_
#define GOKART_WAYPOINT_RECORDER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
using namespace std;

namespace gokart_planner
{
    class WaypointRecorder : public rclcpp::Node
    {
        public:
            /**
             * @brief A constructor for gokart_planner::WaypointRecorder class
             */
            WaypointRecorder();
            /**
             * @brief A destructor for gokart_planner::WaypointRecorder class
             */
            ~WaypointRecorder();
        
        protected:
            void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
            std::string msg_to_String(const nav_msgs::msg::Odometry::SharedPtr msg);
            int record_interval_nanoseconds;
            rclcpp::Time prev_time;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
            ofstream outputFile;
    };
}
#endif // GOKART_WAYPOINT_RECORDER_HPP_
