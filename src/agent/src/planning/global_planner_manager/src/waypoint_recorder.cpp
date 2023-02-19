// Copyright 2023 michael. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include "global_planner_manager/waypoint_recorder.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <iostream>
#include <fstream>
#include <string.h>
#include <filesystem>
#include <experimental/filesystem>
#include <cstdlib>
using std::placeholders::_1;

namespace fs = std::experimental::filesystem;

namespace gokart_planner
{
    WaypointRecorder::WaypointRecorder() : Node("waypoint_recorder")
    {
        this->declare_parameter("output_file_path", "./data/recording.txt");
        this->declare_parameter("odom_topic", "/carla/ego_vehicle/odometry");
        this->declare_parameter("record_interval", 1.0);

        // setting subscription
        std::string odom_topic =
            this->get_parameter("odom_topic").get_parameter_value().get<std::string>();
        RCLCPP_INFO(get_logger(), "odom_topic: %s", odom_topic.c_str());
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, std::bind(&WaypointRecorder::topic_callback, this, _1));

        // open file
        std::string output_file_path =
        this->get_parameter("output_file_path").get_parameter_value().get<std::string>();
        RCLCPP_INFO(get_logger(), "output_file_path: %s", output_file_path.c_str());
        fs::path p(output_file_path.c_str());
        fs::path dir = p.parent_path();
        bool status = fs::create_directories(dir);
        this->outputFile.open(p.c_str());

        // setting rate
        this->record_interval_nanoseconds = int(this->get_parameter("record_interval").as_double()*1e9);
        RCLCPP_INFO(get_logger(), "record_interval: %fs", this->get_parameter("record_interval").as_double());
        this->prev_time = this->now();
    }

    WaypointRecorder::~WaypointRecorder()
    {
        RCLCPP_INFO(get_logger(), "Shutting down");
        this->outputFile.close();
    }

    void WaypointRecorder::topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {   
        auto time_diff = (this->now() - this->prev_time).nanoseconds();
        if (time_diff > this->record_interval_nanoseconds)
        {
            std::string msg_to_file = this->msg_to_String(msg) + "\n";
            this->outputFile << msg_to_file;
            this->prev_time = this->now();
        }
    }

    std::string WaypointRecorder::msg_to_String(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::string output("");
        output += to_string(msg->pose.pose.position.x);
        output += ",";

        output += to_string(msg->pose.pose.position.y);
        output += ",";

        output += to_string(msg->pose.pose.position.z);
        return output;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gokart_planner::WaypointRecorder>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
