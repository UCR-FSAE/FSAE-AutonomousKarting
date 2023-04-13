#ifndef VEHICLE_MODEL_INTERFACE_HPP_
#define VEHICLE_MODEL_INTERFACE_HPP_

#include <nav_msgs/msg/odometry.hpp>

namespace local_planning
{
    class VehicleModelInterface
    {
    public:
        virtual void update(float velocity, float steering, float dt) = 0;
        virtual void setInitialState(const nav_msgs::msg::Odometry &initial_state) = 0;
    };
}

#endif // VEHICLE_MODEL_INTERFACE_HPP_
