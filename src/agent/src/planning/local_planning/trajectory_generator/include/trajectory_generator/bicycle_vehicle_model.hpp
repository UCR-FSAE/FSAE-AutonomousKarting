#ifndef BICYCLE_VEHICLE_MODEL_HPP_
#define BICYCLE_VEHICLE_MODEL_HPP_

#include "trajectory_generator/vehicle_model_interface.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <math.h>

namespace local_planning
{
    class BicycleVehicleModel : public VehicleModelInterface
    {
    public:
        BicycleVehicleModel(float wheelbase, float max_steering_angle);

        void update(float velocity, float steering, float dt) override;
        void setInitialState(const nav_msgs::msg::Odometry &initial_state) override;

    private:
        float wheelbase_;
        float max_steering_angle_;

        nav_msgs::msg::Odometry state_;
    };

    BicycleVehicleModel::BicycleVehicleModel(float wheelbase, float max_steering_angle) : wheelbase_(wheelbase),
                                                                                            max_steering_angle_(max_steering_angle)
    {
    }

    void BicycleVehicleModel::update(float velocity, float steering, float dt)
    {
        // TODO implement this
        // // Compute new heading based on current heading, steering angle, and time step
        // float heading = state_.pose.pose.orientation.z;
        // heading += dt * velocity * tan(steering) / wheelbase_;

        // // Limit heading to [-pi, pi] range
        // if (heading > M_PI)
        // {
        //     heading -= 2.0 * M_PI;
        // }
        // else if (heading < -M_PI)
        // {
        //     heading += 2.0 * M_PI;
        // }

        // // Update state with new heading
        // state_.pose.pose.orientation.z = heading;
    }

    void BicycleVehicleModel::setInitialState(const nav_msgs::msg::Odometry &initial_state)
    {
        state_ = initial_state;
    }
}

#endif // BICYCLE_VEHICLE_MODEL_HPP_
