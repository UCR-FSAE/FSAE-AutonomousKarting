#include <nav_msgs/msg/odometry.hpp>
namespace local_planning
{
    class VehicleModelInterface
    {
    public:
        virtual void update(float throttle, float steering);
        virtual void setInitialState(const nav_msgs::msg::Odometry &initial_state);
    };
}