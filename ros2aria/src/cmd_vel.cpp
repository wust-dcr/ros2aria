#include "ros2aria/ros2aria.hpp"
#include <cmath>

void Ros2Aria::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    float x, y, z;
    x = msg->linear.x;
    y = msg->linear.y;
    z = msg->angular.z;
    RCLCPP_INFO(this->get_logger(), "cmd_vel: x:%f y:%f z:%f", x, y, z);

    std::lock_guard<RAIIBot> lock(*robot.get());

    this->robot->pokeWatchdog();

    auto r = robot->getRobot();

    if(use_safety_system){
        if (master_stop or obstacle_too_close or user_stop){
            x = 0.0;
            y = 0.0;
            z = 0.0;
        }
        else{
            // apply limits
            x = std::abs(x) > restrictions.linear_velocity.data ? std::abs(x)/x * restrictions.linear_velocity.data : x;
            y = std::abs(y) > restrictions.linear_velocity.data ? std::abs(y)/y * restrictions.linear_velocity.data : y;
            z = std::abs(z) > restrictions.angular_velocity.data ?  std::abs(z)/z * restrictions.angular_velocity.data : z;
        }
    }
    x *= 1e3;
    y *= 1e3;
    z *= 180 / M_PI;
    r->setVel(x);
    if (r->hasLatVel())
        r->setLatVel(y);
    r->setRotVel(z);
}
