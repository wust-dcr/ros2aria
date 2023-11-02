// Copyright 2023 WUST Department of Cybernetics and Robotics
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
// USA.

#include <cmath>
#include "ros2aria/ros2aria.hpp"

void Ros2Aria::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    const float limit_linear = restrictions.linear_velocity.data;
    const float limit_angular = restrictions.angular_velocity.data;

    float x, y, z;
    x = msg->linear.x;
    y = msg->linear.y;
    z = msg->angular.z;
    RCLCPP_INFO(this->get_logger(), "cmd_vel: x:%f y:%f z:%f", x, y, z);

    std::lock_guard<RAIIBot> lock(*robot.get());

    this->robot->pokeWatchdog();

    auto r = robot->getRobot();

    if (use_safety_system) {
        if (master_stop || obstacle_too_close || user_stop) {
            x = 0.0;
            y = 0.0;
            z = 0.0;
        } else {
            // apply limits
            x = std::abs(x) > limit_linear ? std::abs(x) / x * limit_linear : x;
            y = std::abs(y) > limit_linear ? std::abs(y) / y * limit_linear : y;
            z = std::abs(z) > limit_angular ? std::abs(z) / z * limit_angular : z;
        }
    }
    x *= 1e3;
    y *= 1e3;
    z *= 180 / M_PI;
    r->setVel(x);
    if (r->hasLatVel()) r->setLatVel(y);
    r->setRotVel(z);
}
