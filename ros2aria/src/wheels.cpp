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

sensor_msgs::msg::JointState Ros2Aria::handleWheels(rclcpp::Time stamp) {
    sensor_msgs::msg::JointState wheels;

    auto r = this->robot->getRobot();

    r->requestEncoderPackets();
    wheels.header.stamp = stamp;
    wheels.name.resize(2);
    wheels.position.resize(2);
    wheels.velocity.resize(2);
    wheels.effort.resize(0);

    // robot_state_publisher gives namespace
    wheels.name[0] = "left_wheel_joint";
    wheels.name[1] = "right_wheel_joint";

    wheels.position[0] = encoder_to_rad(r->getLeftEncoder());
    wheels.position[1] = encoder_to_rad(r->getRightEncoder());
    wheels.velocity[0] = mm_per_sec_to_rad_per_sec(r->getLeftVel());
    wheels.velocity[1] = mm_per_sec_to_rad_per_sec(r->getRightVel());
    return wheels;
}

void Ros2Aria::publishWheels(sensor_msgs::msg::JointState wheels) {
    if (this->wheels_pub_->get_subscription_count() == 0) return;

    this->wheels_pub_->publish(wheels);
}
