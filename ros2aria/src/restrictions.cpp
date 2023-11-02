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

#include <limits>
#include "ros2aria/ros2aria.hpp"

void Ros2Aria::restrictions_callback(const ros2aria_msgs::msg::RestrictionsMsg::SharedPtr msg) {
    restrictions = *msg;
}

void Ros2Aria::init_restrictions() {
    restrictions.linear_velocity.data = 0.0;
    restrictions.angular_velocity.data = 0.0;
    restrictions.distance.data = std::numeric_limits<float>::max();
}

void Ros2Aria::user_stop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    user_stop = msg->data;
}

void Ros2Aria::master_stop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    master_stop = msg->data;
}
