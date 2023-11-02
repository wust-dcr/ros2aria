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

#include <algorithm>
#include <limits>
#include "ros2aria/ros2aria.hpp"

void Ros2Aria::scan_callback(const sensor_msgs::msg::LaserScan msg) {
    float actual_minimal_distance = std::numeric_limits<float>::max();
    for (const auto &range : msg.ranges) {
        if (!std::isnan(range) && range > msg.range_min && range < msg.range_max) {
            actual_minimal_distance = std::min(actual_minimal_distance, range);
        }
    }
    minimal_distance = actual_minimal_distance;
    if (actual_minimal_distance < restrictions.distance.data) {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "obstacle detected, minimal_distance: " << actual_minimal_distance);
        obstacle_too_close = true;
    } else {
        obstacle_too_close = false;
    }
}
