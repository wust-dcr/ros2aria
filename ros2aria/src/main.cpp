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

#include <cstdio>
#include <memory>

#include "ros2aria/ros2aria.hpp"

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    printf("hello world ros2aria package\n");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ros2Aria>());
    rclcpp::shutdown();

    return 0;
}
