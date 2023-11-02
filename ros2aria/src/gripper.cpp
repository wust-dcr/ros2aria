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

#include "ros2aria/ros2aria.hpp"

void Ros2Aria::gripper_open_callback(const std_srvs::srv::Empty::Request::SharedPtr request,
                                     std_srvs::srv::Empty::Response::SharedPtr response) const {
    UNUSED(request);
    UNUSED(response);
    std::lock_guard<RAIIBot> lock(*robot.get());
    auto g = robot->getGripper();
    g->gripOpen();
}

void Ros2Aria::gripper_close_callback(const std_srvs::srv::Empty::Request::SharedPtr request,
                                      std_srvs::srv::Empty::Response::SharedPtr response) const {
    UNUSED(request);
    UNUSED(response);
    std::lock_guard<RAIIBot> lock(*robot.get());
    auto g = robot->getGripper();
    g->gripClose();
}

void Ros2Aria::gripper_up_callback(const std_srvs::srv::Empty::Request::SharedPtr request,
                                   std_srvs::srv::Empty::Response::SharedPtr response) const {
    UNUSED(request);
    UNUSED(response);
    std::lock_guard<RAIIBot> lock(*robot.get());
    auto g = robot->getGripper();
    g->liftUp();
}

void Ros2Aria::gripper_down_callback(const std_srvs::srv::Empty::Request::SharedPtr request,
                                     std_srvs::srv::Empty::Response::SharedPtr response) const {
    UNUSED(request);
    UNUSED(response);
    std::lock_guard<RAIIBot> lock(*robot.get());
    auto g = robot->getGripper();
    g->liftDown();
}
