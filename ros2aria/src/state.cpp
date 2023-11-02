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

void Ros2Aria::publishState() {
    auto r = this->robot->getRobot();

    if (this->battery_recharge_state_pub_->get_subscription_count() > 0) {
        char s = r->getChargeState();
        std_msgs::msg::Int8 recharge_state;
        recharge_state.data = s;
        this->battery_recharge_state_pub_->publish(recharge_state);
    }

    if (robot_info_pub_->get_subscription_count() > 0) {
        ros2aria_msgs::msg::RobotInfoMsg robot_info_msg;

        robot_info_msg.robot_id.data = id;
        robot_info_msg.battery_voltage.data = r->getRealBatteryVoltageNow();
        robot_info_msg.minimal_distance.data = minimal_distance;
        robot_info_msg.twist.linear.x = r->getVel() / 1000;
        robot_info_msg.twist.linear.y = r->getLatVel() / 1000.0;
        robot_info_msg.twist.angular.z = r->getRotVel() * M_PI / 180;
        robot_info_msg.state.data = !user_stop;
        robot_info_msg.clutch.data = r->areMotorsEnabled();
        robot_info_msg.obstacle_detected.data = obstacle_too_close;

        robot_info_pub_->publish(robot_info_msg);
    }
}
