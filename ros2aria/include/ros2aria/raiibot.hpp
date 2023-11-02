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

#ifndef ROS2ARIA__RAIIBOT_HPP_
#define ROS2ARIA__RAIIBOT_HPP_

#include <iostream>
#include <memory>
#include <string>

#include "Aria/Aria.h"
#include "rclcpp/rclcpp.hpp"

class RAIIBot {
   public:
    typedef std::shared_ptr<RAIIBot> SharedPtr;

    RAIIBot(rclcpp::Node *node, std::string port);
    ~RAIIBot();
    ArRobot *getRobot();
    ArGripper *getGripper();
    rclcpp::Clock::SharedPtr getClock();
    void lock();
    void unlock();
    void pokeWatchdog();

   private:
    ArRobot *robot = nullptr;
    ArRobotConnector *robotConn = nullptr;
    ArArgumentBuilder *args = nullptr;
    ArArgumentParser *argparser = nullptr;
    ArGripper *gripper = nullptr;
    rclcpp::Node *node = nullptr;
    rclcpp::TimerBase::SharedPtr watchdogTimer;
    rclcpp::Time lastWatchdogPoke;
    rclcpp::Clock::SharedPtr clock;

    rclcpp::Logger get_logger();
    void startWatchdog();
    void watchdog_cb();
};

#endif  // ROS2ARIA__RAIIBOT_HPP_
