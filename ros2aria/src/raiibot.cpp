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

#include "ros2aria/raiibot.hpp"

RAIIBot::RAIIBot(rclcpp::Node *node, std::string port) {
    this->robot = new ArRobot();
    this->gripper = new ArGripper(this->robot);
    this->node = node;
    this->clock = rclcpp::Clock::make_shared();

    args = new ArArgumentBuilder();
    this->argparser = new ArArgumentParser(args);
    argparser->loadDefaultArguments();
    std::string port_arg = "-robotPort " + port;
    args->add(port_arg.c_str());

    this->robotConn = new ArRobotConnector(argparser, robot);
    if (!this->robotConn->connectRobot()) {
        RCLCPP_INFO_STREAK(this->get_logger(), "RosAria: ARIA could not connect to robot!"
                                                   << "(Check ~port parameter is correct, and "
                                                      "permissions on port device,"
                                                   << "or any errors reported above)");
    }

    this->robot->runAsync(true);
    this->robot->enableMotors();

    this->startWatchdog();
}

RAIIBot ::~RAIIBot() {
    std::cout << std::endl << "RAIIBOT DESTRUCTOR RUNNING!" << std::endl;

    if (this->robot != nullptr) {
        this->robot->lock();
        std::cout << "disabling motors" << std::endl;
        this->robot->disableMotors();
        std::cout << "disabled motors" << std::endl;
        Aria::shutdown();
    }
    if (this->robotConn != nullptr) {
        std::cout << "disconnecting" << std::endl;
        this->robotConn->disconnectAll();
        std::cout << "disconnected" << std::endl;
    }

    if (this->args != nullptr) delete this->args;
    if (this->argparser != nullptr) delete this->argparser;
    if (this->robotConn != nullptr) delete this->robotConn;
    if (this->robot != nullptr) delete this->robot;
}

ArRobot *RAIIBot::getRobot() { return this->robot; }

ArGripper *RAIIBot::getGripper() { return this->gripper; }

rclcpp::Clock::SharedPtr RAIIBot::getClock() { return this->clock; }

void RAIIBot::lock() {
    if (this->robot != nullptr) this->robot->lock();
}

void RAIIBot::unlock() {
    if (this->robot != nullptr) this->robot->unlock();
}

void RAIIBot::pokeWatchdog() {
    // this should probably be made thread safe, or something?
    this->lastWatchdogPoke = this->clock->now();
}

rclcpp::Logger RAIIBot::get_logger() { return this->node->get_logger(); }

void RAIIBot::startWatchdog() {
    using namespace std::chrono_literals;
    this->lastWatchdogPoke = clock->now();
    watchdogTimer = this->node->create_wall_timer(0.1s, std::bind(&RAIIBot::watchdog_cb, this));
}

void RAIIBot::watchdog_cb() {
    using namespace std::chrono_literals;

    auto now = this->clock->now();
    if (now - this->lastWatchdogPoke > 1s) {
        std::lock_guard<RAIIBot> lock(*this);
        auto r = this->getRobot();
        r->setVel(0);
        r->setRotVel(0);
        if (r->hasLatVel()) r->setLatVel(0);
    }
}
