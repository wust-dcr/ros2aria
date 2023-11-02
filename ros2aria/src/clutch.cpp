// Copyright 2023 WUST Department of Cybernetics and Robotics, GPL2
#include "ros2aria/ros2aria.hpp"

void Ros2Aria::clutch_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<RAIIBot> lock(*robot.get());

    auto r = robot->getRobot();
    if (msg->data)
        r->enableMotors();
    else
        r->disableMotors();
}
