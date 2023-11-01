#include "ros2aria/ros2aria.hpp"

void Ros2Aria::gripper_open_callback(const std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response) const {
    UNUSED(request);
    UNUSED(response);
    std::lock_guard<RAIIBot> lock(*robot.get());
    auto g = robot->getGripper();
    g->gripOpen();
}

void Ros2Aria::gripper_close_callback(const std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response) const {
    UNUSED(request);
    UNUSED(response);
    std::lock_guard<RAIIBot> lock(*robot.get());
    auto g = robot->getGripper();
    g->gripClose();
}

void Ros2Aria::gripper_up_callback(const std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response) const {
    UNUSED(request);
    UNUSED(response);
    std::lock_guard<RAIIBot> lock(*robot.get());
    auto g = robot->getGripper();
    g->liftUp();
}

void Ros2Aria::gripper_down_callback(const std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response) const {
    UNUSED(request);
    UNUSED(response);
    std::lock_guard<RAIIBot> lock(*robot.get());
    auto g = robot->getGripper();
    g->liftDown();
}
