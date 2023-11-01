#include "ros2aria/ros2aria.hpp"
#include <cmath>
sensor_msgs::msg::JointState Ros2Aria::handleWheels(rclcpp::Time stamp)
{
    sensor_msgs::msg::JointState wheels;

    auto r = this->robot->getRobot();

    r->requestEncoderPackets(); // TODO: check if this is synchronous or do we have a race condition or something
    wheels.header.stamp = stamp;
    wheels.name.resize(2);
    wheels.position.resize(2);
    wheels.velocity.resize(2);
    wheels.effort.resize(0);

    // robot_state_publisher gives namespace
    wheels.name[0] = "left_wheel_joint";
    wheels.name[1] =  "right_wheel_joint";

    wheels.position[0] = encoder_to_rad(r->getLeftEncoder());
    wheels.position[1] = encoder_to_rad(r->getRightEncoder());
    wheels.velocity[0] = mm_per_sec_to_rad_per_sec(r->getLeftVel());
    wheels.velocity[1] = mm_per_sec_to_rad_per_sec(r->getRightVel());
    return wheels;
}

void Ros2Aria::publishWheels(sensor_msgs::msg::JointState wheels)
{
    if (this->wheels_pub_->get_subscription_count() == 0)
        return;

    this->wheels_pub_->publish(wheels);
}