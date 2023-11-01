#include "ros2aria/ros2aria.hpp"

void Ros2Aria::publish()
{
    rclcpp::Time t = robot->getClock()->now();

    if(use_sonar){
        sensor_msgs::msg::PointCloud sonarData = handleSonar(t);
        publishSonar(sonarData);
        publishSonarPointCloud2(sonarData);
    }

    auto pose = handlePose(t);
    publishPose(pose.first);
    publishTf(pose.second);

    sensor_msgs::msg::JointState wheels = handleWheels(t);
    publishWheels(wheels);

    publishState();
}