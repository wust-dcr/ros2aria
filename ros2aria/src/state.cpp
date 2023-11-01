#include "ros2aria/ros2aria.hpp"

void Ros2Aria::publishState()
{
    auto r = this->robot->getRobot();

    // TODO: original rosaria only publishes it when it changes.
    if (this->battery_recharge_state_pub_->get_subscription_count() > 0)
    {
        char s = r->getChargeState();
        std_msgs::msg::Int8 recharge_state;
        recharge_state.data = s;
        this->battery_recharge_state_pub_->publish(recharge_state);
    }


    if (robot_info_pub_->get_subscription_count() > 0)
    {
        ros2aria_msgs::msg::RobotInfoMsg robot_info_msg;

        robot_info_msg.robot_id.data =  id;
        robot_info_msg.battery_voltage.data = r->getRealBatteryVoltageNow();
        robot_info_msg.minimal_distance.data = minimal_distance;
        robot_info_msg.twist.linear.x = r->getVel() / 1000;
        robot_info_msg.twist.linear.y = r->getLatVel() / 1000.0;
        robot_info_msg.twist.angular.z = r->getRotVel() * M_PI / 180;
        robot_info_msg.state.data = not user_stop;
        robot_info_msg.clutch.data = r->areMotorsEnabled();
        robot_info_msg.obstacle_detected.data = obstacle_too_close;

        robot_info_pub_->publish(robot_info_msg);
    }
}