#include "ros2aria/ros2aria.hpp"
#include <limits>
void Ros2Aria::restrictions_callback(const ros2aria_msgs::msg::RestrictionsMsg::SharedPtr msg){
    restrictions = *msg;
    RCLCPP_INFO(this->get_logger(), "restrictions: x:%f y:%f z:%f", restrictions.linear_velocity.data, restrictions.angular_velocity.data, 
                                                                 restrictions.distance.data);
}

void Ros2Aria::init_restrictions(){
    restrictions.linear_velocity.data = 0.0;
    restrictions.angular_velocity.data = 0.0;
    restrictions.distance.data = std::numeric_limits<float>::max();
}

void Ros2Aria::user_stop_callback(const std_msgs::msg::Bool::SharedPtr msg){
    user_stop = msg->data;
}

void Ros2Aria::master_stop_callback(const std_msgs::msg::Bool::SharedPtr msg){
    master_stop = msg->data;
}