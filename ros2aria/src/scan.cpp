#include "ros2aria/ros2aria.hpp"
#include <algorithm>
#include <limits>

void Ros2Aria::scan_callback(const sensor_msgs::msg::LaserScan msg){
    float actual_minimal_distance = std::numeric_limits<float>::max();
    for(const auto &range: msg.ranges){
        if(not std::isnan(range) and range > msg.range_min and range < msg.range_max){
            actual_minimal_distance = std::min(actual_minimal_distance, range);
        }
    }
    minimal_distance = actual_minimal_distance;
    if(actual_minimal_distance < restrictions.distance.data){
        RCLCPP_INFO(this->get_logger(), "obstacle detected, minimal_distance: %f", actual_minimal_distance);
        obstacle_too_close = true;
    }else{
        obstacle_too_close = false;
    }
}
