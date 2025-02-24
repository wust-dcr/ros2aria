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

std::pair<nav_msgs::msg::Odometry, geometry_msgs::msg::TransformStamped> Ros2Aria::handlePose(
    rclcpp::Time stamp) {
    nav_msgs::msg::Odometry odom_msg;

    auto r = robot->getRobot();
    ArPose p = r->getPose();

    tf2::Quaternion rotation;
    auto position = tf2::Vector3(p.getX() / 1000, p.getY() / 1000, 0);
    rotation.setRPY(0, 0, p.getTh() * M_PI / 180);

    odom_msg.pose.pose.orientation = tf2::toMsg(rotation);
    odom_msg.pose.pose.position.x = position.getX();
    odom_msg.pose.pose.position.y = position.getY();
    odom_msg.pose.pose.position.z = position.getZ();

    odom_msg.twist.twist.linear.x = r->getVel() / 1000;
    odom_msg.twist.twist.linear.y = r->getLatVel() / 1000;
    odom_msg.twist.twist.angular.z = r->getRotVel() * M_PI / 180;

    odom_msg.header.frame_id = ros_namespace + "/odom";
    odom_msg.child_frame_id = ros_namespace + "/base_link";
    odom_msg.header.stamp = stamp;

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom_msg.header;
    transform.child_frame_id = odom_msg.child_frame_id;
    transform.transform.translation.x = odom_msg.pose.pose.position.x;
    transform.transform.translation.y = odom_msg.pose.pose.position.y;
    transform.transform.translation.z = odom_msg.pose.pose.position.z;
    transform.transform.rotation = odom_msg.pose.pose.orientation;

    std::pair<nav_msgs::msg::Odometry, geometry_msgs::msg::TransformStamped> pair_msgs = {
        odom_msg, transform};
    return pair_msgs;
}

void Ros2Aria::publishPose(nav_msgs::msg::Odometry pose) {
    if (this->pose_pub_->get_subscription_count() == 0) return;

    this->pose_pub_->publish(pose);
}

void Ros2Aria::publishTf(geometry_msgs::msg::TransformStamped tf) {
    odom_tf_broadcaster_->sendTransform(tf);
}
