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
#include "sensor_msgs/point_cloud_conversion.hpp"

sensor_msgs::msg::PointCloud Ros2Aria::handleSonar(rclcpp::Time stamp) {
    sensor_msgs::msg::PointCloud cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = ros_namespace + "/front_sonar";

    auto r = robot->getRobot();

    auto lenght = r->getNumSonar() / 2;
    for (int i = 0; i < num_of_sonars; i++) {
        ArSensorReading *reading = NULL;
        reading = r->getSonarReading(i);
        if (!reading) {
            RCLCPP_INFO(this->get_logger(), "did not receive a sonar reading.");
            continue;
        }

        geometry_msgs::msg::Point32 p;
        p.x = reading->getLocalX() / 1000.0;
        p.y = reading->getLocalY() / 1000.0;
        p.z = 0.0;
        cloud.points.push_back(p);

        if (distance(p.x, p.y) < minimal_distance) {
            obstacle_too_close = true;
        }
    }
    return cloud;
}

void Ros2Aria::publishSonar(sensor_msgs::msg::PointCloud cloud) {
    if (this->sonar_pub_->get_subscription_count() == 0) return;

    this->sonar_pub_->publish(cloud);
}

void Ros2Aria::publishSonarPointCloud2(sensor_msgs::msg::PointCloud cloud) {
    if (this->sonar_pointcloud2_pub_->get_subscription_count() == 0) return;

    sensor_msgs::msg::PointCloud2 cloud2;
    sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);

    this->sonar_pointcloud2_pub_->publish(cloud2);
}
