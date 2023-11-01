#include "ros2aria/ros2aria.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"

sensor_msgs::msg::PointCloud Ros2Aria::handleSonar(rclcpp::Time stamp)
{
    sensor_msgs::msg::PointCloud cloud; 
    cloud.header.stamp = stamp;
    cloud.header.frame_id = ros_namespace + "/front_sonar";

    auto r = robot->getRobot();

    auto lenght = r->getNumSonar()/2;
    for (int i = 0; i < num_of_sonars; i++)
    {
        ArSensorReading *reading = NULL;
        reading = r->getSonarReading(i);
        if (!reading)
        {
            RCLCPP_INFO(this->get_logger(), "did not receive a sonar reading.");
            continue;
        }

        // getRange() will return an integer between 0 and 5000 (5m)
        // local (x,y). Appears to be from the centre of the robot, since values may
        // exceed 5000. This is good, since it means we only need 1 transform.
        // x & y seem to be swapped though, i.e. if the robot is driving north
        // x is north/south and y is east/west.
        //
        //ArPose sensor = reading->getSensorPosition();  //position sensor.
        // sonar_debug_info << "(" << reading->getLocalX()
        //                  << ", " << reading->getLocalY()
        //                  << ") from (" << sensor.getX() << ", "
        //                  << sensor.getY() << ") ;; " ;
        //add sonar readings (robot-local coordinate frame) to cloud
        geometry_msgs::msg::Point32 p;
        p.x = reading->getLocalX() / 1000.0;
        p.y = reading->getLocalY() / 1000.0;
        p.z = 0.0;
        cloud.points.push_back(p);

        if(distance(p.x, p.y) < minimal_distance){
            obstacle_too_close = true;
        }
    }
    return cloud;
}

void Ros2Aria::publishSonar(sensor_msgs::msg::PointCloud cloud)
{
    if (this->sonar_pub_->get_subscription_count() == 0)
        return;

    this->sonar_pub_->publish(cloud);
}

void Ros2Aria::publishSonarPointCloud2(sensor_msgs::msg::PointCloud cloud)
{
    if (this->sonar_pointcloud2_pub_->get_subscription_count() == 0)
        return;

    sensor_msgs::msg::PointCloud2 cloud2;
    sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);

    this->sonar_pointcloud2_pub_->publish(cloud2);
}