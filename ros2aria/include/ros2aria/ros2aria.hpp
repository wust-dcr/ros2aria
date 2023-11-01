#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>
#include<string>

#include "ros2aria/raiibot.hpp"
#include <Aria/Aria.h>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2aria_msgs/msg/robot_info_msg.hpp"
#include "ros2aria_msgs/msg/restrictions_msg.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#define UNUSED(x) (void)(x)

constexpr double mm_per_sec_to_rad_per_sec(double mm_per_sec){
    return mm_per_sec  / 195.0 / 2.0; /*diameter in mm*/
}

constexpr double encoder_to_rad(long int encoder){
    return static_cast<double>(encoder) * M_PI / 32768.0;
}

constexpr float distance(float a, float b){
    return std::sqrt(a*a + b*b);
}

class Ros2Aria : public rclcpp::Node
{
public:
    Ros2Aria();
    ~Ros2Aria();

private:
    std::size_t id{0};
    RAIIBot::SharedPtr robot;
    ArFunctorC<Ros2Aria> sensorCb;
    ros2aria_msgs::msg::RestrictionsMsg restrictions;
    float minimal_distance = 0.0;
    bool obstacle_too_close = true;
    bool master_stop = true;
    bool user_stop = true;
    bool use_sonar = true;
    bool use_safety_system = true;
    uint8_t num_of_sonars = 0;
    std::string ros_namespace="";

    std::size_t extract_number_from_string(const std::string& str);

    void handle_parameters();
    void publish();

    sensor_msgs::msg::PointCloud handleSonar(rclcpp::Time stamp);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr sonar_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sonar_pointcloud2_pub_;
    void publishSonar(sensor_msgs::msg::PointCloud cloud);
    void publishSonarPointCloud2(sensor_msgs::msg::PointCloud cloud);

    std::pair<nav_msgs::msg::Odometry, geometry_msgs::msg::TransformStamped> handlePose(rclcpp::Time stamp);
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
    void publishPose(nav_msgs::msg::Odometry pose);
    void publishTf(geometry_msgs::msg::TransformStamped tf);

    sensor_msgs::msg::JointState handleWheels(rclcpp::Time stamp);
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheels_pub_;
    void publishWheels(sensor_msgs::msg::JointState wheels);

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr battery_recharge_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_state_of_charge_pub_;
    rclcpp::Publisher<ros2aria_msgs::msg::RobotInfoMsg>::SharedPtr robot_info_pub_;
    void publishState();

    // subscribers
    rclcpp::Subscription<ros2aria_msgs::msg::RestrictionsMsg>::SharedPtr restrictions_sub_;
    void restrictions_callback(const ros2aria_msgs::msg::RestrictionsMsg::SharedPtr msg);
    void init_restrictions();

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr user_stop_sub_;
    void user_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr master_stop_sub_;
    void master_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clutch_sub_;
    void clutch_callback(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    void scan_callback(const sensor_msgs::msg::LaserScan msg);

    // services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;
    void stop(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response) const;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr gripper_open_service_;
    void gripper_open_callback(const std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response) const;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr gripper_close_service_;
    void gripper_close_callback(const std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response) const;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr gripper_up_service_;
    void gripper_up_callback(const std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response) const;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr gripper_down_service_;
    void gripper_down_callback(const std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response) const;
};