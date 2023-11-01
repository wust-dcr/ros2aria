#include "ros2aria/ros2aria.hpp"
#include <iostream>
#include <string>
#include <sstream>
using std::placeholders::_1;
using std::placeholders::_2;

Ros2Aria::Ros2Aria()
    : Node("ros2aria"),
      sensorCb(this, &Ros2Aria::publish)
{
    if(get_namespace() != std::string("/")){
        id = extract_number_from_string(std::string(get_namespace()));
    }

    this->robot = std::make_shared<RAIIBot>(this, "/dev/ttyS0");
    handle_parameters();

    RCLCPP_INFO(this->get_logger(), "starting subscribers and services");

    if(use_safety_system){
        restrictions_sub_ = this->create_subscription<ros2aria_msgs::msg::RestrictionsMsg>(
            "/pioneers/restrictions", 10, std::bind(&Ros2Aria::restrictions_callback, this, _1));
        init_restrictions();
        master_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/pioneers/master_stop", 10, std::bind(&Ros2Aria::master_stop_callback, this, _1));
        user_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "user_stop", 10, std::bind(&Ros2Aria::user_stop_callback, this, _1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&Ros2Aria::scan_callback, this, _1));
    }
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&Ros2Aria::cmd_vel_callback, this, _1));
    clutch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "clutch", 10, std::bind(&Ros2Aria::clutch_callback, this, _1));


    auto r = robot->getRobot();
    if(use_sonar){
        sonar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>("sonar", 10);
        sonar_pointcloud2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sonar_pointcloud2", 10);
    }
    else{
        r->disableSonar();
    }
    r->disableMotors();

    pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1000);
    odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    wheels_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1000);

    battery_recharge_state_pub_ = this->create_publisher<std_msgs::msg::Int8>("battery_recharge_state", 10);
    battery_state_of_charge_pub_ = this->create_publisher<std_msgs::msg::Float32>("battery_state_of_charge", 10);
    robot_info_pub_ = this->create_publisher<ros2aria_msgs::msg::RobotInfoMsg>("robot_info", 10);

    // services
    stop_service_ = this->create_service<std_srvs::srv::Empty>("stop", std::bind(&Ros2Aria::stop, this, _1, _2));
    gripper_open_service_ = this->create_service<std_srvs::srv::Empty>("gripper_open", std::bind(&Ros2Aria::gripper_open_callback, this, _1, _2));
    gripper_close_service_ = this->create_service<std_srvs::srv::Empty>("gripper_close", std::bind(&Ros2Aria::gripper_close_callback, this, _1, _2));
    gripper_up_service_ = this->create_service<std_srvs::srv::Empty>("gripper_up", std::bind(&Ros2Aria::gripper_up_callback, this, _1, _2));
    gripper_down_service_ = this->create_service<std_srvs::srv::Empty>("gripper_down", std::bind(&Ros2Aria::gripper_down_callback, this, _1, _2));

    r->addSensorInterpTask("ROSPublishingTask", 100, &this->sensorCb);

    ros_namespace = get_namespace() == std::string("/")? "" : get_namespace();
    RCLCPP_INFO_STREAM(get_logger(), "NAMESPACE = " << ros_namespace);
    RCLCPP_INFO_STREAM(get_logger(), "ID = " << id);

}

std::size_t Ros2Aria::extract_number_from_string(const std::string& str) {
    std::string numericPart;
    for (char c : str) {
        if (isdigit(c)) {
            numericPart += c;
        } else if (!numericPart.empty()) {
            break;
        }
    }
    int number;
    std::istringstream(numericPart) >> number;
    return number;
}

void Ros2Aria::handle_parameters(){
    declare_parameter("use_sonar", true);
    declare_parameter("num_of_sonars", 0);
    declare_parameter("use_safety_system", true);
    use_sonar = get_parameter("use_sonar").as_bool();
    num_of_sonars = get_parameter("num_of_sonars").as_int();
    use_safety_system = get_parameter("use_safety_system").as_bool();

    RCLCPP_INFO_STREAM(get_logger(), "use_sonar = " << use_sonar);
    RCLCPP_INFO_STREAM(get_logger(), "num_of_sonars = " << num_of_sonars);
    RCLCPP_INFO_STREAM(get_logger(), "use_safety_system = " << use_safety_system);
}

Ros2Aria::~Ros2Aria()
{
    auto r = robot->getRobot();
    r->remSensorInterpTask("ROSPublishingTask");
}

void Ros2Aria::stop(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response) const
{
    UNUSED(request);
    UNUSED(response);
    RCLCPP_INFO(this->get_logger(), "stop");
    rclcpp::shutdown();
}