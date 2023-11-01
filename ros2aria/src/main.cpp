#include <cstdio>
#include <memory>

#include "ros2aria/ros2aria.hpp"

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  printf("hello world ros2aria package\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ros2Aria>());
  rclcpp::shutdown();

  return 0;
}
