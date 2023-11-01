#include <iostream>
#include <memory>
#include <string>

#include "Aria/Aria.h"
#include "rclcpp/rclcpp.hpp"


class RAIIBot {
   public:
    typedef std::shared_ptr<RAIIBot> SharedPtr;

    RAIIBot(rclcpp::Node *node, std::string port);
    ~RAIIBot();
    ArRobot *getRobot();
    ArGripper *getGripper();
    rclcpp::Clock::SharedPtr getClock();
    void lock();
    void unlock();
    void pokeWatchdog();

   private:
    ArRobot *robot = nullptr;
    ArRobotConnector *robotConn = nullptr;
    ArArgumentBuilder *args = nullptr;
    ArArgumentParser *argparser = nullptr;
    ArGripper *gripper = nullptr;
    rclcpp::Node *node = nullptr;
    rclcpp::TimerBase::SharedPtr watchdogTimer;
    rclcpp::Time lastWatchdogPoke;
    rclcpp::Clock::SharedPtr clock;

    rclcpp::Logger get_logger();
    void startWatchdog();
    void watchdog_cb();
};