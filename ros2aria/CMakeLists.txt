cmake_minimum_required(VERSION 3.5)
project(ros2aria)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros2aria_msgs REQUIRED)

add_library(aria_coda SHARED
  lib/AriaCoda/src/ArAction.cpp 
	lib/AriaCoda/src/ArActionAvoidFront.cpp 
	lib/AriaCoda/src/ArActionAvoidSide.cpp 
	lib/AriaCoda/src/ArActionBumpers.cpp 
	lib/AriaCoda/src/ArActionConstantVelocity.cpp 
	lib/AriaCoda/src/ArActionDeceleratingLimiter.cpp 
	lib/AriaCoda/src/ArActionDesired.cpp 
	lib/AriaCoda/src/ArActionDriveDistance.cpp 
	lib/AriaCoda/src/ArActionGoto.cpp 
	lib/AriaCoda/src/ArActionGotoStraight.cpp 
	lib/AriaCoda/src/ArActionGroup.cpp 
	lib/AriaCoda/src/ArActionGroups.cpp 
	lib/AriaCoda/src/ArActionInput.cpp 
	lib/AriaCoda/src/ArActionIRs.cpp 
	lib/AriaCoda/src/ArActionJoydrive.cpp 
	lib/AriaCoda/src/ArActionKeydrive.cpp 
	lib/AriaCoda/src/ArActionLimiterBackwards.cpp 
	lib/AriaCoda/src/ArActionLimiterForwards.cpp 
	lib/AriaCoda/src/ArActionLimiterRot.cpp 
	lib/AriaCoda/src/ArActionLimiterTableSensor.cpp 
	lib/AriaCoda/src/ArActionMovementParameters.cpp 
	lib/AriaCoda/src/ArActionMovementParametersDebugging.cpp 
	lib/AriaCoda/src/ArActionRatioInput.cpp 
	lib/AriaCoda/src/ArActionRobotJoydrive.cpp 
	lib/AriaCoda/src/ArActionStallRecover.cpp 
	lib/AriaCoda/src/ArActionStop.cpp 
	lib/AriaCoda/src/ArActionTriangleDriveTo.cpp 
	lib/AriaCoda/src/ArActionTurn.cpp 
	lib/AriaCoda/src/ArAnalogGyro.cpp 
	lib/AriaCoda/src/ArArg.cpp 
	lib/AriaCoda/src/ArArgumentBuilder.cpp 
	lib/AriaCoda/src/ArArgumentParser.cpp 
	lib/AriaCoda/src/ArASyncTask.cpp 
	lib/AriaCoda/src/ArBasePacket.cpp 
	lib/AriaCoda/src/ArBatteryConnector.cpp 
	lib/AriaCoda/src/ArBatteryMTX.cpp 
	lib/AriaCoda/src/ArBumpers.cpp 
	lib/AriaCoda/src/ArCondition_LIN.cpp 
	lib/AriaCoda/src/ArConfig.cpp 
	lib/AriaCoda/src/ArConfigArg.cpp 
	lib/AriaCoda/src/ArConfigGroup.cpp 
	lib/AriaCoda/src/ArDataLogger.cpp 
	lib/AriaCoda/src/ArDeviceConnection.cpp 
	lib/AriaCoda/src/ArDPPTU.cpp 
	lib/AriaCoda/src/ArFileDeviceConnection.cpp 
	lib/AriaCoda/src/ArFileParser.cpp 
	lib/AriaCoda/src/ArForbiddenRangeDevice.cpp 
	lib/AriaCoda/src/ArFunctorASyncTask.cpp 
	lib/AriaCoda/src/ArGPS.cpp 
	lib/AriaCoda/src/ArGPSConnector.cpp 
	lib/AriaCoda/src/ArGPSCoords.cpp 
	lib/AriaCoda/src/ArGripper.cpp 
	lib/AriaCoda/src/ArInterpolation.cpp 
	lib/AriaCoda/src/ArIRs.cpp 
	lib/AriaCoda/src/ArJoyHandler.cpp 
	lib/AriaCoda/src/ArKeyHandler.cpp 
	lib/AriaCoda/src/ArLaser.cpp 
	lib/AriaCoda/src/ArLaserConnector.cpp 
	lib/AriaCoda/src/ArLaserFilter.cpp 
	lib/AriaCoda/src/ArLaserLogger.cpp 
	lib/AriaCoda/src/ArLCDConnector.cpp 
	lib/AriaCoda/src/ArLCDMTX.cpp
	lib/AriaCoda/src/ArLineFinder.cpp
	lib/AriaCoda/src/ArLMS1XX.cpp
	lib/AriaCoda/src/ArLMS2xx.cpp
	lib/AriaCoda/src/ArLMS2xxPacket.cpp 
	lib/AriaCoda/src/ArLMS2xxPacketReceiver.cpp 
	lib/AriaCoda/src/ArLog.cpp 
	lib/AriaCoda/src/ArMap.cpp 
	lib/AriaCoda/src/ArMapComponents.cpp 
	lib/AriaCoda/src/ArMapInterface.cpp 
	lib/AriaCoda/src/ArMapObject.cpp
	lib/AriaCoda/src/ArMapUtils.cpp
	lib/AriaCoda/src/ArMD5Calculator.cpp
	lib/AriaCoda/src/ArMutex.cpp
	lib/AriaCoda/src/ArMutex_LIN.cpp
	lib/AriaCoda/src/ArNMEAParser.cpp
	lib/AriaCoda/src/ArNovatelGPS.cpp
  lib/AriaCoda/src/ArPacketUtil.cpp
	lib/AriaCoda/src/ArPriorityResolver.cpp
	lib/AriaCoda/src/ArPTZ.cpp
	lib/AriaCoda/src/ArPTZConnector.cpp
	lib/AriaCoda/src/ArRangeBuffer.cpp
	lib/AriaCoda/src/ArRangeDevice.cpp
	lib/AriaCoda/src/ArRangeDeviceThreaded.cpp
	lib/AriaCoda/src/ArRatioInputKeydrive.cpp
	lib/AriaCoda/src/ArRatioInputJoydrive.cpp
	lib/AriaCoda/src/ArRatioInputRobotJoydrive.cpp
	lib/AriaCoda/src/ArRecurrentTask.cpp
	lib/AriaCoda/src/ArRobot.cpp
	lib/AriaCoda/src/ArRobotBatteryPacketReader.cpp
	lib/AriaCoda/src/ArRobotConfigPacketReader.cpp
	lib/AriaCoda/src/ArRobotConnector.cpp
	lib/AriaCoda/src/ArRobotJoyHandler.cpp
	lib/AriaCoda/src/ArRobotPacket.cpp
	lib/AriaCoda/src/ArRobotPacketReceiver.cpp
	lib/AriaCoda/src/ArRobotPacketReaderThread.cpp
	lib/AriaCoda/src/ArRobotPacketSender.cpp
	lib/AriaCoda/src/ArRobotParams.cpp
	lib/AriaCoda/src/ArRobotTypes.cpp
	lib/AriaCoda/src/ArRVisionPTZ.cpp
	lib/AriaCoda/src/ArS3Series.cpp
	lib/AriaCoda/src/ArSZSeries.cpp
  lib/AriaCoda/src/ArSeekurIMU.cpp
	lib/AriaCoda/src/ArSimulatedLaser.cpp
	lib/AriaCoda/src/ArSocket.cpp
	lib/AriaCoda/src/ArSonarConnector.cpp
	lib/AriaCoda/src/ArSonarDevice.cpp
	lib/AriaCoda/src/ArSonarMTX.cpp
	lib/AriaCoda/src/ArSensorReading.cpp
	lib/AriaCoda/src/ArSoundsQueue.cpp
	lib/AriaCoda/src/ArSoundPlayer.cpp
	lib/AriaCoda/src/ArStringInfoGroup.cpp
	lib/AriaCoda/src/ArSyncLoop.cpp
	lib/AriaCoda/src/ArSyncTask.cpp
	lib/AriaCoda/src/ArSonarAutoDisabler.cpp
	lib/AriaCoda/src/ArTcpConnection.cpp
	lib/AriaCoda/src/ArThread.cpp
	lib/AriaCoda/src/ArThread_LIN.cpp
	lib/AriaCoda/src/ArTransform.cpp
	lib/AriaCoda/src/ArTrimbleGPS.cpp
	lib/AriaCoda/src/ArUrg.cpp
	lib/AriaCoda/src/ArUrg_2_0.cpp
	lib/AriaCoda/src/ArVCC4.cpp
	lib/AriaCoda/src/Aria.cpp
	lib/AriaCoda/src/ariaUtil.cpp
	lib/AriaCoda/src/md5.cpp
  lib/AriaCoda/src/ArSocket_LIN.cpp
  lib/AriaCoda/src/ArJoyHandler_LIN.cpp
  lib/AriaCoda/src/ArSerialConnection_LIN.cpp
  lib/AriaCoda/src/ArSignalHandler_LIN.cpp
  lib/AriaCoda/src/ArSystemStatus.cpp
  lib/AriaCoda/src/ArMTXIO.cpp
)

include_directories(
  lib/AriaCoda/include/
)

add_executable(ros2aria
  src/main.cpp
  src/ros2aria.cpp
  src/raiibot.cpp
  src/cmd_vel.cpp
  src/publish.cpp
  src/sonar.cpp
  src/pose.cpp
  src/gripper.cpp
  src/wheels.cpp
  src/clutch.cpp
  src/state.cpp
  src/restrictions.cpp
  src/scan.cpp
)

target_include_directories(ros2aria PRIVATE
  include
)

target_link_libraries(ros2aria
  aria_coda
)

ament_target_dependencies(ros2aria
  rclcpp
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ros2aria_msgs
)

ament_export_libraries(
  aria_coda
)

install(TARGETS  aria_coda
  DESTINATION lib)
  install(TARGETS ros2aria
  DESTINATION lib/ros2aria)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    DIRECTORY lib/AriaCoda/include
    DESTINATION lib/AriaCoda/include
)

ament_export_include_directories(
  include
  lib/AriaCoda/include
)

ament_package()
