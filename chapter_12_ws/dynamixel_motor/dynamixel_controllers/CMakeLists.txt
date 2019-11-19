cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_controllers)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_service_files(
  FILES
  RestartController.srv
  SetComplianceMargin.srv
  SetCompliancePunch.srv
  SetComplianceSlope.srv
  SetSpeed.srv
  SetTorqueLimit.srv
  StartController.srv
  StopController.srv
  TorqueEnable.srv
)

catkin_python_setup()

generate_messages()

catkin_package(
  CATKIN_DEPENDS
  rospy
  actionlib
  dynamixel_driver
  std_msgs
  trajectory_msgs
  diagnostic_msgs
  dynamixel_msgs
  control_msgs
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  nodes/controller_manager.py
  nodes/controller_spawner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
