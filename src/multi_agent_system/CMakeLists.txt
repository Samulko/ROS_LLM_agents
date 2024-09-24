cmake_minimum_required(VERSION 3.0.2)
project(multi_agent_system)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  UserCommand.msg
  AgentResponse.msg
)

add_service_files(
  FILES
  ValidateRequest.srv
  PlanExecution.srv
  StabilityAnalysis.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
