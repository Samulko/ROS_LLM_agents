cmake_minimum_required(VERSION 3.0.2)
project(manager_agent)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  multi_agent_system
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs multi_agent_system
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/manager_agent.py
  scripts/test_manager_agent.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
