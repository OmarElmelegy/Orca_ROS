cmake_minimum_required(VERSION 3.0.2)
project(orca_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
  robot_state_publisher
  joint_state_publisher
  joint_state_publisher_gui
  rviz
  gazebo_ros
)

catkin_package(
  CATKIN_DEPENDS urdf xacro robot_state_publisher joint_state_publisher joint_state_publisher_gui rviz gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
