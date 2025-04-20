cmake_minimum_required(VERSION 3.0.2)
project(orca_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
  tf2
  tf2_ros
  dynamic_reconfigure
  controller_manager
  control_toolbox
)

# Find Eigen library
find_package(Eigen3 REQUIRED)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/ThrusterControl.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs mavros_msgs tf2 tf2_ros dynamic_reconfigure controller_manager control_toolbox
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(thruster_allocator src/thruster_allocator.cpp)
target_link_libraries(thruster_allocator ${catkin_LIBRARIES})
add_dependencies(thruster_allocator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(depth_controller src/depth_controller.cpp)
target_link_libraries(depth_controller ${catkin_LIBRARIES})
add_dependencies(depth_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(heading_controller src/heading_controller.cpp)
target_link_libraries(heading_controller ${catkin_LIBRARIES})
add_dependencies(heading_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS thruster_allocator depth_controller heading_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
