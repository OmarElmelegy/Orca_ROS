cmake_minimum_required(VERSION 3.0.2)
project(orca_hw_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  mavros
  mavros_msgs
  hardware_interface
  controller_manager
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs mavros mavros_msgs hardware_interface controller_manager
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add the MS5837 library
add_library(ms5837 src/ms5837.cpp)
target_link_libraries(ms5837 ${catkin_LIBRARIES})

# Add the pressure sensor node
add_executable(pressure_sensor_node src/pressure_sensor_node.cpp)
target_link_libraries(pressure_sensor_node ms5837 ${catkin_LIBRARIES})
add_dependencies(pressure_sensor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS pressure_sensor_node ms5837
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
