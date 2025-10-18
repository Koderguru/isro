cmake_minimum_required(VERSION 3.0.2)
project(lunar_habitat_robot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  move_base
  amcl
  gmapping
  robot_localization
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  laser_geometry
  message_generation
  actionlib
  actionlib_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  EnvironmentalData.msg
  MaintenanceTask.msg
  HazardAlert.msg
  NavigationStatus.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  StartMaintenance.srv
  GetEnvironmentalData.srv
  SetNavigationGoal.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  PatrolAction.action
  MaintenanceAction.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lunar_habitat_robot
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs move_base amcl gmapping robot_localization cv_bridge image_transport pcl_ros pcl_conversions laser_geometry message_runtime actionlib actionlib_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /usr/include/eigen3
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/navigation/navigation_controller.cpp
  src/navigation/slam_processor.cpp
  src/navigation/path_planner.cpp
  src/sensors/sensor_fusion.cpp
  src/sensors/lidar_processor.cpp
  src/sensors/camera_processor.cpp
  src/sensors/imu_processor.cpp
  src/environmental/environmental_monitor.cpp
  src/environmental/hazard_detector.cpp
  src/maintenance/maintenance_controller.cpp
  src/maintenance/patrol_manager.cpp
  src/utils/coordinate_transformer.cpp
  src/utils/data_logger.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(navigation_node src/nodes/navigation_node.cpp)
add_executable(sensor_fusion_node src/nodes/sensor_fusion_node.cpp)
add_executable(environmental_monitor_node src/nodes/environmental_monitor_node.cpp)
add_executable(maintenance_node src/nodes/maintenance_node.cpp)
add_executable(ui_controller_node src/nodes/ui_controller_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(navigation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sensor_fusion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(environmental_monitor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(maintenance_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ui_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(navigation_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}
)

target_link_libraries(sensor_fusion_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}
)

target_link_libraries(environmental_monitor_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

target_link_libraries(maintenance_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

target_link_libraries(ui_controller_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/navigation_controller.py
  scripts/environmental_monitor.py
  scripts/maintenance_controller.py
  scripts/ui_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
install(TARGETS navigation_node sensor_fusion_node environmental_monitor_node maintenance_node ui_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY web_ui/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/web_ui
  PATTERN ".svn" EXCLUDE)
