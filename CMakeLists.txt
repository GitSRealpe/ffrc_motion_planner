cmake_minimum_required(VERSION 3.0.2)
project(ffrc_motion_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

list(APPEND INCLUDES
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

include_directories(SYSTEM ${INCLUDES})

add_executable(ffrc_example  src/ffrc_example.cpp)
target_link_libraries(ffrc_example
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Ffrc planning plugin
add_library(moveit_ffrc_planner_plugin
  src/ffrc_planner_manager.cpp
  src/ffrc_interface.cpp
  src/ffrc_planning_context.cpp)
# set_target_properties(moveit_ffrc_planner_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_ffrc_planner_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    moveit_ffrc_planner_plugin
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Mark roslaunch files for installation
# install(
#   DIRECTORY
#     launch
#   DESTINATION
#     ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
#catkin_lint ignore uninstalled_plugin
install(FILES ffrc_interface_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ffrc_motion_planner/)