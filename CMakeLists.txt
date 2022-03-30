cmake_minimum_required(VERSION 3.0.2)
project(ffrc_motion_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(LIBRARY_NAME moveit_ffrc_planner_plugin)

set(CMAKE_MODULE_PATH
  "${CMAKE_MODULE_PATH}"
  "${CMAKE_ROOT_DIR}/cmake/Modules"
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

include(CompileOptions)
include(HelperFunctions)

find_package(Boost REQUIRED filesystem date_time thread)
find_package(Eigen3 REQUIRED)
# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND LIBRARIES ${OpenCV_LIBRARIES})

# ompl
find_package(ompl REQUIRED)
include_directories(SYSTEM ${ompl_INCLUDE_DIRS})
list(APPEND LIBRARIES ${ompl_LIBRARIES})

# message("1 ${OMPL_FOUND}")
# message("2 ${OMPL_INCLUDE_DIRS}")
# message("3 ${OMPL_LIBRARY_DIRS}")
# message("4 ${OMPL_LIBRARIES}")
# message("5 ${OMPLAPP_LIBRARIES}")
# message("6 ${OMPL_VERSION}")

##
## Catkin setup
##

list(APPEND CATKIN_SOURCES
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    moveit_planners_ompl
  )

find_package(catkin REQUIRED COMPONENTS ${CATKIN_SOURCES})

catkin_package(
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS ${CATKIN_SOURCES}
  DEPENDS
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
  )

##
## Library Creation
##

list(APPEND SOURCES
  src/ffrc_planner_manager.cpp
  src/ffrc_interface.cpp
  src/ffrc_planning_context.cpp
  src/ffrc.cpp
)

list(APPEND INCLUDES
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

list(APPEND LIBRARIES
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )


include_directories(SYSTEM ${INCLUDES})
link_directories(${catkin_LIBRARY_DIRS})
add_library(${LIBRARY_NAME} ${SOURCES})
set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${LIBRARY_NAME} ${LIBRARIES})

##
## Scripts
##

add_script(ffrc_demo scripts)
add_script(planner_viz scripts)

# add_script(omplapp tests/ompl_demos)

##
## Installation of programs, library, headers, and YAML used by scripts
##

install_scripts()
install_library()
install_directory(yaml)
