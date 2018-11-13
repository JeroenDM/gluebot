cmake_minimum_required(VERSION 2.8.3)
project(gluebot_demo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  arf_moveit_wrapper
  arf_planning
  arf_trajectory
  ur_kinematics
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gluebot_demo
#  CATKIN_DEPENDS moveit_core moveit_ros_planning_interface moveit_visual_tools roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/gluebot_demo.cpp
# )

add_executable(${PROJECT_NAME}_demo
  src/demo.cpp src/ur5_kin.cpp
)
add_executable(${PROJECT_NAME}_ur5_ik_test
  src/ur5_ik_test.cpp
)


set_target_properties(${PROJECT_NAME}_demo
  PROPERTIES
  OUTPUT_NAME demo
  PREFIX ""
)
set_target_properties(${PROJECT_NAME}_ur5_ik_test
  PROPERTIES
  OUTPUT_NAME ur5_ik_test
  PREFIX ""
)


target_link_libraries(${PROJECT_NAME}_demo
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_ur5_ik_test
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gluebot_demo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
