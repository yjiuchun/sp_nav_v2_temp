cmake_minimum_required(VERSION 3.0.2)
project(sp_decision)
# 设置编译选项
# option(DEBUG_MODE "Enable debug mode" OFF)

# # 添加可选的编译宏定义
# if(DEBUG_MODE)
#     add_compile_definitions(DEBUG_MODE)
# endif()


find_package(catkin REQUIRED COMPONENTS
  robot_msg
  roscpp
  rospy
  roslib
  geometry_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sp_decision
#  CATKIN_DEPENDS robot_msg roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  include
  ${CMAKE_CURRENT_SOURCE_DIR} 
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
src/main.cpp
src/perception/blackboard.cpp
src/executor/chassis.cpp
src/decision_node/decision_tree.cpp
src/executor/control_node.cpp
src/executor/gimbal.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(armor_relay
src/perception/armor_relay.cpp
)
target_link_libraries(armor_relay
  ${catkin_LIBRARIES})

add_executable(cloud_tracker
src/perception/cloud_tracker.cpp
)
target_link_libraries(cloud_tracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES})

add_executable(cmd_vel_test
src/perception/cmd_vel_test.cpp
)
target_link_libraries(cmd_vel_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(map_matching
include/tools/map_matching.cpp)
target_link_libraries(map_matching
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    yaml-cpp)
  
  
add_executable(plan_select
include/tools/plan_select.cpp)
target_link_libraries(plan_select
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    yaml-cpp)
# add_executable(yaml_test
# src/decision_node/yaml_read.cpp
# )
# target_link_libraries(yaml_test 
# ${catkin_LIBRARIES}
# yaml-cpp
# )
