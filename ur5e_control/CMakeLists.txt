cmake_minimum_required(VERSION 3.10)
project(ur5e_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  trajectory_msgs
  control_msgs
  actionlib
  actionlib_msgs
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning_interface  
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy trajectory_msgs control_msgs actionlib actionlib_msgs tf2_ros geometry_msgs tf2_geometry_msgs moveit_core moveit_ros_planning_interface  
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ur5e_trajectory_client src/ur5e_trajectory_client.cpp)
add_executable(aruco_follower src/aruco_follower.cpp)

target_link_libraries(ur5e_trajectory_client ${catkin_LIBRARIES})
target_link_libraries(aruco_follower ${catkin_LIBRARIES})

add_dependencies(ur5e_trajectory_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(aruco_follower         ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
