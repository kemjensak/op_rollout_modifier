cmake_minimum_required(VERSION 3.0.2)
project(op_rollout_modifier)

find_package(
  catkin REQUIRED COMPONENTS
    autoware_can_msgs
    geometry_msgs
    jsk_recognition_msgs
    libwaypoint_follower
    map_file
    op_planner
    op_ros_helpers
    op_simu  
    op_utility
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    tf
    vector_map_msgs
    lanelet2_extension
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    autoware_can_msgs
    geometry_msgs
    libwaypoint_follower
    map_file
    op_planner
    op_ros_helpers
    op_simu
    op_utility
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(lib)

add_executable(
  op_rollout_modifier
  src/op_rollout_modifier.cpp
  
)

target_link_libraries(op_rollout_modifier ${catkin_LIBRARIES} ${PCL_LIBRARIES})