cmake_minimum_required(VERSION 3.1)
project(dstar_trajectory_planner)

add_compile_options(-std=c++17)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
    roslib
    roscpp
    tf2
    tf2_geometry_msgs
    sensor_msgs
    geometry_msgs
    std_msgs
    nav_msgs
    nav_core
    pluginlib
    costmap_2d
    move_base
)

include_directories(${catkin_INCLUDE_DIRS} include)

catkin_package(CATKIN_DEPENDS
    message_runtime
    sensor_msgs
    geometry_msgs
    std_msgs
    nav_msgs
    tf2
    tf2_geometry_msgs
    nav_core
    pluginlib
    costmap_2d
    move_base
)
add_library(dstar src/dstar.cpp)

add_library(dstar_global_planner src/trajectory_planner_plugin.cpp)
target_link_libraries(dstar_global_planner PUBLIC ${catkin_LIBRARIES} dstar)

