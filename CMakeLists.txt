# cmake_minimum_required(VERSION 3.1)
# project(dstar_trajectory_planner)

# add_compile_options(-std=c++17)

# set(CMAKE_BUILD_TYPE Release)

# find_package(catkin REQUIRED COMPONENTS
#     roslib
#     roscpp
#     tf2
#     tf2_geometry_msgs
#     sensor_msgs
#     geometry_msgs
#     std_msgs
#     nav_msgs
#     nav_core
#     pluginlib
#     costmap_2d
#     move_base
# )

# include_directories(${catkin_INCLUDE_DIRS} include)

# catkin_package(CATKIN_DEPENDS
#     message_runtime
#     sensor_msgs
#     geometry_msgs
#     std_msgs
#     nav_msgs
#     tf2
#     tf2_geometry_msgs
#     nav_core
#     pluginlib
#     costmap_2d
#     move_base
# )
# add_library(dstar src/dstar.cpp)

# add_library(dstar_global_planner src/trajectory_planner_plugin.cpp)
# target_link_libraries(dstar_global_planner PUBLIC ${catkin_LIBRARIES} dstar)


cmake_minimum_required(VERSION 3.5)
project(dstar_trajectory_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)
find_package( rclcpp REQUIRED )
# find_package( roslib REQUIRED )
# find_package( tf2 REQUIRED )
# find_package( tf2_geometry_msgs REQUIRED )
# find_package( sensor_msgs REQUIRED )
find_package( geometry_msgs REQUIRED )
# find_package( std_msgs REQUIRED )
find_package( nav_msgs REQUIRED )
find_package( nav2_core REQUIRED )
# find_package( pluginlib REQUIRED )
find_package( nav2_costmap_2d REQUIRED )
# find_package( move_base REQUIRED )

include_directories(include)

# catkin_package(CATKIN_DEPENDS
#     message_runtime
#     sensor_msgs
#     geometry_msgs
#     std_msgs
#     nav_msgs
#     tf2
#     tf2_geometry_msgs
#     nav_core
#     pluginlib
#     costmap_2d
#     move_base
# )


add_library(dstar src/dstar.cpp)

# add_library(jobot_trajectory_planner src/trajectory_planner.cpp)
# # ament_export_targets(jobot_trajectory_planner HAS_LIBRARY_TARGET)
# ament_target_dependencies(jobot_trajectory_planner
#   rclcpp
#   std_msgs
#   nav2_core
#   geometry_msgs
#   )
add_library(dstar_global_planner src/trajectory_planner_plugin.cpp)
# target_link_libraries(dstar_global_planner PUBLIC ${catkin_LIBRARIES} dstar)
ament_target_dependencies(dstar_global_planner
  rclcpp
  std_msgs
  nav2_core
  nav2_costmap_2d
  geometry_msgs
  )

pluginlib_export_plugin_description_file(nav2_core dstar_global_planner_plugin.xml)

  # add_executable(jobot_trajectory_planner_node src/trajectory_planner_node.cpp)
# target_link_libraries(jobot_trajectory_planner_node PRIVATE ${catkin_LIBRARIES} jobot_trajectory_planner)

# find_package(OpenCV REQUIRED)

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(JSONCPP jsoncpp)
# FIND_PACKAGE(Boost COMPONENTS filesystem REQUIRED)
# add_library(virtual_walls src/virtual_walls_plugin.cpp)
# target_include_directories(virtual_walls PRIVATE ${OpenCV_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
# target_link_libraries(virtual_walls PRIVATE ${catkin_LIBRARIES} ${OpenCV_LIBS} ${JSONCPP_LIBRARIES} ${Boost_LIBRARIES})

# # Internal adoption of SpiralSTC library
# add_library(spiral_planner src/spiral_planner.cpp)
# target_link_libraries(spiral_planner PRIVATE ${catkin_LIBRARIES})
# # Plugin library
# add_library(spiral_coverage_planner src/spiral_coverage_planner_plugin.cpp)
# target_link_libraries(spiral_coverage_planner PRIVATE ${catkin_LIBRARIES} spiral_planner)

ament_package()