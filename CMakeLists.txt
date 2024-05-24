cmake_minimum_required(VERSION 3.5)
project(dstar_global_planner)

# Default to C99
set(CMAKE_C_STANDARD 99)


# Default to C++14
set(CMAKE_CXX_STANDARD 17)



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

# find_package( roslib REQUIRED )
find_package( tf2 REQUIRED )
find_package( tf2_geometry_msgs REQUIRED )
# find_package( sensor_msgs REQUIRED )
# find_package( move_base REQUIRED )

find_package( jsoncpp REQUIRED )
find_package( OpenCV REQUIRED )


include_directories(
  include
)

set(library_name ${PROJECT_NAME}_plugin)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  nav2_core
  pluginlib
  # roslib
  tf2
  tf2_geometry_msgs
)

add_library(dstar src/dstar.cpp)

add_library(${library_name} SHARED
  src/trajectory_planner_plugin.cpp
)

add_library(costmap_virtual_walls SHARED src/virtual_walls_plugin.cpp)

target_include_directories(costmap_virtual_walls PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(costmap_virtual_walls
  ${dependencies}
  OpenCV
)

target_compile_options(dstar PRIVATE -fPIC)
target_compile_options(${library_name} PRIVATE -fPIC)
# target_compile_options(costmap_virtual_walls PRIVATE -fPIC)
# target_link_options()

ament_export_libraries(dstar)
ament_export_libraries(costmap_virtual_walls)
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(dstar PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
# target_compile_definitions(costmap_virtual_walls PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# target_link_libraries(dstar ${rclcpp_LIBRARIES})
# target_include_directories(dstar PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>"
#   ${rclcpp_INCLUDE_DIRS}
# )
target_link_libraries(${library_name} dstar jsoncpp)

ament_target_dependencies(${library_name}
  ${dependencies}
  # dstar
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

# install(
#   TARGETS costmap_virtual_walls
#   EXPORT export_${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )


install(TARGETS ${library_name} dstar costmap_virtual_walls
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(include)
ament_export_libraries(${library_name} dstar costmap_virtual_walls)
# ament_export_targets(
#   export_${PROJECT_NAME}
# )
ament_export_dependencies(${dependencies} dstar)
ament_package()