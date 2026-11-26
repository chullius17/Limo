cmake_minimum_required(VERSION 3.12)
project(limo_map)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# The evidence and map serialization library can also be checked without ROS.
option(LIMO_MAP_CORE_ONLY "Build only the ROS-independent library and tests" OFF)
add_library(limo_map_core SHARED src/grid.cpp src/map_io.cpp)
target_compile_features(limo_map_core PUBLIC cxx_std_17)
target_include_directories(limo_map_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(LIMO_MAP_CORE_ONLY)
  enable_testing()
  add_executable(test_grid test/test_grid.cpp)
  target_link_libraries(test_grid limo_map_core)
  add_test(NAME test_grid COMMAND test_grid)
  return()
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
# Optional: normal TF mapping has no Cartographer build/runtime dependency.
find_package(cartographer_ros_msgs QUIET)

add_library(limo_map_node_lib SHARED src/cloud.cpp src/limo_map_node.cpp)
target_link_libraries(limo_map_node_lib limo_map_core)
ament_target_dependencies(limo_map_node_lib
  geometry_msgs nav_msgs rclcpp rcl_interfaces sensor_msgs std_msgs std_srvs tf2_ros)
if(cartographer_ros_msgs_FOUND)
  target_compile_definitions(limo_map_node_lib PRIVATE LIMO_MAP_HAS_CARTOGRAPHER)
  ament_target_dependencies(limo_map_node_lib cartographer_ros_msgs)
endif()
add_executable(limo_map_node src/main.cpp)
target_link_libraries(limo_map_node limo_map_node_lib)

add_library(limo_map_gui_lib SHARED src/map_save_window.cpp)
target_compile_features(limo_map_gui_lib PUBLIC cxx_std_17)
target_include_directories(limo_map_gui_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(limo_map_gui_lib Qt5::Widgets)
ament_target_dependencies(limo_map_gui_lib rclcpp std_msgs std_srvs)
add_executable(map_save_gui src/map_save_gui.cpp)
target_link_libraries(map_save_gui limo_map_gui_lib)

install(TARGETS limo_map_core limo_map_node_lib
  EXPORT limo_map_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS limo_map_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS limo_map_gui_lib LIBRARY DESTINATION lib)
install(TARGETS map_save_gui DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_grid test/test_grid.cpp)
  target_link_libraries(test_grid limo_map_core)
  add_test(NAME test_grid COMMAND test_grid)
  add_executable(test_cloud test/test_cloud.cpp)
  target_link_libraries(test_cloud limo_map_node_lib)
  add_test(NAME test_cloud COMMAND test_cloud)
  add_executable(test_gui test/test_gui.cpp)
  target_link_libraries(test_gui limo_map_gui_lib)
  add_test(NAME test_gui COMMAND test_gui)
  set_tests_properties(test_gui PROPERTIES TIMEOUT 30 ENVIRONMENT
    "QT_QPA_PLATFORM=offscreen;ROS_LOG_DIR=/tmp/limo_map_gui_test_logs;ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=195")
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_mapping_launch
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mapping_launch.py)
  add_test(NAME test_stack_integration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_stack_integration.py)
  set_tests_properties(test_stack_integration PROPERTIES TIMEOUT 35 ENVIRONMENT
    "QT_QPA_PLATFORM=offscreen;ROS_LOG_DIR=/tmp/limo_map_stack_test_logs;ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=197;PYTHONDONTWRITEBYTECODE=1")
  add_test(NAME test_node_integration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_node_integration.py
      $<TARGET_FILE:limo_map_node>)
  set_tests_properties(test_node_integration PROPERTIES TIMEOUT 45 ENVIRONMENT
    "ROS_LOG_DIR=/tmp/limo_map_test_logs;ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=193")
endif()

ament_export_targets(limo_map_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs nav_msgs rclcpp rcl_interfaces sensor_msgs std_msgs std_srvs tf2_ros)
ament_package()
