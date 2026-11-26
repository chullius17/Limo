cmake_minimum_required(VERSION 3.12)
project(limo_amcl)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
set(dependencies
  rclcpp rclcpp_lifecycle diagnostic_msgs message_filters tf2_geometry_msgs
  geometry_msgs nav_msgs sensor_msgs std_srvs tf2_ros tf2 nav2_util nav2_msgs)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)
include(CheckSymbolExists)
check_symbol_exists(drand48 stdlib.h HAVE_DRAND48)
add_subdirectory(src/pf)
add_subdirectory(src/map)
add_subdirectory(src/motion_model)
add_subdirectory(src/sensors)

add_library(limo_amcl_core SHARED
  src/amcl_node.cpp src/amcl_cv.cpp
  src/experiments/cv_diagnostics.cpp src/experiments/laser_diagnostics.cpp)
target_include_directories(limo_amcl_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(limo_amcl_core PRIVATE src/include)
if(HAVE_DRAND48)
  target_compile_definitions(limo_amcl_core PRIVATE HAVE_DRAND48)
endif()
ament_target_dependencies(limo_amcl_core ${dependencies})
target_link_libraries(limo_amcl_core limo_amcl_map limo_amcl_motion limo_amcl_sensors)
add_executable(limo_amcl_node src/main.cpp)
target_link_libraries(limo_amcl_node limo_amcl_core)
ament_target_dependencies(limo_amcl_node rclcpp)

install(TARGETS limo_amcl_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS limo_amcl_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md COPYING.LGPL LICENSE.Apache-2.0 DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cv_cloud test/test_cv_cloud.cpp)
  ament_target_dependencies(test_cv_cloud nav_msgs sensor_msgs tf2)
  target_link_libraries(test_cv_cloud limo_amcl_sensors limo_amcl_map limo_amcl_pf)
  ament_add_gtest(test_cv_sync test/test_cv_sync.cpp)
  ament_target_dependencies(test_cv_sync ${dependencies})
  target_link_libraries(test_cv_sync limo_amcl_core)
  set_tests_properties(test_cv_sync PROPERTIES ENVIRONMENT
    "ROS_LOG_DIR=/tmp/limo_amcl_test_logs;ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=201")
  ament_add_gtest(test_laser_counts test/test_laser_counts.cpp)
  target_link_libraries(test_laser_counts limo_amcl_sensors limo_amcl_map limo_amcl_pf)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_launch
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch.py)
  set_tests_properties(test_launch PROPERTIES ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
  add_test(NAME test_node_integration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_node_integration.py)
  set_tests_properties(test_node_integration PROPERTIES TIMEOUT 50 ENVIRONMENT
    "QT_QPA_PLATFORM=offscreen;ROS_LOG_DIR=/tmp/limo_amcl_integration_logs;ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=202;PYTHONDONTWRITEBYTECODE=1")
endif()

ament_export_include_directories(include)
ament_export_libraries(limo_amcl_core limo_amcl_pf limo_amcl_sensors limo_amcl_motion limo_amcl_map)
ament_export_dependencies(${dependencies})
ament_package()
