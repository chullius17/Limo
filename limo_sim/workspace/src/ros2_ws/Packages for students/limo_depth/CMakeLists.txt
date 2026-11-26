cmake_minimum_required(VERSION 3.12)
project(limo_depth)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(limo_depth_core SHARED src/geometry.cpp)
target_compile_features(limo_depth_core PUBLIC cxx_std_14)
target_include_directories(limo_depth_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(limo_depth_core PUBLIC Eigen3::Eigen)

add_executable(limo_depth_node src/depth_node.cpp)
target_link_libraries(limo_depth_node limo_depth_core ${OpenCV_LIBS})
target_include_directories(limo_depth_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(limo_depth_node rclcpp rcl_interfaces sensor_msgs std_srvs tf2_ros)

install(TARGETS limo_depth_core EXPORT limo_depth_targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS limo_depth_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_link_libraries(test_geometry limo_depth_core)
  add_test(NAME test_node_integration COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_node_integration.py $<TARGET_FILE:limo_depth_node>)
  set_tests_properties(test_node_integration PROPERTIES TIMEOUT 120 ENVIRONMENT
    "ROS_LOG_DIR=/tmp/limo_depth_test_logs;ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=197")
  add_test(NAME test_launch COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch.py)
  add_test(NAME test_live_launch COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_live_launch.py)
  set_tests_properties(test_live_launch PROPERTIES TIMEOUT 90 ENVIRONMENT
    "ROS_LOG_DIR=/tmp/limo_depth_launch_test_logs;ROS_LOCALHOST_ONLY=1")
endif()

ament_export_targets(limo_depth_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)
ament_package()
