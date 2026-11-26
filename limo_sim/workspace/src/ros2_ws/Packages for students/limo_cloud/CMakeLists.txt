cmake_minimum_required(VERSION 3.12)
project(limo_cloud)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Threads REQUIRED)

add_library(limo_cloud_core SHARED src/projector.cpp)
target_compile_features(limo_cloud_core PUBLIC cxx_std_14)
target_include_directories(limo_cloud_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(limo_cloud_core geometry_msgs rclcpp sensor_msgs)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(limo_cloud_core PRIVATE -ffp-contract=off)
endif()

add_library(limo_cloud_python SHARED src/python_processor.cpp)
target_link_libraries(limo_cloud_python PUBLIC limo_cloud_core PRIVATE Python3::Python)
set_source_files_properties(src/python_processor.cpp PROPERTIES
  COMPILE_OPTIONS "-Wno-missing-field-initializers")

add_library(limo_cloud_processors SHARED src/select_classes.cpp)
target_link_libraries(limo_cloud_processors limo_cloud_core)
ament_target_dependencies(limo_cloud_processors pluginlib)
pluginlib_export_plugin_description_file(limo_cloud plugins.xml)

add_library(limo_cloud_node_lib SHARED src/limo_cloud_node.cpp)
target_link_libraries(limo_cloud_node_lib
  PUBLIC limo_cloud_core
  PRIVATE limo_cloud_python Threads::Threads)
ament_target_dependencies(limo_cloud_node_lib PUBLIC pluginlib tf2_ros)

add_executable(limo_cloud_node src/main.cpp)
target_link_libraries(limo_cloud_node limo_cloud_node_lib)

install(TARGETS limo_cloud_core limo_cloud_python limo_cloud_processors limo_cloud_node_lib
  EXPORT limo_cloud_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS limo_cloud_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_projector test/test_projector.cpp)
  target_link_libraries(test_projector limo_cloud_core)
  ament_add_gtest(test_processors test/test_processors.cpp)
  target_link_libraries(test_processors limo_cloud_core limo_cloud_python)
  ament_target_dependencies(test_processors pluginlib)
  set_tests_properties(test_processors PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:${CMAKE_CURRENT_SOURCE_DIR}/test:$ENV{PYTHONPATH};ROS_LOG_DIR=/tmp/limo_cloud_test_logs;ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=191")
  add_test(NAME test_node_integration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_node_integration.py
      $<TARGET_FILE:limo_cloud_node>)
  set_tests_properties(test_node_integration PROPERTIES TIMEOUT 40 ENVIRONMENT
    "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python:$ENV{PYTHONPATH};ROS_LOG_DIR=/tmp/limo_cloud_test_logs;ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=192")
endif()

add_executable(benchmark_projection EXCLUDE_FROM_ALL test/benchmark_projection.cpp)
target_link_libraries(benchmark_projection limo_cloud_core)

ament_export_targets(limo_cloud_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs rclcpp sensor_msgs pluginlib tf2_ros)

ament_package()
