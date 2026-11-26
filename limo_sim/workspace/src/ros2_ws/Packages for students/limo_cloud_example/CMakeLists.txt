cmake_minimum_required(VERSION 3.12)
project(limo_cloud_example)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(limo_cloud REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(limo_cloud_example_node src/main.cpp)
target_compile_features(limo_cloud_example_node PUBLIC cxx_std_14)
target_link_libraries(limo_cloud_example_node limo_cloud::limo_cloud_node_lib)
ament_target_dependencies(limo_cloud_example_node rclcpp)

install(TARGETS limo_cloud_example_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_example test TIMEOUT 90
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    ENV ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=194 ROS_LOG_DIR=/tmp/limo_cloud_example_test_logs)
endif()

ament_package()
