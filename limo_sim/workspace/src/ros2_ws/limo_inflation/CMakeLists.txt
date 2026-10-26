cmake_minimum_required(VERSION 3.8)
project(limo_inflation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/border_follow_layer.cpp
  src/cost_profile.cpp
  src/local_lane_layer.cpp
  src/gaussian_layer.cpp
  src/path_distance_layer.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  nav2_costmap_2d
  nav2_msgs
  nav_msgs
  pluginlib
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

pluginlib_export_plugin_description_file(
  nav2_costmap_2d limo_inflation_plugins.xml
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES limo_inflation_plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cost_profile test/test_cost_profile.cpp)
  target_link_libraries(test_cost_profile ${PROJECT_NAME})
  target_include_directories(test_cost_profile PUBLIC include)

  ament_add_gtest(test_layer test/test_layer.cpp)
  ament_target_dependencies(test_layer nav2_costmap_2d pluginlib)

  ament_add_gtest(test_local_lane_layer test/test_local_lane_layer.cpp
    ENV ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=1)
  target_link_libraries(test_local_lane_layer ${PROJECT_NAME})
  ament_target_dependencies(test_local_lane_layer nav2_costmap_2d rclcpp)

  ament_add_gtest(test_gaussian_layer test/test_gaussian_layer.cpp
    ENV ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=1)
  target_link_libraries(test_gaussian_layer ${PROJECT_NAME})
  ament_target_dependencies(test_gaussian_layer nav2_costmap_2d rclcpp)

  ament_add_gtest(test_path_distance_layer test/test_path_distance_layer.cpp
    ENV ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=1)
  target_link_libraries(test_path_distance_layer ${PROJECT_NAME})
  ament_target_dependencies(test_path_distance_layer nav2_costmap_2d rclcpp tf2_ros)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(nav2_costmap_2d nav2_msgs nav_msgs pluginlib rclcpp tf2 tf2_geometry_msgs tf2_ros)
ament_package()
