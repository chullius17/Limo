cmake_minimum_required(VERSION 3.8)
project(limo_dwb_critics)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
  message(FATAL_ERROR
    "Use GCC 9 or newer for this C++17 plugin. GCC 8 std::filesystem headers "
    "are incompatible with the newer libstdc++ used by the Foxy runtime. "
    "Rebuild with --cmake-clean-cache --cmake-args "
    "-DCMAKE_CXX_COMPILER=/usr/bin/g++-9 -DCMAKE_C_COMPILER=/usr/bin/gcc-9")
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_critics REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ackermann_kinematics_critic.cpp
  src/sampling_mpc.cpp
  src/ackermann_mpc_controller.cpp
  src/mppi_cost_critics.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  dwb_core
  dwb_critics
  dwb_msgs
  nav2_core
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(
  dwb_core limo_dwb_critics_plugins.xml
)
pluginlib_export_plugin_description_file(
  nav2_core limo_mpc_controller_plugins.xml
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES limo_dwb_critics_plugins.xml DESTINATION share/${PROJECT_NAME})
install(FILES limo_mpc_controller_plugins.xml README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ackermann_kinematics test/test_ackermann_kinematics.cpp)
  target_link_libraries(test_ackermann_kinematics ${PROJECT_NAME})
  target_include_directories(test_ackermann_kinematics PUBLIC include)

  ament_add_gtest(test_plugin test/test_plugin.cpp)
  ament_target_dependencies(test_plugin dwb_core nav2_core pluginlib)

  ament_add_gtest(test_sampling_mpc test/test_sampling_mpc.cpp)
  target_link_libraries(test_sampling_mpc ${PROJECT_NAME})

  ament_add_gtest(test_mpc_controller test/test_mpc_controller.cpp)
  target_link_libraries(test_mpc_controller ${PROJECT_NAME})

  ament_add_gtest(test_mppi_costs test/test_mppi_costs.cpp)
  target_link_libraries(test_mppi_costs ${PROJECT_NAME})
  target_compile_definitions(test_mppi_costs PRIVATE
    LIMO_MPC_PARAMS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../limo_controller/config/control_real.yaml")
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(dwb_core dwb_critics dwb_msgs nav2_core pluginlib rclcpp)
ament_package()
