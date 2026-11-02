cmake_minimum_required(VERSION 3.8)
project(limo_rviz_plugins)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(${PROJECT_NAME} SHARED src/polygon_display.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} pluginlib rviz_common rviz_default_plugins)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
# Foxy's TF buffer retains type-erased callback destructors after RViz unloads
# its displays. Keep their template code mapped until process exit so buffer
# teardown cannot jump into an unloaded display library on Ctrl+C.
if(UNIX AND NOT APPLE)
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-Wl,-z,nodelete")
endif()
pluginlib_export_plugin_description_file(rviz_common plugins.xml)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Threads REQUIRED)
  ament_add_gtest(test_latest_message test/test_latest_message.cpp)
  target_compile_features(test_latest_message PUBLIC cxx_std_14)
  target_include_directories(test_latest_message PRIVATE include)
  target_link_libraries(test_latest_message Threads::Threads)
endif()

ament_package()
