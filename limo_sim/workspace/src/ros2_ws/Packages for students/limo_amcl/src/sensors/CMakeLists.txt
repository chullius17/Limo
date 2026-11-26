add_library(limo_amcl_sensors SHARED
  cv_likelihood_model.cpp
  cv_point_cloud.cpp
  laser/laser.cpp
  laser/beam_model.cpp
  laser/likelihood_field_model.cpp
  laser/likelihood_field_model_prob.cpp
)
ament_target_dependencies(limo_amcl_sensors nav_msgs sensor_msgs tf2)
# map_update_cspace
target_link_libraries(limo_amcl_sensors limo_amcl_pf limo_amcl_map)

install(TARGETS
  limo_amcl_sensors
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
