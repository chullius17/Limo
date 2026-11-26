add_library(limo_amcl_motion SHARED
  omni_motion_model.cpp
  differential_motion_model.cpp
  motion_model.cpp
)
target_link_libraries(limo_amcl_motion limo_amcl_pf)
ament_target_dependencies(limo_amcl_motion
  nav2_util
)

install(TARGETS
  limo_amcl_motion
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
