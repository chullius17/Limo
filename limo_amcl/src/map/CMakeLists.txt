add_library(limo_amcl_map SHARED
  map.c
  map_store.c
  map_range.c
  map_draw.c
  map_cspace.cpp
)

install(TARGETS
  limo_amcl_map
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
