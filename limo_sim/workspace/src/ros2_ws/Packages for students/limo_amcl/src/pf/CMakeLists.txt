if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-gnu-folding-constant)
endif()

add_library(limo_amcl_pf SHARED
  pf.c
  pf_kdtree.c
  pf_pdf.c
  pf_vector.c
  eig3.c
  pf_draw.c
)

target_include_directories(limo_amcl_pf PRIVATE ../include)
if(HAVE_DRAND48)
  target_compile_definitions(limo_amcl_pf PRIVATE "HAVE_DRAND48")
endif()

install(TARGETS
  limo_amcl_pf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
