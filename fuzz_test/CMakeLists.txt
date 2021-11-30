file(GLOB test_sources
  ${CMAKE_CURRENT_SOURCE_DIR}/*/*.cpp
  )

set(ENV{FUZZING_TIMEOUT} 10)
set(ENV{MAX_FUZZING_TIME} 10)
foreach(test_source IN LISTS test_sources)
  get_filename_component(test_prog ${test_source} NAME_WE)
  # add_executable(${test_prog} ${test_source} ${CMAKE_CURRENT_SOURCE_DIR}/helper.cpp)
  target_link_libraries(${test_prog} PRIVATE CyyAlgorithmLib)
  add_fuzzing(TARGET ${test_prog})
endforeach()
