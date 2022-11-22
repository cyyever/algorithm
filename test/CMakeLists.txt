find_package(doctest REQUIRED)
find_package(CyyMathLib QUIET)

file(GLOB test_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/*/*.cpp)

foreach(test_source IN LISTS test_sources)
  get_filename_component(test_prog ${test_source} NAME_WE)
  add_executable(${test_prog} ${test_source})
  target_link_libraries(${test_prog} PRIVATE CyyAlgorithmLib)
  if(TARGET CyyMathLib)
    target_link_libraries(${test_prog} PRIVATE CyyMathLib)
    target_compile_definitions(${test_prog} PRIVATE CYY_MATH)
  endif()
  target_link_libraries(${test_prog} PRIVATE doctest::doctest)
  target_compile_definitions(${test_prog}
                             PRIVATE DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN)
  target_compile_definitions(
    ${test_prog} PRIVATE DOCTEST_CONFIG_NO_EXCEPTIONS_BUT_WITH_ALL_ASSERTS)
  add_test_with_runtime_analysis(TARGET ${test_prog} HELGRIND OFF TSAN OFF)
endforeach()
