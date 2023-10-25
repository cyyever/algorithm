cmake_minimum_required(VERSION 3.20)

project(CyyAlgorithmLib LANGUAGES CXX)

include(cmake/all.cmake)

option(BUILD_PYTHON_BINDING "Build python binding" OFF)
file(GLOB_RECURSE SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/src/**/*.cpp)

find_package(Boost)
find_package(spdlog REQUIRED)
find_package(Eigen3)
find_package(CyyNaiveLib REQUIRED)

add_library(CyyAlgorithmLib ${SOURCE})

if(TARGET Eigen3::eigen)
  target_link_libraries(CyyAlgorithmLib INTERFACE Eigen3::eigen)
endif()
if(TARGET Boost::headers)
  target_link_libraries(CyyAlgorithmLib PUBLIC Boost::headers)
endif()
target_link_libraries(CyyAlgorithmLib INTERFACE CyyNaiveLib::util)
target_link_libraries(CyyAlgorithmLib INTERFACE CyyNaiveLib::log)
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  target_compile_options(CyyAlgorithmLib PUBLIC -fexperimental-library)
endif()

target_include_directories(
  CyyAlgorithmLib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
                         $<INSTALL_INTERFACE:include/cyy/algorithm>)

if(BUILD_PYTHON_BINDING)
  add_subdirectory(python_binding)
endif()

# test
option(BUILD_TEST "Build test" OFF)
if(BUILD_TEST)
  add_subdirectory(test)
endif()

option(BUILD_FUZZING "Build fuzzing" OFF)
if(BUILD_FUZZING)
  add_subdirectory(fuzz_test)
endif()

# install lib
install(
  TARGETS CyyAlgorithmLib
  EXPORT CyyAlgorithmLibConfig
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib COMPONENT library
  INCLUDES
  DESTINATION include)

install(
  DIRECTORY src/
  DESTINATION include/cyy/algorithm
  FILES_MATCHING
  PATTERN "*.hpp")

# install cmake config file
install(EXPORT CyyAlgorithmLibConfig DESTINATION lib/cmake/CyyAlgorithmLib)
