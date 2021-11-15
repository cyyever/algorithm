cmake_minimum_required(VERSION 3.20)

project(CyyAlgorithmLib LANGUAGES CXX)

include(cmake/all.cmake)

option(BUILD_PYTHON_BINDING "Build python binding" OFF)
file(GLOB_RECURSE SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/src/**/*.cpp)

find_package(Boost REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)
find_package(Eigen3)
find_package(CyyNaiveLib REQUIRED)

add_library(CyyAlgorithmLib ${SOURCE})

find_package(range-v3 REQUIRED)
target_link_libraries(CyyAlgorithmLib PUBLIC range-v3::range-v3)
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  find_package(libcxx REQUIRED)
  target_link_libraries(CyyAlgorithmLib PUBLIC libcxx::libcxx)
endif()

target_link_libraries(CyyAlgorithmLib PUBLIC Boost::headers)
target_link_libraries(CyyAlgorithmLib INTERFACE CyyNaiveLib::util)
target_link_libraries(CyyAlgorithmLib INTERFACE CyyNaiveLib::log)

target_include_directories(
  CyyAlgorithmLib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
                         $<INSTALL_INTERFACE:include/cyy/algorithm>)

if(BUILD_PYTHON_BINDING)
  add_subdirectory(python_binding)
endif()

# test
add_subdirectory(test)

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
