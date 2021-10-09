cmake_minimum_required(VERSION 3.20)

project(MyAlgorithmLib LANGUAGES CXX)

include(cmake/all.cmake)

file(GLOB_RECURSE SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/src/**/*.cpp)

find_package(Boost REQUIRED)
find_package(range-v3 REQUIRED)

add_library(MyAlgorithmLib ${SOURCE})

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  find_package(libcxx REQUIRED)
  target_link_libraries(MyAlgorithmLib PRIVATE libcxx::libcxx)
endif()

target_link_libraries(MyAlgorithmLib PUBLIC Boost::headers)
target_link_libraries(MyAlgorithmLib PUBLIC range-v3::range-v3)

target_include_directories(
  MyAlgorithmLib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
                        $<INSTALL_INTERFACE:include/cyy/algorithm>)

# test
add_subdirectory(test)

option(BUILD_FUZZING "Build fuzzing" OFF)
if(BUILD_FUZZING)
  add_subdirectory(fuzz_test)
endif()

# install lib
install(
  TARGETS MyAlgorithmLib
  EXPORT MyAlgorithmLibConfig
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
install(EXPORT MyAlgorithmLibConfig DESTINATION lib/cmake/MyAlgorithmLib)
