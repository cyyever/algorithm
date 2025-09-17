cmake_minimum_required(VERSION 4.1.0)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPERIMENTAL_CXX_IMPORT_STD "d0edc3af-4c50-42ea-a356-e2862fe7a444")
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_MODULE_STD 1)

if(NOT WIN32)
  set(CMAKE_CXX_COMPILER clang++)
endif()

project(CyyAlgorithmLib LANGUAGES CXX)

include(cmake/all.cmake)

option(BUILD_PYTHON_BINDING "Build python binding" OFF)
file(GLOB_RECURSE src_files ${CMAKE_CURRENT_SOURCE_DIR}/src/**/*.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE headers ${CMAKE_CURRENT_SOURCE_DIR}/src/**/*.hpp
     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp)

find_package(Boost CONFIG)
find_package(Eigen3)
find_package(spdlog)
find_package(CyyNaiveLib)

option(BUILD_OBJECT_LIB "Build CMake object libraries" OFF)

if(BUILD_OBJECT_LIB)
  add_library(CyyAlgorithmLib OBJECT ${src_files})
else()
  add_library(CyyAlgorithmLib ${src_files})
endif()

if(TARGET Eigen3::eigen)
  target_link_libraries(CyyAlgorithmLib INTERFACE Eigen3::eigen)
endif()
if(TARGET Boost::headers)
  target_link_libraries(CyyAlgorithmLib PUBLIC Boost::headers)
endif()
if(TARGET CyyNaiveLib::util)
  target_link_libraries(CyyAlgorithmLib INTERFACE CyyNaiveLib::util)
  target_link_libraries(CyyAlgorithmLib INTERFACE CyyNaiveLib::log)
endif()

target_sources(
  CyyAlgorithmLib
  PUBLIC FILE_SET
         public_headers
         TYPE
         HEADERS
         BASE_DIRS
         src
         FILES
         ${headers})

if(BUILD_PYTHON_BINDING)
  add_subdirectory(python_binding)
endif()

# test
option(BUILD_TESTING "Build test" ON)
if(BUILD_TESTING)
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
  FILE_SET public_headers
  DESTINATION include/cyy/algorithm/${submodule_name})

# install cmake config file
install(EXPORT CyyAlgorithmLibConfig DESTINATION lib/cmake/CyyAlgorithmLib)
