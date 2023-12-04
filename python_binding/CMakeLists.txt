file(GLOB SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
find_package(pybind11 CONFIG REQUIRED)
pybind11_add_module(cyy_algorithm ${SOURCE})
target_link_libraries(cyy_algorithm PUBLIC CyyAlgorithmLib)
