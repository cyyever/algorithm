#include <pybind11/pybind11.h>

#include "define_heap.hpp"
namespace py = pybind11;

PYBIND11_MODULE(cyy_algorithm, m) { define_heap(m); }
