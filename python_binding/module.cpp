#include <pybind11/pybind11.h>

#include "define_graph.hpp"
#include "define_heap.hpp"
namespace py = pybind11;

PYBIND11_MODULE(cyy_algorithm, m) {
  define_heap(m);
  define_graph(m);
}
