/*!
 * \file define_graph.hpp
 *
 * \brief
 */

#pragma once
#include <pybind11/pybind11.h>

#include "graph/dag.hpp"
#include "graph/graph.hpp"
#include "graph/tree.hpp"
#include "improved_pyobject.hpp"
namespace py = pybind11;
inline void define_graph(py::module_ &m) {
  using edge_type = cyy::algorithm::edge<py::object, double>;
  py::class_<edge_type>(m, "Edge").def(
      py::init<py::object, py::object, double>(), py::arg("first"),
      py::arg("second"), py::arg("weight") = 1);
  using graph_type = cyy::algorithm::graph<py::object, double>;
  py::class_<graph_type>(m, "Graph")
      .def(py::init<>())
      .def("add_vertex", &graph_type::add_vertex)
      .def("add_edge", &graph_type::add_edge)
      .def("empty", &graph_type::empty);
  using directed_graph_type =
      cyy::algorithm::directed_graph<py::object, double>;
  py::class_<directed_graph_type>(m, "DirectedGraph")
      .def(py::init<>())
      .def("add_vertex", &directed_graph_type::add_vertex)
      .def("add_edge", &directed_graph_type::add_edge)
      .def("empty", &directed_graph_type::empty);
  using DAG_type = cyy::algorithm::DAG<py::object, double>;
  py::class_<DAG_type, directed_graph_type>(m, "DAG")
      .def(py::init<directed_graph_type, bool>())
      .def("get_topological_ordering", &DAG_type::get_topological_ordering);
  using directed_tree_type =
      cyy::algorithm::directed_tree<py::object, double>;
  py::class_<directed_tree_type,DAG_type>(m, "DirectedTree")
      .def(py::init<>());
}
