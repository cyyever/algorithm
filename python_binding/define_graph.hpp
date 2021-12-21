/*!
 * \file define_graph.hpp
 *
 * \brief
 */

#pragma once
#include <pybind11/pybind11.h>

#include "graph/dag.hpp"
#include "graph/graph.hpp"
#include "improved_pyobject.hpp"
namespace py = pybind11;
inline void define_graph(py::module_ &m) {
  using edge_type = cyy::algorithm::edge<py::object, double>;
  py::class_<edge_type>(m, "edge").def(
      py::init<py::object, py::object, double>());
  using graph_type = cyy::algorithm::graph<py::object, double>;
  py::class_<graph_type>(m, "graph")
      .def(py::init<>())
      .def("add_vertex", &graph_type::add_vertex,
           py::call_guard<py::gil_scoped_release>())
      .def("add_edge", &graph_type::add_edge,
           py::call_guard<py::gil_scoped_release>())
      .def("empty", &graph_type::empty,
           py::call_guard<py::gil_scoped_release>());
  using directed_graph_type =
      cyy::algorithm::directed_graph<py::object, double>;
  py::class_<directed_graph_type>(m, "directed_graph")
      .def(py::init<>())
      .def("add_vertex", &directed_graph_type::add_vertex,
           py::call_guard<py::gil_scoped_release>())
      .def("add_edge", &directed_graph_type::add_edge,
           py::call_guard<py::gil_scoped_release>())
      .def("empty", &directed_graph_type::empty,
           py::call_guard<py::gil_scoped_release>());
  using DAG_type = cyy::algorithm::DAG<py::object, double>;
  py::class_<DAG_type, directed_graph_type>(m, "DAG").def(
      py::init<directed_graph_type, bool>());
}
