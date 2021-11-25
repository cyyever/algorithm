/*!
 * \file define_heap.hpp
 *
 * \brief
 */

#pragma once
#include <pybind11/pybind11.h>

#include "heap.hpp"
#include "improved_pyobject.hpp"
namespace py = pybind11;
inline void define_heap(py::module_ &m) {
  using heap_type = cyy::algorithm::heap<py::object, py::object>;
  py::class_<heap_type>(m, "Heap")
      .def(py::init<>())
      .def("top_key", &heap_type::top_key,
           py::call_guard<py::gil_scoped_release>())
      .def("top", &heap_type::top, py::call_guard<py::gil_scoped_release>())
      .def("size", &heap_type::size, py::call_guard<py::gil_scoped_release>())
      .def("empty", &heap_type::empty, py::call_guard<py::gil_scoped_release>())
      .def("contains", &heap_type::contains,
           py::call_guard<py::gil_scoped_release>())
      .def("insert",
           py::overload_cast<py::object, py::object>(&heap_type::insert),
           py::call_guard<py::gil_scoped_release>())
      .def("change_key", &heap_type::change_key,
           py::call_guard<py::gil_scoped_release>())
      .def("pop", &heap_type::pop, py::call_guard<py::gil_scoped_release>());
}
