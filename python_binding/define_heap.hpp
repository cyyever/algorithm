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
      .def("top_key", &heap_type::top_key)
      .def("top_data", &heap_type::top_data)
      .def("size", &heap_type::size)
      .def("empty", &heap_type::empty)
      .def("contains", &heap_type::contains)
      .def("insert",
           py::overload_cast<py::object, py::object>(&heap_type::insert))
      .def("change_key", &heap_type::change_key)
      .def("pop", &heap_type::pop);
}
