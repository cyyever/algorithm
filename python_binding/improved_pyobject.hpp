/*!
 * \file define_heap.hpp
 *
 * \brief
 */

#pragma once
#include <pybind11/pybind11.h>

namespace py = pybind11;
/* namespace cyy { */
/*   class improved_pyobject:public py::object { */
/*     public: */
/*       using py::object::object; */
/*       using py::handle::check_; */
/*       auto operator==(const auto &rhs) const { */
/*         return this->equal(rhs); */
/*       } */
/*   }; */
/* } */

namespace std {
  template <> struct hash<py::object> {
    std::size_t operator()(const auto &x) const noexcept {
      return pybind11::hash(x);
    }
  };
} // namespace std
