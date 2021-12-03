/*!
 * \file helper.hpp
 *
 * \brief fuzzing helper functions
 * \author cyy
 * \date 2019-02-12
 */
#pragma once

#include <span>

#include "linear_programming/linear_program.hpp"

template <typename number_type = double>
cyy::algorithm::linear_program<number_type>
generate_linear_program(std::span<const uint8_t> &data) {

  cyy::math::la::matrix<number_type> A;
  cyy::math::la::vector<number_type> b;
  cyy::math::la::vector<number_type> c;
  int m = 0;
  int n = 0;
  if (data.size() >= 2) {
    m = data[0];
    n = data[1];
    data = data.subspan(2);
  }
  A.resize(m, n);
  b.resize(n);
  c.resize(n);
  if (data.size() >= m * n + n + n) {
    for (int i = 0; i < m; i++) {
      for (int j = 0; j < n; j++) {
        A(m, n) = data[0];
        data = data.subspan(1);
      }
    }
    for (int j = 0; j < n; j++) {
      b(j) = data[0];
      data = data.subspan(1);
      c(j) = data[0];
      data = data.subspan(1);
    }
  }
  return cyy::algorithm::linear_program(c, cyy::algorithm::polyhedron(A, b));
}
