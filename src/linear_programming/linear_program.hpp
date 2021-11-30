/*!
 * \file polyhedron.hpp
 *
 * \brief
 */
#pragma once

#include <algorithm>
#include <optional>
#include <ranges>
#include <vector>

#include "polyhedron.hpp"

namespace cyy::algorithm {
  // min c^T x  s.t. Ax<=b
  template <typename number_type = double> class linear_program {
  public:
    linear_program(cyy::math::la::vector<number_type> c_,
                   polyhedron<number_type> A_b_)
        : c(std::move(c_)), A_b(std::move(A_b_)) {
      if (c.rows() != A_b.get_A().cols()) {
        throw std::invalid_argument("mismatched c and A_b");
      }
      if (!A_b.full_rank()) {
        std::tie(kernel_space, A_b) = A_b.decompose();
      }
    }

  private:
    cyy::math::la::vector<number_type> c;
    polyhedron<number_type> A_b;
    std::optional<cyy::math::la::matrix<number_type>> kernel_space;
  };

} // namespace cyy::algorithm
