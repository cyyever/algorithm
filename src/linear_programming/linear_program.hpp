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
    using basis_type = polyhedron<number_type>::basis_type;

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
    bool is_primally_feasible_basis(const basis_type &basis) const {
      return A_b.is_feasible(A_b.get_extreme_point(basis));
    }
    std::optional<basis_type> primal_simplex_phase_2(const basis_type &basis) {
      auto tableau = get_tableau_form(basis);
      while (true) {
        auto y = tableau.topLeftCorner(1, get_A().cols());
        auto it = std::ranges::find_if(y, [](auto const &a) { return a > 0; });
        if (it == y.end()) {
          return basis;
        }
        auto pivot_col = std::advance(y.begin(), it);
        number_type min_lambda = -1;
        int pivot_row = -1;
        for (auto row_idx = 1; row_idx < tableau.rows(); row_idx++) {
          auto const &row = tableau.row(row_idx);
          if (row(pivot_col) >= 0) {
            continue;
          }
          if (min_lambda == -1) {
            min_lambda = -row.tail(1) / row(pivot_col);
            pivot_row = row_idx;
          } else {
            auto lambda = -row.tail(1) / row(pivot_col);
            if (lambda < min_lambda) {
              pivot_row = row_idx;
              min_lambda = lambda;
            }
          }
          assert(min_lambda >= 0);
        }
        if (min_lambda < 0) {
          break;
        }
        basis.rease(pivot_col);
        basis.add(pivot_row);
        tableau.cols(pivot_col) /= tableau(pivot_row, pivot_col);
        for (int col_idx = 0; col_idx < tableau.cols(); col_idx++) {
          tableau.col(col_idx) -=
              tableau(pivot_row, col_idx) * tableau.col(pivot_col);
        }
      }

      return {};
    }

    auto get_tableau_form(const basis_type &basis) {
      cyy::math::la::matrix<number_type> tableau;
      tableau.reshape(get_A().rows() + 1, get_A().cols() + 1);
      auto [sub_A, sub_b] = A_b.get_subset(basis);
      auto basis_inverse = sub_A.inverse();
      auto x = basis_inverse * sub_b;
      tableau.topLeftCorner(1, get_A().cols()) = c.transpose();
      tableau.bottomLeftCorner(get_A().rows(), get_A().cols()) = get_A();
      tableau.leftCols(get_A().cols()) *= basis_inverse;
      tableau.topRightCorner(1, 1) = -c.dot(x);
      tableau.bottomRightCorner(get_b().rows(), 1) = get_b() - get_A() * x;
    }
    auto const &get_A() const { return A_b.get_A(); }
    auto const &get_b() const { return A_b.get_b(); }

  private:
    cyy::math::la::vector<number_type> c;
    polyhedron<number_type> A_b;
    std::optional<cyy::math::la::matrix<number_type>> kernel_space;
  };

} // namespace cyy::algorithm
