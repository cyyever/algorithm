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
    using matrix_type = polyhedron<number_type>::matrix_type;
    using vector_type = polyhedron<number_type>::vector_type;

    linear_program(vector_type c_, polyhedron<number_type> A_b_)
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
    bool is_dually_feasible_basis(const basis_type &basis) const {
      return c.dot(A_b.get_extreme_point(basis)) <= 0;
    }

    template <bool b_is_0 = false>
    std::optional<basis_type> primal_simplex_phase_2(const basis_type &basis) {
      assert(is_primally_feasible_basis(basis));
      auto tableau = get_tableau_form(basis);
      while (true) {
        auto const &y = get_y(tableau);
        auto it = std::ranges::find_if(y, [](auto const &n) { return n > 0; });
        if (it == y.end()) {
          assert(is_primally_feasible_basis(basis));
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
          if constexpr (b_is_0) {
            pivot_row = row_idx;
            break;
          } else {

            if (min_lambda == -1) {
              min_lambda = -row(Eigen::last) / row(pivot_col);
              pivot_row = row_idx;
            } else {
              auto lambda = -row(Eigen::last) / row(pivot_col);
              if (lambda < min_lambda) {
                pivot_row = row_idx;
                min_lambda = lambda;
              }
            }
            assert(min_lambda >= 0);
          }
        }
        if (min_lambda < 0) {
          break;
        }
        basis.rease(pivot_col);
        basis.add(pivot_row);
        do_pivot_col_operation(tableau, pivot_row, pivot_col);
      }
      return {};
    }

    template <bool c_is_0 = false>
    std::optional<basis_type> dual_simplex_phase_2(const basis_type &basis) {
      assert(is_dually_feasible_basis(basis));
      auto tableau = get_tableau_form(basis);

      while (true) {
        auto b = tableau.topLeftCorner(1, get_A().cols());
        auto it = std::ranges::find_if(b, [](auto const &n) { return n < 0; });
        if (it == b.end()) {
          assert(is_dually_feasible_basis(basis));
          return basis;
        }
        auto pivot_row = std::advance(b.begin(), it);
        number_type min_lambda = -1;
        std::vector<int> pivot_cols;
        for (auto col_idx = 0; col_idx + 1 < tableau.cols(); col_idx++) {
          auto const &col = tableau.col(col_idx);
          if (col(pivot_row) <= 0) {
            continue;
          }
          if constexpr (c_is_0) {
            pivot_cols.push_back(col_idx);
          } else {

            if (min_lambda == -1) {
              min_lambda = -col(0) / col(pivot_row);
              pivot_cols.push_back(col_idx);
            } else {
              auto lambda = -col(0) / col(pivot_row);
              if (lambda == min_lambda) {
                pivot_cols.push_back(col_idx);
              } else if (lambda < min_lambda) {
                pivot_cols.clear();
                pivot_cols.push_back(col_idx);
                min_lambda = lambda;
              }
            }
            assert(min_lambda >= 0);
          }
        }
        if (min_lambda < 0) {
          break;
        }
        // choose pivot_col from lexicographical order
        auto pivot_col = *std::ranges::min_element(
            pivot_cols, [](auto const &v1, auto const &v2) {
              for (int i = 0; i < v1.rows(); i++) {
                if (v1(i) < v2(i)) {
                  return true;
                }
                if (v2(i) < v1(i)) {
                  return false;
                }
              }
              return false;
            });

        basis.rease(pivot_row);
        basis.add(pivot_col);
        do_pivot_col_operation(tableau, pivot_row, pivot_col);
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
    auto &get_b(matrix_type &tableau) const {
      return tableau.bottomRightCorner(get_b().rows(), 1);
    }

    auto &get_y(matrix_type &tableau) const {
      return tableau.topLeftCorner(1, get_A().cols());
    }

    static void do_pivot_col_operation(matrix_type &tableau, int pivot_row,
                                       int pivot_col) {
      tableau.cols(pivot_col) /= tableau(pivot_row, pivot_col);
      for (int col_idx = 0; col_idx < tableau.cols(); col_idx++) {
        tableau.col(col_idx) -=
            tableau(pivot_row, col_idx) * tableau.col(pivot_col);
      }
    }

  private:
    vector_type c;
    polyhedron<number_type> A_b;
    std::optional<matrix_type> kernel_space;
  };

} // namespace cyy::algorithm
