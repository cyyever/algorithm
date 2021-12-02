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

    std::optional<vector_type> solve_by_primal_simplex() const {
      auto primal_feasible_basis_opt = primal_simplex_phase_1();
      if (!primal_feasible_basis_opt.has_value()) {
        return {};
      }
      primal_feasible_basis_opt =
          primal_simplex_phase_2(*primal_feasible_basis_opt);
      if (!primal_feasible_basis_opt.has_value()) {
        return {};
      }
      return A_b.get_extreme_point(*primal_feasible_basis_opt);
    }

    std::optional<vector_type> solve_by_dual_simplex() const {
      auto dual_feasible_basis_opt = dual_simplex_phase_1();
      if (!dual_feasible_basis_opt.has_value()) {
        return {};
      }
      dual_feasible_basis_opt = dual_simplex_phase_2(*dual_feasible_basis_opt);
      if (!dual_feasible_basis_opt.has_value()) {
        return {};
      }
      return A_b.get_extreme_point(*dual_feasible_basis_opt);
    }

  private:
    template <bool b_is_0 = false>
    bool is_primally_feasible_basis(const basis_type &basis) const {
      return A_b.is_feasible<b_is_0>(A_b.get_extreme_point<b_is_0>(basis));
    }
    template <bool c_is_0 = false>
    bool is_dually_feasible_basis(const basis_type &basis) const {
      if constexpr (c_is_0) {
        return true;
      } else {
        auto y =
            (c.transpose() * A_b.get_basis_matrix(basis).inverse()).reshaped();
        return std::ranges::all_of(y, [](auto const &a) { return a <= 0; });
      }
    }

    std::optional<basis_type> primal_simplex_phase_1() const {
      return dual_simplex_phase_2<true>(A_b.get_basis());
    }
    std::optional<basis_type> dual_simplex_phase_1() const {
      return primal_simplex_phase_2<true>(A_b.get_basis());
    }

    template <bool b_is_0 = false>
    std::optional<basis_type> primal_simplex_phase_2(basis_type basis) const {
      assert(is_primally_feasible_basis<b_is_0>(basis));
      auto tableau = get_tableau_form(basis);
      while (true) {
        auto const &y = get_y(tableau).reshaped();
        auto it = std::ranges::find_if(y, [](auto const &n) { return n > 0; });
        if (it == y.end()) {
          return basis;
        }
        auto pivot_col = std::distance(y.begin(), it);
        number_type min_lambda = -1;
        int pivot_row = -1;
        for (auto row_idx = 1; row_idx < tableau.rows(); row_idx++) {
          auto const &row = tableau.row(row_idx);
          if (row(pivot_col) >= 0) {
            continue;
          }
          if constexpr (b_is_0) {
            pivot_row = row_idx;
          } else {
          auto lambda = -row(Eigen::last) / row(pivot_col);

          if (min_lambda == -1 || lambda < min_lambda) {
            min_lambda = lambda;
            pivot_row = row_idx;
          }
          assert(min_lambda >= 0);
          }
        }
        if (pivot_row< 0) {
          break;
        }
        do_pivot_col_operation(tableau, pivot_row, pivot_col);
        auto old_row = get_nth_basis(basis, pivot_col);
        assert(pivot_row - 1 != old_row);
        assert(basis.contains(old_row));
        basis.erase(old_row);
        basis.insert(pivot_row - 1);
        assert(tableau == get_tableau_form(basis));
        assert(is_primally_feasible_basis<b_is_0>(basis));
      }
      return {};
    }

    template <bool c_is_0 = false>
    std::optional<basis_type> dual_simplex_phase_2(basis_type basis) const {
      assert(is_dually_feasible_basis<c_is_0>(basis));
      auto tableau = get_tableau_form(basis);

      while (true) {
        auto const b = get_b(tableau).reshaped();
        auto it = std::ranges::find_if(b, [](auto const &n) { return n < 0; });
        if (it == b.end()) {
          return basis;
        }
        auto pivot_row = std::distance(b.begin(), it) + 1;
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
            pivot_cols, [&tableau](auto const &U, auto const &V) {
              auto v1 = tableau.col(U).reshaped();
              auto v2 = tableau.col(V).reshaped();
              for (int i = 0; i < v1.size(); i++) {
                if (v1(i) < v2(i)) {
                  return true;
                }
                if (v2(i) < v1(i)) {
                  return false;
                }
              }
              return false;
            });

        assert(pivot_row - 1 != pivot_col);
        auto old_row = get_nth_basis(basis, pivot_row - 1);
        assert(old_row != pivot_col);
        basis.erase(old_row);
        basis.insert(pivot_col);
        do_pivot_col_operation(tableau, pivot_row, pivot_col);
        assert(is_dually_feasible_basis<c_is_0>(basis));
      }
      return {};
    }

    matrix_type get_tableau_form(const basis_type &basis) const {
      matrix_type tableau;
      tableau.resize(get_A().rows() + 1, get_A().cols() + 1);
      get_y(tableau) = c.transpose();
      tableau.bottomLeftCorner(get_A().rows(), get_A().cols()) = get_A();
      tableau(0, tableau.cols() - 1) = 0;
      get_b(tableau) = get_b();
      auto [sub_A, sub_b] = A_b.get_subset(basis);
      auto basis_inverse = sub_A.inverse();
      auto x = basis_inverse * sub_b;
      tableau.rightCols(1) -= tableau.leftCols(get_A().cols()) * x;
      tableau.leftCols(get_A().cols()) *= basis_inverse;
      assert(tableau(*basis.begin() + 1, 0) == 1);
      return tableau;
    }
    auto const &get_A() const { return A_b.get_A(); }
    auto const &get_b() const { return A_b.get_b(); }
    auto get_b(matrix_type &tableau) const {
      return tableau.bottomRightCorner(get_b().rows(), 1);
    }

    auto get_y(matrix_type &tableau) const {
      return tableau.topLeftCorner(1, get_A().cols());
    }

    static void do_pivot_col_operation(matrix_type &tableau, int pivot_row,
                                       int pivot_col) {
      tableau.col(pivot_col) /= tableau(pivot_row, pivot_col);
      assert(tableau(pivot_row, pivot_col) == 1);
      for (int col_idx = 0; col_idx < tableau.cols(); col_idx++) {
        if (col_idx == pivot_col) {
          continue;
        }
        tableau.col(col_idx) -=
            tableau(pivot_row, col_idx) * tableau.col(pivot_col);
      }
      assert(tableau(pivot_row, pivot_col) == 1);
    }

    static int get_nth_basis(const basis_type &basis, int n) {
      auto it = basis.begin();
      std::advance(it, n);
      return *it;
    }

  private:
    vector_type c;
    polyhedron<number_type> A_b;
    std::optional<matrix_type> kernel_space;
  };

} // namespace cyy::algorithm
