/*!
 * \file polyhedron.hpp
 *
 * \brief
 */
#pragma once

#include <algorithm>
#include <ranges>
#include <vector>

#include <cyy/math/algebra/matrix.hpp>

namespace cyy::algorithm {
  // A polyhedron satisfying Ax<=b
  template <typename number_type = double> class polyhedron {
  public:
    using basis_type = std::set<int>;
    using matrix_type = cyy::math::la::matrix<number_type>;
    using vector_type = cyy::math::la::vector<number_type>;
    polyhedron(matrix_type A_, vector_type b_)
        : A(std::move(A_)), b(std::move(b_)) {
      if (A.rows() != b.rows()) {
        throw std::invalid_argument("mismatched A and b");
      }
      if (A.size() == 0) {
        throw std::invalid_argument("A is empty");
      }
    }

    std::pair<matrix_type, polyhedron<number_type>> decompose() {
      // If A is not full rank, decompose it into U and U_complement, where
      // U=kernel(A). If A is full rank, then U would be {0}
      Eigen::FullPivLU<matrix_type> lu_decomp(A);
      auto U = lu_decomp.kernel();
      auto pointed_polyhedron = *this;
      pointed_polyhedron.A.resize(A.rows() + U.cols() * 2, A.cols());
      auto U_transpose = U.transpose();
      pointed_polyhedron.A << A, U_transpose, (U_transpose * -1);
      pointed_polyhedron.b.conservativeResize(pointed_polyhedron.A.rows(), 1);
      for (auto row_num = A.rows(); row_num < pointed_polyhedron.A.rows();
           row_num++) {
        pointed_polyhedron.b.row(row_num).setConstant(0);
      }
      assert(pointed_polyhedron.A.rows() == pointed_polyhedron.b.rows());
      return {std::move(U), std::move(pointed_polyhedron)};
    }

    auto const &get_A() const { return A; }
    auto const &get_b() const { return b; }

    bool full_rank() const {
      return this->rank() == std::min(A.rows(), A.cols());
    }

    auto rank() const { return Eigen::FullPivLU<matrix_type>(A).rank(); }

    auto get_basis() const {
      // return a basis of A.
      auto transpose = A.transpose();
      auto D = transpose.fullPivLu().image(transpose).transpose().eval();
      basis_type basis;
      for (int j = 0; j < D.rows(); j++) {
        for (int i = 0; i < A.rows(); i++) {
          if (D.row(j) == A.row(i)) {
            basis.insert(i);
            break;
          }
        }
      }
      return basis;
    }

    bool is_feasible(const vector_type &x) const { return A * x <= b; }

    auto get_extreme_point(const basis_type &basis) const {
      return get_basis_matrix(basis).inverse() * b(basis);
    }

    auto get_basis_matrix(const basis_type &basis) const {
      return A(basis, Eigen::all);
    }
    auto get_subset(const basis_type &basis) const {
      return std::pair{get_basis_matrix(basis), b(basis)};
    }

  private:
    matrix_type A;
    vector_type b;
  };

} // namespace cyy::algorithm
