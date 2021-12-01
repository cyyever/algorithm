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
    polyhedron(cyy::math::la::matrix<number_type> A_,
               cyy::math::la::vector<number_type> b_)
        : A(std::move(A_)), b(std::move(b_)) {
      if (A.rows() != b.rows()) {
        throw std::invalid_argument("mismatched A and b");
      }
      if (A.size() == 0) {
        throw std::invalid_argument("A is empty");
      }
    }

    std::pair<cyy::math::la::matrix<number_type>, polyhedron<number_type>>
    decompose() {
      // If A is not full rank, decompose it into U and U_complement, where
      // U=kernel(A). If A is full rank, then U would be {0}
      Eigen::FullPivLU<cyy::math::la::matrix<number_type>> lu_decomp(A);
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

    auto rank() const {
      return Eigen::FullPivLU<cyy::math::la::matrix<number_type>>(A).rank();
    }

    auto get_basis_matrix() const {
      // return a basis matrix of A, i.e. n rows that is invertible.
      auto transpose = A.transpose();
      auto D = transpose.fullPivLu().image(transpose).transpose();
      return D;
    }

  private:
    cyy::math::la::matrix<number_type> A;
    cyy::math::la::matrix<number_type> b;
  };

} // namespace cyy::algorithm
