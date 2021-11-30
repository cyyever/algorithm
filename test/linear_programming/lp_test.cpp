/*!
 * \file lp_test.cpp
 *
 * \brief
 */

#include <doctest/doctest.h>

#include "linear_programming/linear_program.hpp"

using namespace cyy::algorithm;
TEST_CASE("LP") {
  cyy::math::la::matrix<cyy::math::rational> A{
      {-1, 2}, {1, -1}, {-1, -2}, {-2, 1}, {3, 4}};
  cyy::math::la::vector<cyy::math::rational> b{{1}, {2}, {2}, {2}, {2}};
  cyy::math::la::vector<cyy::math::rational> c{{1}, {1}};
  linear_program LP(c, polyhedron(A, b));
}
