/*!
 * \file lp_test.cpp
 *
 * \brief
 */

#include <doctest/doctest.h>

#ifdef MY_MATH
#include "linear_programming/linear_program.hpp"

using namespace cyy::algorithm;
TEST_CASE("LP") {
  SUBCASE("has_optimal") {
    cyy::math::la::matrix<cyy::math::rational> A{
        {-1, 2}, {1, -1}, {-1, -2}, {-2, 1}, {3, 4}};
    cyy::math::la::vector<cyy::math::rational> b{{1}, {2}, {2}, {2}, {2}};
    cyy::math::la::vector<cyy::math::rational> c{{1}, {1}};

    linear_program LP(c, polyhedron(A, b));
    auto x_opt = LP.solve_by_primal_simplex();
    DOCTEST_REQUIRE(x_opt.has_value());
    DOCTEST_REQUIRE_EQ(x_opt.value()(0), cyy::math::rational(-6, 5));
    DOCTEST_REQUIRE_EQ(x_opt.value()(1), cyy::math::rational(-2, 5));
    x_opt = LP.solve_by_dual_simplex();
    DOCTEST_REQUIRE(x_opt.has_value());
    DOCTEST_REQUIRE_EQ(x_opt.value()(0), cyy::math::rational(-6, 5));
    DOCTEST_REQUIRE_EQ(x_opt.value()(1), cyy::math::rational(-2, 5));
  }
  SUBCASE("unbounded") {
    cyy::math::la::matrix<cyy::math::rational> A{{-1, 0}};
    cyy::math::la::vector<cyy::math::rational> b{{0}};
    cyy::math::la::vector<cyy::math::rational> c{{1}, {1}};

    linear_program LP(c, polyhedron(A, b));
    auto x_opt = LP.solve_by_primal_simplex();
    DOCTEST_REQUIRE(!x_opt.has_value());
  }
}
#endif
