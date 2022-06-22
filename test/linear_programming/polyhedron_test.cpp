/*!
 * \file polyhedron_test.cpp
 *
 * \brief
 */

#include <doctest/doctest.h>

#include "linear_programming/polyhedron.hpp"

#ifdef MY_MATH
using namespace cyy::algorithm;
TEST_CASE("polyhedron") {
  SUBCASE("decompose and basis") {
    cyy::math::la::matrix<cyy::math::rational> A{{1, 0}, {1, 0}, {1, 0}};
    cyy::math::la::vector<cyy::math::rational> b{{1}, {1}, {1}};
    cyy::algorithm::polyhedron polyhedron(A, b);

    DOCTEST_REQUIRE(!polyhedron.full_rank());
    DOCTEST_REQUIRE_EQ(polyhedron.rank(), 1);
    auto [U, pointed_polyhedron] = polyhedron.decompose();
    auto full_rank = pointed_polyhedron.full_rank();
    DOCTEST_REQUIRE(full_rank);
    auto basis = pointed_polyhedron.get_basis();
    auto rank = pointed_polyhedron.rank();
    DOCTEST_REQUIRE_EQ(basis.size(), rank);
  }
}
#endif
