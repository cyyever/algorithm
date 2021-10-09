/*!
 * \file union_test.cpp
 *
 */

#include <vector>

#include <doctest/doctest.h>

#include "union_find.hpp"

using namespace cyy::algorithm;
TEST_CASE("union_find") {
  std::vector<int> data{1, 2, 3};
  cyy::algorithm::union_find<int> h(data);
  REQUIRE(h.find(1));
  REQUIRE(h.find(2));
  REQUIRE(h.find(3));
  h.UNION(1,2);
  REQUIRE_EQ(h.find(1),h.find(2));
}
