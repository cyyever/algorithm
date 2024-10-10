/*!
 * \file union_test.cpp
 *
 */

#include <doctest/doctest.h>

#include "union_find.hpp"

TEST_CASE("union_find") {
  using uf = cyy::algorithm::union_find<int>;
  auto node1 = uf::make_set(1);
  auto node2 = uf::make_set(2);
  auto node3 = uf::make_set(3);
  REQUIRE_EQ(uf::find(node1), node1.get());
  REQUIRE(!uf::find(nullptr));
  uf::UNION(node1, node2);
  uf::UNION(node1, node3);
  REQUIRE_EQ(uf::find(node1), uf::find(node2));
  REQUIRE_EQ(uf::find(node2), uf::find(node3));
}
