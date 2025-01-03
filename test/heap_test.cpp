/*!
 * \file container_test.cpp
 *
 * \brief 测试container相关函数
 * \author cyy
 */

#include <doctest/doctest.h>

#include "heap.hpp"

using namespace cyy::algorithm;

TEST_CASE("heap") {
  cyy::algorithm::heap<int> h;
  REQUIRE(h.empty());

  SUBCASE("insert") { h.insert(5); }
  SUBCASE("top") {
    h.insert(5);
    h.insert(1);
    REQUIRE_EQ(h.top(), 1);
  }
  SUBCASE("remove") {
    h.insert(5);
    h.insert(1);
    h.remove_item(1);
    REQUIRE_EQ(h.top(), 1);
  }
  SUBCASE("pop") {
    h.insert(5);
    h.insert(1);
    h.pop();
    REQUIRE_EQ(h.top(), 5);
    h.pop();
    REQUIRE(h.empty());
    h.pop();
    REQUIRE(h.empty());
  }
}

TEST_CASE("window_heap") {
  cyy::algorithm::window_heap<int> h;
  REQUIRE(h.empty());

  SUBCASE("insert") {
    h.push(5);
    h.push(1);
    REQUIRE_EQ(h.top(), 1);
  }
  SUBCASE("pop") {
    h.push(1);
    h.push(5);
    REQUIRE_EQ(h.top(), 1);
    h.pop();
    REQUIRE_EQ(h.top(), 5);
  }
}
