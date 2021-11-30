/*!
 * \file container_test.cpp
 *
 * \brief 测试container相关函数
 * \author cyy
 */

#include <doctest/doctest.h>

#include "heap.hpp"

using namespace cyy::algorithm;

TEST_CASE("simple_heap") {
  cyy::algorithm::simple_heap<int> h;
  REQUIRE(h.empty());

  SUBCASE("insert") { h.insert(5); }
  SUBCASE("top") {
    h.insert(5);
    h.insert(1);
    REQUIRE_EQ(h.top(), 1);
  }
  SUBCASE("pop") {
    h.insert(5);
    h.insert(1);
    h.pop();
    REQUIRE_EQ(h.top(), 5);
    h.pop();
    REQUIRE(h.empty());
  }
  SUBCASE("change data") {
    h.insert(5);
    REQUIRE_EQ(h.top(), 5);
    h.change_data(0, 1);
    REQUIRE_EQ(h.top(), 1);
  }
}

TEST_CASE("heap") {
  cyy::algorithm::heap<int, int> h;
  REQUIRE(h.empty());

  SUBCASE("insert") {
    h.insert(5, 5);
    h.insert(1, 1);
  }
  SUBCASE("top_data") {
    h.insert(5, 5);
    h.insert(1, 1);
    REQUIRE_EQ(h.top_data(), 1);
  }
  SUBCASE("pop") {
    h.insert(5, 5);
    h.insert(1, 1);
    h.pop();
    REQUIRE_EQ(h.top_data(), 5);
    h.pop();
    REQUIRE(h.empty());
  }
  SUBCASE("change key") {
    h.insert(5, 5);
    h.insert(1, 1);
    REQUIRE_EQ(h.top_data(), 1);
    h.change_key(1, 100);
    REQUIRE_EQ(h.top_data(), 5);
  }
  SUBCASE("change data") {
    h.insert(5, 5);
    REQUIRE_EQ(h.top_data(), 5);
    h.change_data(5, 1);
    REQUIRE_EQ(h.top_data(), 1);
  }
}

TEST_CASE("max_heap") {
  cyy::algorithm::max_heap<int, int> h;
  REQUIRE(h.empty());

  SUBCASE("top_data") {
    h.insert(5);
    h.insert(1);
    REQUIRE_EQ(h.top_data(), 5);
  }
}
