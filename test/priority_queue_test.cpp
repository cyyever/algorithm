/*!
 * \file container_test.cpp
 *
 * \brief 测试container相关函数
 * \author cyy
 */

#include <doctest/doctest.h>

#include "priority_queue.hpp"

using namespace cyy::algorithm;

TEST_CASE("priority_queue") {
  cyy::algorithm::priority_queue<int, int> h;
  REQUIRE(h.empty());

  SUBCASE("insert") {
    REQUIRE(h.insert(5, 5));
    REQUIRE(h.insert(1, 1));
    REQUIRE(!h.insert(1, 1));
  }
  SUBCASE("top_data") {
    h.insert(5, 5);
    h.insert(1, 1);
    REQUIRE_EQ(h.top_data(), 1);
  }
  SUBCASE("remove") {
    h.insert(5, 5);
    h.insert(1, 1);
    h.remove(5);
    REQUIRE_EQ(h.top_data(), 1);
  }
  SUBCASE("pop") {
    h.insert(5, 5);
    h.insert(1, 1);
    h.pop();
    REQUIRE_EQ(h.top_data(), 5);
    h.pop();
    REQUIRE(h.empty());
    h.pop();
  }
  SUBCASE("change key") {
    h.insert(5, 5);
    h.insert(1, 1);
    REQUIRE_EQ(h.top_data(), 1);
    h.change_key(1, 100);
    REQUIRE_EQ(h.top_data(), 5);
  }
}

TEST_CASE("max_priority_queue") {
  cyy::algorithm::max_priority_queue<int, int> h;
  REQUIRE(h.empty());

  SUBCASE("top_data") {
    h.insert(5, 5);
    h.insert(1, 1);
    REQUIRE_EQ(h.top_data(), 5);
  }
}
