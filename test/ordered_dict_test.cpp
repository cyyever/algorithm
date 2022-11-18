/*!
 * \file ordered_dict_test.cpp
 *
 * \author cyy
 */
#include <doctest/doctest.h>

#include "dict/order_dict.hpp"

TEST_CASE("order_dict") {
  cyy::algorithm::order_dict<int, std::string> container;

  CHECK(container.empty());

  SUBCASE("emplace") {
    container.emplace(1, "");
    CHECK(!container.empty());
    CHECK_EQ(container.size(), 1);
    container.emplace(1, "hello");
    CHECK_EQ(container.size(), 1);
    CHECK_EQ(container.rbegin()->second, "hello");
    container.clear();
    CHECK(container.empty());
  }

  SUBCASE("find") {
    CHECK_EQ(container.find(1), container.end());
    CHECK_EQ(container.begin(), container.end());
    CHECK_EQ(container.rbegin(), container.rend());

    container.emplace(1, "a");
    container.emplace(2, "b");
    auto it = container.begin();
    CHECK_EQ(it->second, "a");
    it++;
    CHECK_EQ(it->second, "b");

    container.find(1);
    it = container.begin();
    CHECK_EQ(it->second, "b");
    it++;
    CHECK_EQ(it->second, "a");
    container.clear();
  }

  SUBCASE("pop_oldest") {
    container.emplace(1, "a");
    container.emplace(2, "b");
    CHECK(!container.empty());
    CHECK_EQ(container.size(), 2);
    auto [k, v] = container.pop_oldest();
    CHECK_EQ(k, 1);
    CHECK_EQ(v, "a");
    std::tie(k, v) = container.pop_oldest();
    CHECK_EQ(k, 2);
    CHECK_EQ(v, "b");
    CHECK(container.empty());
    CHECK_THROWS(container.pop_oldest());
  }
}
