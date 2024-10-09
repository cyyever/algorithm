/*!
 * \file cfg_test.cpp
 *
 * \brief 测试cfg
 */

#include <doctest/doctest.h>

#include "trie.hpp"

using namespace cyy::algorithm;
TEST_CASE("tire construction") {
  SUBCASE("string") {
    cyy::algorithm::trie<char> string_trie(std::initializer_list<std::string>{
        std::string("abc"), std::string("abd")});
    auto view = string_trie.get_level_view(0);
    REQUIRE(std::ranges::size(view) == 1);
    for (auto a : view) {
      REQUIRE(a.element_id == string_trie.get_data_id('a'));
    }
    view = string_trie.get_level_view(1);
    REQUIRE(std::ranges::size(view) == 1);
    for (auto a : view) {
      REQUIRE(a.parent_id == string_trie.get_data_id('a'));
      REQUIRE(a.element_id == string_trie.get_data_id('b'));
    }
    view = string_trie.get_level_view(2);
    REQUIRE(std::ranges::size(view) == 2);
    for (auto a : view) {
      REQUIRE(a.is_end);
      auto res = (a.element_id == string_trie.get_data_id('c') ||
                  a.element_id == string_trie.get_data_id('d'));
      REQUIRE(res);
    }
  }
  SUBCASE("std::vector<int>") {
    cyy::algorithm::trie<int> string_trie(std::vector<std::vector<int>>{
        std::vector{1, 2, 3}, std::vector{1, 2, 4}});
    auto view = string_trie.get_level_view(0);
    REQUIRE(std::ranges::size(view) == 1);
    for (auto a : view) {
      REQUIRE(a.element_id == string_trie.get_data_id(1));
    }
    view = string_trie.get_level_view(1);
    REQUIRE(std::ranges::size(view) == 1);
    for (auto a : view) {
      REQUIRE(a.parent_id == string_trie.get_data_id(1));
      REQUIRE(a.element_id == string_trie.get_data_id(2));
    }
    view = string_trie.get_level_view(2);
    REQUIRE(std::ranges::size(view) == 2);
    for (auto a : view) {
      REQUIRE(a.is_end);
      auto res = (a.element_id == string_trie.get_data_id(3) ||
                  a.element_id == string_trie.get_data_id(4));
      REQUIRE(res);
    }
  }
}
