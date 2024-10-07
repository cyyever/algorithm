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
    cyy::algorithm::trie<char> string_trie(
        std::vector{std::string("abc"), std::string("abd")});
    static_assert(std::is_same_v<decltype(string_trie)::element_id_type, char>);
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
}
