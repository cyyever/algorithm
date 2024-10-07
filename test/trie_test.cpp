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
    cyy::algorithm::trie<char> string_trie(std::vector{std::string("abc"),std::string("abd")});
    auto view= string_trie.get_level_view(0);
    REQUIRE(std::ranges::size(view)==1);
    for (auto a:view) {
    REQUIRE(a==string_trie.get_data_id('a'));

    }

  }

}
