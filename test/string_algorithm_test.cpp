/*!
 * \file cfg_test.cpp
 *
 * \brief 测试cfg
 */

#include <doctest/doctest.h>

#include "string_algorithm.hpp"

using namespace cyy::algorithm;

TEST_CASE("Aho_Corasick") {
  SUBCASE("dismatch") {
    const auto *str = "abababaab";
    std::vector<std::string_view> const words = {"aaa", "abaaa", "ababaaa"};

    CHECK(Aho_Corasick<char>(words, str).empty());
  }

  SUBCASE("match") {
    const auto *str = "abaa";
    std::vector<std::string_view> words = {"aa", "abaaa"};

    auto p = Aho_Corasick<char>(words, str);
    REQUIRE(!p.empty());
    CHECK(p == words[0]);
  }
}
