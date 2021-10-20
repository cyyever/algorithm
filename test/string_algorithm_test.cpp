/*!
 * \file cfg_test.cpp
 *
 * \brief 测试cfg
 */

#include <doctest/doctest.h>

#include "string_algorithm.hpp"

using namespace cyy::algorithm;
TEST_CASE("KMP") {
  SUBCASE("match") {

    std::string str = "abababaab";
    std::string word = "ababaa";

    auto idx_opt = KMP(word, str);
    REQUIRE(idx_opt.has_value());
    auto idx = *idx_opt;
    REQUIRE(str.substr(idx, word.size()) == word);
  }

  SUBCASE("dismatch") {
    std::string str = "abababbaa";
    std::string word = "ababaa";
    auto idx_opt = KMP(word, str);
    REQUIRE(!idx_opt.has_value());
  }
}

TEST_CASE("Aho_Corasick") {
  SUBCASE("dismatch") {
    auto str = "abababaab";
    std::vector<std::string_view> words = {"aaa", "abaaa", "ababaaa"};

    CHECK(Aho_Corasick<char>(words, str).empty());
  }

  SUBCASE("match") {
    auto str = "abaa";
    std::vector<std::string_view> words = {"aa", "abaaa"};

    auto p = Aho_Corasick<char>(words, str);
    REQUIRE(!p.empty());
    CHECK(p == words[0]);
  }
}
