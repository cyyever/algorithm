/*!
 * \file cfg_test.cpp
 *
 * \brief 测试cfg
 */

#include <doctest/doctest.h>

#include "kmp.hpp"

using namespace cyy::algorithm;
TEST_CASE("KMP") {
  SUBCASE("match") {

    std::string str = "abababaab";
    std::string word = "ababaa";

    KMP<char> kmp(word);
    auto idx_opt = kmp.search(str);
    REQUIRE(idx_opt == 2);
    str = "2111222";
    word = "111222";

    KMP<char> kmp2(word);
    idx_opt = kmp2.search(str);
    REQUIRE(idx_opt == 1);
  }
  SUBCASE("match empty") {

    std::string str = "abababaab";
    std::string word;

    KMP<char> kmp(word);
    auto idx_opt = kmp.search(str);
    REQUIRE(idx_opt == 0);
  }

  SUBCASE("dismatch") {
    std::string str = "abababbaa";
    std::string word = "ababaa";
    KMP<char> kmp(word);
    auto idx_opt = kmp.search(str);
    REQUIRE(!idx_opt.has_value());
    word = "c";
    kmp = KMP<char>(word);
    idx_opt = kmp.search(str);
    REQUIRE(!idx_opt.has_value());
  }
}
