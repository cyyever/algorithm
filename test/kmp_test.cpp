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
    REQUIRE(idx_opt ==2);
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
