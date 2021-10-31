/*!
 * \file cfg_test.cpp
 *
 * \brief 测试cfg
 */

#include <doctest/doctest.h>

#include "list_algorithm.hpp"

using namespace cyy::algorithm;
TEST_CASE("inversion") {
  REQUIRE(get_inversion_number(std::vector<int>{1, 2, 3}) == 0);
  REQUIRE(get_inversion_number(std::vector<int>{3, 2, 1}) == 3);
}
