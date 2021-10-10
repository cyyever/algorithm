/*!
 * \file container_test.cpp
 *
 * \brief 测试container相关函数
 * \author cyy
 */

#include <doctest/doctest.h>

#include "coding/algorithm.hpp"

using namespace cyy::algorithm;
TEST_CASE("huffman") {
  auto code = huffman_code({
      {'a', 0.32},
      {'b', 0.25},
      {'c', 0.2},
      {'d', 0.18},
      {'e', 0.05},
  });
  REQUIRE_EQ(code['a'], "11");
  REQUIRE_EQ(code['b'], "10");
  REQUIRE_EQ(code['c'], "00");
  REQUIRE_EQ(code['e'], "010");
  REQUIRE_EQ(code['d'], "011");
}
