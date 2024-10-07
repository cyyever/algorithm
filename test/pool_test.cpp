/*!
 * \file cfg_test.cpp
 *
 * \brief 测试cfg
 */

#include <doctest/doctest.h>

#include "pool.hpp"

using namespace cyy::algorithm;
TEST_CASE("tire construction") {
  SUBCASE("string") {
    cyy::algorithm::pool<char> p;
    static_assert(std::is_same_v<decltype(p)::element_id_type, char>);
  }
}
