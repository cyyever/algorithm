/*!
 * \file cfg_test.cpp
 *
 * \brief 测试cfg
 */

#include <doctest/doctest.h>

#include "pool.hpp"

using namespace cyy::algorithm;
TEST_CASE("pool") {
  SUBCASE("char") {
    static_assert(
        !std::is_same_v<cyy::algorithm::object_pool<char>::element_id_type,
                        char>);
    static_assert(
        std::is_same_v<
            cyy::algorithm::object_pool<char, false>::element_id_type, char>);
  }
  SUBCASE("std::string") {
    static_assert(!std::is_same_v<
                  cyy::algorithm::object_pool<std::string>::element_id_type,
                  std::string>);
  }
}
