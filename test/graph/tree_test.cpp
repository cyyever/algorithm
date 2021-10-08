/*!
 * \file container_test.cpp
 *
 * \brief 测试container相关函数
 * \author cyy
 */

#include <string>

#include <doctest/doctest.h>

#include "graph/tree.hpp"

TEST_CASE("tree") {
  std::vector<cyy::algorithm::edge<std::string>> edges;
  edges.emplace_back("1", "2");
  cyy::algorithm::tree<std::string> T(edges);
}
