/*!
 * \file container_test.cpp
 *
 * \brief 测试container相关函数
 * \author cyy
 */

#include <string>

#include <doctest/doctest.h>

#include "flow_network/flow_network.hpp"
#include "graph/algorithm.hpp"
#include "graph/graph.hpp"

using namespace cyy::algorithm;
TEST_CASE("graph") {
  cyy::algorithm::directed_graph<std::string> g;
  g.add_edge({"s", "u"});
  g.add_edge({"s", "v"});
  g.add_edge({"u", "v"});
  g.add_edge({"u", "t"});
  g.add_edge({"v", "t"});
  std::unordered_map<std::pair<std::string, std::string>, float> capacities;
  capacities[{"s", "u"}] = 20;
  capacities[{"s", "v"}] = 10;
  capacities[{"u", "v"}] = 30;
  capacities[{"u", "t"}] = 10;
  capacities[{"v", "t"}] = 20;

  cyy::algorithm::flow_network<std::string> network(g, "s", "t", capacities);
}
