/*!
 * \file flow_network_test.cpp
 *
 * \brief
 */

#include <string>

#include <doctest/doctest.h>

#include "flow_network/minimum_cost_flow_network.hpp"
#include "graph/graph.hpp"

using namespace cyy::algorithm;
TEST_CASE("flow network") {
  cyy::algorithm::directed_graph<std::string> g;
  g.add_edge({"s", "u"});
  g.add_edge({"s", "v"});
  g.add_edge({"u", "v"});
  g.add_edge({"u", "t"});
  g.add_edge({"v", "t"});
}
