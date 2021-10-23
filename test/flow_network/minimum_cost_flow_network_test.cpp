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
  cyy::algorithm::minimum_cost_flow_network<
      std::string>::capacity_and_cost_fun_type capacity_and_cost{

      {{"s", "u"}, {0, 5, 1}},  {{"s", "t"}, {0, 6, 1}},
      {{"t", "x"}, {0, 9, -1}}, {{"t", "v"}, {0, 5, -1}},
      {{"u", "x"}, {0, 2, 1}},  {{"u", "v"}, {0, 9, -1}},
      {{"x", "v"}, {3, 7, -1}}, {{"v", "s"}, {0, 7, 1}},
      {{"w", "s"}, {0, 5, -1}}, {{"w", "v"}, {-2, 2, -1}}};
  std::unordered_map<std::string, double> demand{{"s", 0}, {"t", 0}, {"u", 0},
                                                 {"v", 0}, {"w", 0}, {"x", 0}};

  cyy::algorithm::minimum_cost_flow_network<std::string> network(
      capacity_and_cost, demand);
  auto flow_opt = network.min_cost_flow_by_network_simplex();
  REQUIRE(flow_opt.has_value());
  REQUIRE(network.get_cost(*flow_opt) == -2);
}
