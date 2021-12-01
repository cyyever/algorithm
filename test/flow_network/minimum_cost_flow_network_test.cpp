/*!
 * \file flow_network_test.cpp
 *
 * \brief
 */

#include <string>

#ifdef CYY_MATH
#include <cyy/math/integer.hpp>
#endif
#include <doctest/doctest.h>

#include "flow_network/minimum_cost_flow_network.hpp"
#include "graph/graph.hpp"

using namespace cyy::algorithm;
TEST_CASE("flow network") {
#ifdef CYY_MATH
  using weight_type = cyy::math::integer;
#else
  using weight_type = int;
#endif
  using min_cost_flow_by_network_type =
      cyy::algorithm::minimum_cost_flow_network<std::string, weight_type>;
  min_cost_flow_by_network_type ::capacity_and_cost_fun_type capacity_and_cost{

      {{"s", "u"}, {0, 5, 1}},  {{"s", "t"}, {0, 6, 1}},
      {{"t", "x"}, {0, 9, -1}}, {{"t", "v"}, {0, 5, -1}},
      {{"u", "x"}, {0, 2, 1}},  {{"u", "v"}, {0, 9, -1}},
      {{"x", "v"}, {3, 7, -1}}, {{"v", "s"}, {0, 7, 1}},
      {{"w", "s"}, {0, 5, -1}}, {{"w", "v"}, {-2, 2, -1}}};
  min_cost_flow_by_network_type::demand_fun_type demand{
      {"s", 0}, {"t", 0}, {"u", 0}, {"v", 0}, {"w", 0}, {"x", 0}};

  min_cost_flow_by_network_type network(capacity_and_cost, demand);
  auto flow_opt = network.min_cost_flow_by_network_simplex();
  REQUIRE(flow_opt.has_value());
  REQUIRE(network.get_cost(*flow_opt) == -2);
}
