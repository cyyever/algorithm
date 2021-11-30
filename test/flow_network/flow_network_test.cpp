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

#include "flow_network/flow_network.hpp"
#include "graph/graph.hpp"

using namespace cyy::algorithm;
TEST_CASE("flow network") {
#ifdef CYY_MATH
  using weight_type = cyy::math::integer;
#else
  using weight_type = int;
#endif
  cyy::algorithm::directed_graph<std::string, weight_type> g;
  g.add_edge({"s", "u"});
  g.add_edge({"s", "v"});
  g.add_edge({"u", "v"});
  g.add_edge({"u", "t"});
  g.add_edge({"v", "t"});
  cyy::algorithm::flow_network<std::string, weight_type>::capacity_fun_type
      capacities;
  capacities.emplace_back("s", "u", 20);
  capacities.emplace_back("s", "v", 10);
  capacities.emplace_back("u", "v", 30);
  capacities.emplace_back("u", "t", 10);
  capacities.emplace_back("v", "t", 20);

  SUBCASE("max flow ford fulkerson") {
    cyy::algorithm::flow_network<std::string, weight_type> network(capacities,
                                                                   "s", "t");
    network.max_flow_by_ford_fulkerson();
    REQUIRE_EQ(network.get_flow_value(), 30);
  }
  SUBCASE("max flow edmonds karp") {
    cyy::algorithm::flow_network<std::string, weight_type> network(capacities,
                                                                   "s", "t");
    network.max_flow_by_edmonds_karp();
    REQUIRE_EQ(network.get_flow_value(), 30);
  }
  SUBCASE("min cut") {
    cyy::algorithm::flow_network<std::string, weight_type> network(capacities,
                                                                   "s", "t");
    auto [s_set, t_set] = network.get_minimum_capacity_s_t_cut();
  }
}
