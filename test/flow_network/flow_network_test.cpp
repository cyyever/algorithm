/*!
 * \file flow_network_test.cpp
 *
 * \brief
 */

#ifdef CYY_MATH
#include <cyy/math/all.hpp>
#endif

#include <doctest/doctest.h>

#include "flow_network/flow_network.hpp"

using namespace cyy::algorithm;
TEST_CASE("flow network") {
#ifdef CYY_MATH
  using weight_type = cyy::math::integer;
#else
  using weight_type = int;
#endif
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

TEST_CASE("complex flow network") {
  using weight_type = double;
  weight_type const M = 2;
  weight_type const r = (sqrt(5) - 1) / 2;
  cyy::algorithm::flow_network<std::string, weight_type>::capacity_fun_type
      capacities;
  capacities.emplace_back("s", "a", M);
  capacities.emplace_back("s", "b", M);
  capacities.emplace_back("s", "d", M);
  capacities.emplace_back("b", "a", 1);
  capacities.emplace_back("b", "c", 1);
  capacities.emplace_back("d", "c", r);
  capacities.emplace_back("a", "t", M);
  capacities.emplace_back("c", "t", M);
  capacities.emplace_back("d", "t", M);

  SUBCASE("max flow ford fulkerson") {
    cyy::algorithm::flow_network<std::string, weight_type> network(capacities,
                                                                   "s", "t");
    network.max_flow_by_ford_fulkerson();
    REQUIRE_EQ(network.get_flow_value(), 2 * M + 1);
  }
  SUBCASE("max flow edmonds karp") {
    cyy::algorithm::flow_network<std::string, weight_type> network(capacities,
                                                                   "s", "t");
    network.max_flow_by_edmonds_karp();
    REQUIRE_EQ(network.get_flow_value(), 2 * M + 1);
  }
}
