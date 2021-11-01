/*!
 * \file container_test.cpp
 *
 * \brief 测试container相关函数
 * \author cyy
 */

#include <string>

#include <doctest/doctest.h>

#include "graph/dag.hpp"
#include "graph/graph.hpp"
#include "graph/path.hpp"

using namespace cyy::algorithm;
TEST_CASE("graph") {
  cyy::algorithm::graph<std::string> g;
  cyy::algorithm::directed_graph<std::string> h;
  SUBCASE("process edge") {
    g.add_edge({"1", "2"});
    g.remove_edge({"1", "2"});
  }
  SUBCASE("rearrange") { g.rearrange_vertices(); }
  SUBCASE("transpose") { h.get_transpose(); }
  SUBCASE("depth first recursive search") {
    h.add_edge({"1", "2"});
    h.recursive_depth_first_search(0, [](auto u, auto v) {
      REQUIRE_EQ(u, 0);
      REQUIRE_EQ(v, 1);
      return false;
    });
  }
  SUBCASE("topological ordering") {
    h.add_edge({"1", "2"});
    cyy::algorithm::DAG<std::string> dag(h);
    auto order = dag.get_topological_ordering();
    REQUIRE(order);
  }
  SUBCASE("Dijkstra shortest path") {
    g.add_edge({"1", "2"});
    auto parent = shortest_path_Dijkstra(g, 0);
    REQUIRE(parent[1] == 0);
  }
  SUBCASE("Bellman-Ford search") {
    g.add_edge({"s", "u", 2});
    g.add_edge({"u", "w", 3});
    g.add_edge({"w", "v", -6});
    g.add_edge({"s", "v", 1});
    auto parent = shortest_path_Bellman_Ford(g, 0);
    REQUIRE(parent[3] == 2);
  }
}
