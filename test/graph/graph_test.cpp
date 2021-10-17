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
  SUBCASE("Dijkstra search") {
    g.add_edge({"1", "2"});
    auto edges = shortest_path_dijkstra(g, 0);
    REQUIRE(edges[1] == 0);
  }
}
