/*!
 * \file container_test.cpp
 *
 * \brief 测试container相关函数
 * \author cyy
 */

#include <string>

#include <doctest/doctest.h>

#include "graph/algorithm.hpp"
#include "graph/graph.hpp"

TEST_CASE("graph") {
  cyy::algorithm::directed_graph<std::string> g;
  SUBCASE("process edge") {
    g.add_edge({"1", "2"});
    g.remove_edge({"1", "2"});
  }
  SUBCASE("transpose") { g.get_transpose(); }
  SUBCASE("breadth first search") {
    g.add_edge({"1", "2"});
    auto tree = breadth_first_search(g, 0zu);
    REQUIRE(tree.get_vertex_number() == 2);
  }
  SUBCASE("depth first search") {
    g.add_edge({"1", "2"});
    auto tree = depth_first_search(g, 0zu);
    REQUIRE(tree.get_vertex_number() == 2);
  }
  SUBCASE("topological ordering") {
    g.add_edge({"1", "2"});
    cyy::algorithm::DAG<std::string> dag(g);
    auto order = dag.get_topological_ordering();
    REQUIRE(order);
  }
}