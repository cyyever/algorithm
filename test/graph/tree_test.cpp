/*!
 * \file tree_test.cpp
 *
 * \brief
 */

#include <doctest/doctest.h>

#include "graph/tree.hpp"
#include "graph/tree_algorithm.hpp"

TEST_CASE("tree") {
  SUBCASE("Prim MST") {
    cyy::algorithm::graph<std::string> g;
    g.add_edge({"1", "2"});
    auto mst = MST_prime(g);
    REQUIRE_EQ(mst.get_edge_number(), 1);
  }
  SUBCASE("MST") {
    cyy::algorithm::graph<std::string, int> g;
    g.add_edge({"A", "B", 7});
    g.add_edge({"B", "C", 8});
    g.add_edge({"A", "D", 5});
    g.add_edge({"D", "E", 15});
    g.add_edge({"B", "E", 7});
    g.add_edge({"B", "D", 9});
    g.add_edge({"C", "E", 5});
    g.add_edge({"D", "F", 6});
    g.add_edge({"E", "F", 8});
    g.add_edge({"E", "G", 9});
    g.add_edge({"F", "G", 11});
    auto mst = MST_kruskal(g);
    double total_weight = 0;
    for (const auto &e : mst.foreach_edge_with_weight()) {
      total_weight += e.weight;
    }
    REQUIRE_EQ(mst.get_edge_number(), g.get_vertex_number() - 1);
    REQUIRE_EQ(total_weight, 39);
    mst = MST_prime(g);
    total_weight = 0;
    for (const auto &e : mst.foreach_edge_with_weight()) {
      total_weight += e.weight;
    }
    REQUIRE_EQ(mst.get_edge_number(), g.get_vertex_number() - 1);
    REQUIRE_EQ(total_weight, 39);
  }
  SUBCASE("print") {
    cyy::algorithm::tree<std::string> T;
    T.add_edge({"1", "2"});
    std::println("{}", T);
  }
  SUBCASE("Prufer code") {
    cyy::algorithm::tree<std::string> T;
    T.add_vertex("1");
    T.add_vertex("2");
    T.add_vertex("3");
    T.add_vertex("4");
    T.add_vertex("5");
    T.add_vertex("6");
    T.add_edge({"4", "1"});
    T.add_edge({"4", "2"});
    T.add_edge({"4", "3"});
    T.add_edge({"5", "4"});
    T.add_edge({"6", "5"});
    REQUIRE(T.is_tree());
    auto code = cyy::algorithm::get_prufer_code(T);
    REQUIRE_EQ(code, std::vector<size_t>{3, 3, 3, 4});
    auto T2 = cyy::algorithm::recover_tree(code);

    REQUIRE_EQ(T2.get_edge_number(), 5);
  }
}
