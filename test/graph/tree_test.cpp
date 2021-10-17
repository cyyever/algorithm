/*!
 * \file tree_test.cpp
 *
 * \brief
 */

#include <string>

#include <doctest/doctest.h>

#include "graph/tree.hpp"
#include "graph/tree_algorithm.hpp"

TEST_CASE("tree") {
  /*
    SUBCASE("Prim MST") {
      cyy::algorithm::graph<std::string> g;
      g.add_edge({"1", "2"});
      auto mst = MST_prime(g);
    }
    SUBCASE("Kruskal MST") {
      cyy::algorithm::graph<std::string> g;
      g.add_edge({"1", "2"});
      auto mst = MST_kruskal(g);
    }
    */
  SUBCASE("Prufer code") {
    cyy::algorithm::tree<std::string> T;
    T.add_vertex("1");
    T.add_vertex("2");
    T.add_vertex("3");
    T.add_vertex("4");
    T.add_vertex("5");
    T.add_vertex("6");
    /* T.add_edge({"4", "1"}); */
    /* auto code=cyy::algorithm::get_prufer_code(T); */
    /* REQUIRE(code.empty()); */
    T.add_edge({"4", "1"});
    T.add_edge({"4", "2"});
    T.add_edge({"4", "3"});
    T.add_edge({"5", "4"});
    T.add_edge({"6", "5"});
    REQUIRE(T.is_tree());
    auto code = cyy::algorithm::get_prufer_code(T);
    REQUIRE_EQ(code, std::vector<size_t>{3, 3, 3, 4});
  }
}
