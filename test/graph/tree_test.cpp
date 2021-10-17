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
  cyy::algorithm::tree<std::string> T;
  T.add_edge({"1","2"});


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

}
