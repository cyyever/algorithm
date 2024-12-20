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
  SUBCASE("Prim MST") {
    cyy::algorithm::graph<std::string> g;
    g.add_edge({"1", "2"});
    auto mst = MST_prime(g);
    REQUIRE_EQ(mst.get_edge_number(), 1);
  }
  SUBCASE("Kruskal MST") {
    cyy::algorithm::graph<std::string> g;
    g.add_edge({"1", "2"});
    auto mst = MST_kruskal(g);
    REQUIRE_EQ(mst.get_edge_number(), 1);
  }
  /* #ifdef __cpp_lib_format */
  /*   SUBCASE("print") { */
  /*     cyy::algorithm::tree<std::string> T; */
  /*     T.add_edge({"1", "2"}); */
  /*     std::cout << std::format("{}", T) << std::endl; */
  /*   } */
  /* #endif */
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
