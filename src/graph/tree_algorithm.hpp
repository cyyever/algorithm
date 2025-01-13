
/*!
 * \file graph_algorithm.hpp
 *
 * \brief implements graph_algorithm
 */

#pragma once

#include <cassert>
#include <map>
#include <vector>

#include "graph.hpp"
#include "heap.hpp"
#include "tree.hpp"
#include "union_find.hpp"

namespace cyy::algorithm {
  template <typename vertex_type, typename weight_type = double>
  auto MST_prime(const graph<vertex_type, weight_type> &g) {
    assert(g.has_continuous_vertices());

    std::vector<bool> visited(g.get_vertex_number(), false);
    min_heap<weighted_indexed_edge<weight_type>> h;
    h.insert({0, SIZE_MAX, 0});
    graph<vertex_type, weight_type> MST;
    while (!h.empty()) {
      const auto &e = h.top();
      auto u = e.first;
      auto v = e.second;
      if (visited[u]) {
        h.pop();
        continue;
      }
      visited[u] = true;
      if (v != SIZE_MAX) {
        MST.add_edge(g.get_edge(e));
      }
      h.pop();

      for (auto [w, weight] : g.get_adjacent_list(u)) {
        if (!visited[w]) {
          h.insert({w, u, weight});
        }
      }
    }
    return tree(MST, false);
  }

  template <typename vertex_type, typename weight_type>
  auto MST_kruskal(const graph<vertex_type, weight_type> &g) {
    auto edges = std::ranges::to<std::vector>(g.foreach_edge_with_weight());
    std::ranges::sort(edges);
    using uf = union_find<size_t>;
    std::vector<uf::node_ptr> uf_nodes;
    uf_nodes.resize(g.get_vertex_number());
    graph<vertex_type, weight_type> MST;

    for (auto const &edge : edges) {
      auto const &u = edge.first;
      auto const &v = edge.second;
      assert(u < uf_nodes.size());
      assert(v < uf_nodes.size());
      if (!uf_nodes[u]) {
        uf_nodes[u] = uf::make_set(u);
      }
      if (!uf_nodes[v]) {
        uf_nodes[v] = uf::make_set(v);
      }
      if (uf::find(uf_nodes[u]) != uf::find(uf_nodes[v])) {
        MST.add_edge(g.get_edge(edge));
        uf::UNION(uf_nodes[u], uf_nodes[v]);
      }
    }
    return tree(MST, false);
  }

  template <typename vertex_type, typename weight_type>
  auto get_prufer_code(const tree<vertex_type, weight_type> &T) {
    if (!T.has_continuous_vertices()) {
      throw std::logic_error("need continuous vertices");
    }
    auto vertex_number = T.get_vertex_number();
    std::vector<size_t> code;
    if (vertex_number == 2) {
      return code;
    }
    if (vertex_number < 2) {
      throw std::logic_error("need at least two vertices");
    }
    code.reserve(vertex_number - 2);
    std::map<size_t, std::list<std::pair<size_t, weight_type>>> adjacent_list(
        T.get_adjacent_list().begin(), T.get_adjacent_list().end());
    while (vertex_number > 2) {
      auto it = std::ranges::find_if(
          adjacent_list, [](auto const &p) { return p.second.size() == 1; });
      assert(it != adjacent_list.end());
      auto index = it->first;
      auto to_index = it->second.front().first;
      adjacent_list.erase(it);
      code.emplace_back(to_index);
      auto &to_vertices = adjacent_list[to_index];
      to_vertices.remove_if(
          [index](auto const &p) { return p.first == index; });
      vertex_number--;
    }
    return code;
  }
  tree<size_t> recover_tree(const std::vector<size_t> &prufer_code);
} // namespace cyy::algorithm
