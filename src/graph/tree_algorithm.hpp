
/*!
 * \file graph_algorithm.hpp
 *
 * \brief implements graph_algorithm
 */

#pragma once

#include <cassert>
#include <functional>
#include <vector>

#include "graph.hpp"
#include "heap.hpp"
#include "tree.hpp"
#include "union_find.hpp"

namespace cyy::algorithm {

  template <typename vertex_type> auto MST_prime(const graph<vertex_type> &g) {

    std::vector<float> weights(g.get_next_vertex_index(),
                               std::numeric_limits<float>::max());
    std::vector<size_t> edge(g.get_next_vertex_index(), SIZE_MAX);
    graph<vertex_type> MST;
    heap<size_t, float> h;
    auto s = *g.get_vertices().begin();
    h.insert(s, 0);
    while (!h.empty()) {
      auto u = h.top();
      h.pop();
      for (auto [v, weight] : g.get_adjacent_list(u)) {
        if (weight >= weights[v]) {
          continue;
        }
        edge[v] = u;
        weights[v] = weight;
        if (h.contains(v)) {
          h.change_key(v, weights[v]);
        } else {
          h.insert(v, weights[v]);
        }
      }
    }
    for (size_t v = 0; v < edge.size(); v++) {
      auto u = edge[v];
      if (u == SIZE_MAX) {
        continue;
      }
      MST.add_edge({g.get_vertex(u), g.get_vertex(v), weights[v]});
    }
    return tree(MST, false);
  }

  template <typename vertex_type>
  auto MST_kruskal(const graph<vertex_type> &g) {
    std::set<indexed_edge> edges;
    g.foreach_edge([&edges](auto edge) { edges.emplace(std::move(edge)); });
    union_find<size_t> connected_components(g.get_vertices());
    graph<vertex_type> MST;

    for (auto const &edge : edges) {
      auto [u, v] = edge;
      if (u != v) {
        auto u_component = connected_components.find(u);
        auto v_component = connected_components.find(v);
        MST.add_edge({g.get_edge(edge)});
        connected_components.UNION(u_component, v_component);
      }
    }
    return tree(MST, false);
  }

  template <typename vertex_type>
  auto get_prufer_code(const tree<vertex_type> &T) {
    if (!T.has_continuous_vertices()) {
      throw std::logic_error("need continuous vertices");
    }
  }

} // namespace cyy::algorithm
