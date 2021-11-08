
/*!
 * \file graph_algorithm.hpp
 *
 * \brief implements graph_algorithm
 */

#pragma once

#include <cassert>
#include <functional>
#include <vector>

#include "graph_base.hpp"
#include "heap.hpp"

namespace cyy::algorithm {

  template <typename graphType>
  auto shortest_path_Dijkstra(const graphType &g, size_t s) {
#ifndef NDEBUG

    for (auto const &[_, weight] : g.foreach_edge_with_weight()) {
      assert(weight >= 0);
    }
#endif

    using weight_type = graphType::weight_type;
    std::vector<weight_type> distance(g.get_next_vertex_index(),
                                      std::numeric_limits<weight_type>::max());
    distance[s] = 0;
    std::vector<size_t> parent(g.get_next_vertex_index(), SIZE_MAX);
    parent[s] = s;
    heap<size_t, weight_type> h;
    h.insert(s, 0);

    while (!h.empty()) {
      auto u = h.top();
      h.pop();
      for (auto [v, weight] : g.get_adjacent_list(u)) {
        assert(weight >= 0);
        if (distance[v] <= distance[u] + weight) {
          continue;
        }
        parent[v] = u;
        distance[v] = distance[u] + weight;
        if (h.contains(v)) {
          h.change_key(v, distance[v]);
        } else {
          h.insert(v, distance[v]);
        }
      }
    }
    return parent;
  }
  template <typename graphType>
  auto shortest_path_Bellman_Ford(const graphType &g, size_t s) {
    using weight_type = graphType::weight_type;
    std::vector<weight_type> distance(g.get_next_vertex_index(),
                                      std::numeric_limits<weight_type>::max());
    distance[s] = 0;
    std::vector<size_t> parent(g.get_next_vertex_index(), SIZE_MAX);
    parent[s] = s;

    auto vertex_number = g.get_vertex_number();
    if (vertex_number == 0) {
      return parent;
    }
    bool flag = false;
    for (size_t i = 0; i + 1 < vertex_number; i++) {
      flag = false;
      for (auto const u : g.get_vertex_indices()) {
        for (auto [v, weight] : g.get_adjacent_list(u)) {
          if (distance[u] + weight < distance[v]) {
            distance[v] = distance[u] + weight;
            parent[v] = u;
            flag = true;
          }
        }
      }
      if (!flag) {
        break;
      }
    }
    return parent;
  }

} // namespace cyy::algorithm
