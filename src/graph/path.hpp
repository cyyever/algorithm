
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

  template <typename vertex_type, bool directed>
  auto shortest_path_dijkstra(const graph_base<vertex_type, directed> &g,
                              size_t s) {

    std::vector<double> distance(g.get_next_vertex_index(),
                                 std::numeric_limits<double>::max());
    distance[s] = 0;
    std::vector<size_t> parent(g.get_next_vertex_index(), SIZE_MAX);
    parent[s] = s;
    heap<size_t, double> h;
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

} // namespace cyy::algorithm
