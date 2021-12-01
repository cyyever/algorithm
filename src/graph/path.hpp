
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
  using path_type=std::vector<size_t>;
  inline path_type
  convert_parent_list_to_path(const std::vector<size_t> &parent, size_t source,
                              size_t sink) {
    std::vector<size_t> path;
    path.reserve(parent.size());
    auto vertex = sink;
    while (vertex != source) {
      if (vertex == SIZE_MAX) {
        return {};
      }
      path.push_back(vertex);
      vertex = parent[vertex];
    }
    path.push_back(source);
    std::ranges::reverse(path);
    return path;
  }

  template <typename graphType,bool minimum=true>
    auto get_extreme_weight(const graphType &g, const path_type& path) {
      typename graphType::weight_type extreme_weight{};
      for (size_t i = 0; i + 1 < path.size(); i++) {
        indexed_edge e{path[i], path[i + 1]};
        if constexpr(minimum) {
          extreme_weight= std::min(extreme_weight,g.get_edge(e).weight);
        } else {
          extreme_weight= std::max(extreme_weight,g.get_edge(e).weight);

        }
      }
      return extreme_weight;
    }


  template <typename graphType>
  auto shortest_path_by_edge_number(const graphType &g, size_t s) {
#ifndef NDEBUG
    assert(g.has_vertex_index(s));
#endif
    if (!g.has_vertex_index(s)) {
      return std::vector<size_t>();
    }
    std::vector<size_t> parent(g.get_next_vertex_index(), SIZE_MAX);
    parent[s] = s;
    g.breadth_first_search(s, [&parent](auto u, auto v, auto) {
      parent[v] = u;
      return false;
    });

    return parent;
  }

  template <typename graphType>
  auto shortest_path_Dijkstra(const graphType &g, size_t s) {
#ifndef NDEBUG
    assert(g.has_vertex_index(s));
    for (auto const &[_, weight] : g.foreach_edge_with_weight()) {
      assert(weight >= 0);
    }
#endif
    if (!g.has_vertex_index(s)) {
      return std::vector<size_t>();
    }
    using weight_type = typename graphType::weight_type;
    std::vector<std::optional<weight_type>> distance(g.get_next_vertex_index());
    distance[s] = 0;
    std::vector<size_t> parent(g.get_next_vertex_index(), SIZE_MAX);
    parent[s] = s;
    heap<size_t, weight_type> h;
    h.insert(s, 0);

    while (!h.empty()) {
      auto u = h.top_data();
      h.pop();
      for (auto [v, weight] : g.get_adjacent_list(u)) {
        assert(weight >= 0);
        if (distance[v].has_value() &&
            distance[v].value() <= distance[u].value() + weight) {
          continue;
        }
        parent[v] = u;
        distance[v] = distance[u].value() + weight;
        if (h.contains(v)) {
          h.change_key(v, distance[v].value());
        } else {
          h.insert(v, distance[v].value());
        }
      }
    }
    return parent;
  }
  template <typename graphType>
  auto shortest_path_Bellman_Ford(const graphType &g, size_t s) {
#ifndef NDEBUG
    assert(g.has_vertex_index(s));
#endif
    if (!g.has_vertex_index(s)) {
      return std::vector<size_t>();
    }
    using weight_type = typename graphType::weight_type;
    std::vector<std::optional<weight_type>> distance(g.get_next_vertex_index());
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
          if (distance[u].has_value() &&
              (!distance[v].has_value() ||
               distance[u].value() + weight < distance[v].value())) {
            distance[v] = distance[u].value() + weight;
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

  template <typename graphType>
  std::vector<size_t> get_path(const graphType &g, size_t source,
                               size_t target) {
    std::vector<size_t> parent(g.get_next_vertex_index(), SIZE_MAX);
    g.recursive_depth_first_search(source, [&parent, &target](auto u, auto v) {
      parent[v] = u;
      return v == target;
    });
    return convert_parent_list_to_path(parent, source, target);
  }
} // namespace cyy::algorithm
