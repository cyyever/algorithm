
/*!
 * \file graph_algorithm.hpp
 *
 * \brief implements graph_algorithm
 */

#pragma once

#include <cassert>
#include <functional>
#include <iostream>
#include <vector>

#include "graph.hpp"
#include "heap.hpp"
#include "tree.hpp"
#include "union_find.hpp"

namespace cyy::algorithm {

  template <typename vertex_type, typename weight_type = double>
  auto MST_prime(const graph<vertex_type, weight_type> &g) {

    std::vector<std::optional<weight_type>> weights(g.get_next_vertex_index());
    std::vector<size_t> edge(g.get_next_vertex_index(), SIZE_MAX);
    graph<vertex_type> MST;
    heap<size_t, weight_type> h;
    auto s = *g.get_vertex_indices().begin();
    weights[s] = 0;
    h.insert(s, 0);
    while (!h.empty()) {
      auto u = h.top_data();
      h.pop();
      for (auto [v, weight] : g.get_adjacent_list(u)) {
        if (weights[v].has_value() && weight >= weights[v]) {
          continue;
        }
        edge[v] = u;
        weights[v] = weight;
        if (h.contains(v)) {
          h.change_key(v, weight);
        } else {
          h.insert(v, weight);
        }
      }
    }
    for (size_t v = 0; v < edge.size(); v++) {
      auto u = edge[v];
      if (u == SIZE_MAX) {
        continue;
      }
      MST.add_edge({g.get_vertex(u), g.get_vertex(v), weights[v].value()});
    }
    return tree(MST, false);
  }

  template <typename vertex_type>
  auto MST_kruskal(const graph<vertex_type> &g) {
    std::set<indexed_edge> edges;
    for (auto e : g.foreach_edge()) {
      edges.emplace(std::move(e));
    }
    union_find<size_t> connected_components(g.get_vertex_indices());
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

  template <typename vertex_type, typename weight_type = double>
  auto get_prufer_code(const tree<vertex_type, weight_type> &T) {
    if (!T.has_continuous_vertices()) {
      throw std::logic_error("need continuous vertices");
    }
    auto vertex_number = T.get_vertex_number();
    if (vertex_number < 2) {
      throw std::logic_error("need at least two vertices");
    }
    std::vector<size_t> code;
    if (vertex_number == 2) {
      return code;
    }
    code.reserve(vertex_number - 2);
    std::map<size_t, std::list<std::pair<size_t, weight_type>>> adjacent_list(
        T.get_adjacent_list().begin(), T.get_adjacent_list().end());
    while (vertex_number > 2) {
      auto it =
          std::find_if(adjacent_list.begin(), adjacent_list.end(),
                       [](auto const &p) { return p.second.size() == 1; });
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
