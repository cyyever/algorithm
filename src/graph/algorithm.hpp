
/*!
 * \file graph_algorithm.hpp
 *
 * \brief implements graph_algorithm
 */

#pragma once

#include <cassert>
#include <functional>
#include <list>
#include <optional>
#include <ranges>
#include <unordered_map>
#include <vector>

#include "graph.hpp"
#include "tree.hpp"

namespace cyy::algorithm {

// breadth first search in g from s
template <typename vertex_type>
void breadth_first_search(const graph<vertex_type> &g, size_t s,
                          std::function<void(edge<vertex_type>)> edge_fun) {
  std::vector<bool> discovered(g.get_next_vertex_index(), false);
  discovered[s] = true;
  std::list<size_t> queue{s};

  while (!queue.empty()) {
    auto u = queue.front();
    queue.pop_front();
    for (auto const &neighbor : g.get_adjacent_list(u)) {
      auto const &[v, weight] = neighbor;
      if (!discovered[v]) {
        discovered[v] = true;
        queue.push_back(v);
        edge_fun({g.get_vertex(u), g.get_vertex(v), weight});
        continue;
      }
    }
  }
}

template <typename vertex_type>
auto get_breadth_first_search_tree(const graph<vertex_type> &g, size_t s) {

  tree<vertex_type> t;
  breadth_first_search<vertex_type>(g, s,
                                    [&t](auto edge) { t.add_edge(edge); });
  return t;
}

// depth first search in g from s
template <typename vertex_type>
void depth_first_search(const graph<vertex_type> &g, size_t s,
                        std::function<void(edge<vertex_type>)> edge_fun) {
  std::vector<bool> explored(g.get_next_vertex_index(), false);
  std::vector<std::pair<size_t, float>> stack{{s, 0}};
  std::vector<size_t> parent(g.get_next_vertex_index(), 0);

  while (!stack.empty()) {
    auto [u, weight] = stack.back();
    stack.pop_back();
    if (explored[u]) {
      continue;
    }
    explored[u] = true;
    if (u != s) {
      edge_fun({g.get_vertex(parent[u]), g.get_vertex(u), weight});
    }
    for (auto const &neighbor : g.get_adjacent_list(u)) {
      parent[neighbor.first] = u;
      stack.emplace_back(neighbor);
    }
  }
}

template <typename vertex_type>
auto get_depth_first_search_tree(const graph<vertex_type> &g, size_t s) {
  tree<vertex_type> t;

  depth_first_search<vertex_type>(g, s, [&t](auto edge) { t.add_edge(edge); });
  return t;
}
template <typename vertex_type> bool is_connected(const graph<vertex_type> &g) {
  size_t tree_vertex_num = 0;
  depth_first_search<vertex_type>(
      g, g.get_vertices()[0],
      [&tree_vertex_num](auto edge) { tree_vertex_num++; });
  return tree_vertex_num + 1 == g.get_vertex_number();
}
} // namespace cyy::algorithm
