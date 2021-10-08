
/*!
 * \file graph_algorithm.hpp
 *
 * \brief implements graph_algorithm
 */

#pragma once

#include <cassert>
#include <functional>
#include <list>
#include <vector>

#include "graph.hpp"

namespace cyy::algorithm {

// breadth first search in g from s
template <typename vertex_type>
void breadth_first_search(const graph<vertex_type> &g, size_t s,
                          std::function<void(size_t, size_t, float)> edge_fun) {
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
        edge_fun(u, v, weight);
        continue;
      }
    }
  }
}

template <typename vertex_type>
auto get_breadth_first_search_tree(const graph<vertex_type> &g, size_t s) {

  graph<vertex_type> t;
  breadth_first_search<vertex_type>(g, s, [&](auto u, auto v, float weight) {
    t.add_edge({g.get_vertex(u), g.get_vertex(v), weight});
  });
  return t;
}

// depth first search in g from s
template <typename vertex_type>
void recursive_depth_first_search(
    const graph<vertex_type> &g, size_t s,
    std::function<void(size_t, size_t)> after_edge_fun) {

  std::vector<bool> explored(g.get_next_vertex_index(), false);
  auto search_fun = [&](auto &&self, size_t u) {
    if (explored[u]) {
      return;
    }
    explored[u] = true;
    for (auto const &neighbor : g.get_adjacent_list(u)) {
      self(self, neighbor.first);
      after_edge_fun(u, neighbor.first);
    }
  };
  search_fun(search_fun, s);
}

// depth first search in g from s
template <typename vertex_type>
void depth_first_search(const graph<vertex_type> &g, size_t s,
                        std::function<void(size_t, size_t, float)> edge_fun) {
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
      edge_fun(parent[u], u, weight);
    }
    for (auto const &neighbor : g.get_adjacent_list(u)) {
      parent[neighbor.first] = u;
      stack.emplace_back(neighbor);
    }
  }
}

template <typename vertex_type>
auto get_depth_first_search_tree(const graph<vertex_type> &g, size_t s) {
  graph<vertex_type> t;

  depth_first_search<vertex_type>(g, s, [&](auto u, auto v, float weight) {
    t.add_edge({g.get_vertex(u), g.get_vertex(v), weight});
  });
  return t;
}
template <typename vertex_type> bool is_connected(const graph<vertex_type> &g) {
  size_t edge_num = 0;
  depth_first_search<vertex_type>(
      g, g.get_vertices()[0],
      [&edge_num](auto u, auto v, float weight) { edge_num++; });
  return edge_num + 1 == g.get_vertex_number();
}

template <typename vertex_type> bool is_tree(const graph<vertex_type> &g) {
  size_t edge_num = 0;
  depth_first_search<vertex_type>(
      g, *g.get_vertices().begin(),
      [&edge_num](auto u, auto v, float weight) { edge_num++; });
  return edge_num + 1 == g.get_vertex_number() &&
         edge_num == g.get_edge_number();
}

template <typename vertex_type> void shortest_path_dijkstra(const graph<vertex_type> &g,size_t s) {

 
  

}
}
// namespace cyy::algorithm
