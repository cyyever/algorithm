
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
#include "heap.hpp"
#include "tree.hpp"
#include "union_find.hpp"

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

template <typename vertex_type>
auto shortest_path_dijkstra(const graph<vertex_type> &g, size_t s) {

  std::vector<float> distance(g.get_next_vertex_index(),
                              std::numeric_limits<float>::max());
  distance[s] = 0;
  std::vector<size_t> edge(g.get_next_vertex_index(), SIZE_MAX);
  heap<size_t, float> h;
  h.insert(s, 0);
  while (!h.empty()) {
    auto u = h.top();
    h.pop();
    for (auto [v, weight] : g.get_adjacent_list(u)) {
      assert(weight >= 0);
      if (distance[v] <= distance[u] + weight) {
        continue;
      }
      edge[v] = u;
      distance[v] = distance[u] + weight;
      if (h.contains(v)) {
        h.change_key(v, distance[v]);
      } else {
        h.insert(v, distance[v]);
      }
    }
  }
  return edge;
}
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
  return MST;
}

template <typename vertex_type> auto MST_kruskal(const graph<vertex_type> &g) {

  std::set<indexed_edge> edges;
  g.foreach_edge([&edges](auto edge) { edges.emplace(std::move(edge)); });
  union_find<size_t> connected_components(g.get_vertices());
  graph<vertex_type> MST;

  for (auto const &edge : edges) {
    auto [u, v, weight] = edge;
    auto u_component = connected_components.find(u);
    auto v_component = connected_components.find(v);
    if (u_component != v_component) {
      MST.add_edge({g.get_vertex(u), g.get_vertex(v), weight});
      connected_components.UNION(u_component, v_component);
    }
  }
  return MST;
}
} // namespace cyy::algorithm
