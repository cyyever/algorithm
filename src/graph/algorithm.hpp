
/*!
 * \file graph_algorithm.hpp
 *
 * \brief implements graph_algorithm
 */

#pragma once

#include <cassert>
#include <list>
#include <optional>
#include <ranges>
#include <unordered_map>
#include <vector>

#include "graph.hpp"
#include "tree.hpp"

namespace cyy::algorithm {
// breadth first search in g from s
#if 0
  template <typename vertex_type> 
    std::vector<path_node_ptr>
    breadth_first_search(const graph<vertex_type> &g,size_t s) {
      std::vector<path_node_ptr> paths(g.get_next_vertex_index(),{});
      std::vector<float> total_cost(g.get_next_vertex_index(),0);
      /* std::vector<bool> reached(g.get_next_vertex_index(),false); */
      /* reached[s]=true; */
      std::list<size_t> queue{s};
      while(!queue.empty()) {
        auto u=queue.front();
        queue.pop_front();
        for(auto const &neighbor:g.get_adjacent_list(u)) {
          auto const &[v,weight]=neighbor;
          if(!paths[v]) {
            paths[v]=std::make_shared<path_node>(v);
            paths[v]->prev=paths[u];
            total_cost[v]=total_cost[u]+weight;
            /* reached[v]=true; */
            queue.push_back(v);
            continue;
          }
          auto new_cost=total_cost[u]+weight;
          if(new_cost <total_cost[v]) {
            paths[v]->prev=paths[u];
            total_cost[v]=new_cost;
            queue.push_back(v);
            continue;
          }
        }
      }
      return paths;
    }
#endif
template <typename vertex_type>
tree<vertex_type> breadth_first_search(const graph<vertex_type> &g, size_t s) {
  std::vector<bool> discovered(g.get_next_vertex_index(), false);
  discovered[s] = true;
  std::list<size_t> queue{s};
  tree<vertex_type> t;

  while (!queue.empty()) {
    auto u = queue.front();
    queue.pop_front();
    for (auto const &neighbor : g.get_adjacent_list(u)) {
      auto const &[v, weight] = neighbor;
      if (!discovered[v]) {
        discovered[v] = true;
        queue.push_back(v);
        t.add_edge({g.get_vertex(u), g.get_vertex(v), weight});
        continue;
      }
    }
  }
  return t;
}
template <typename vertex_type>
tree<vertex_type> depth_first_search(const graph<vertex_type> &g, size_t s) {
  std::vector<bool> explored(g.get_next_vertex_index(), false);
  std::vector<std::pair<size_t, float>> stack{{s,0}};
  std::vector<size_t> parent(g.get_next_vertex_index(), 0);
  tree<vertex_type> t;

  while (!stack.empty()) {
    auto [u, weight] = stack.back();
    stack.pop_back();
    if (explored[u]) {
      continue;
    }
    explored[u] = true;
    if (u != s) {
      t.add_edge({g.get_vertex(parent[u]), g.get_vertex(u), weight});
    }
    for (auto const &neighbor : g.get_adjacent_list(u)) {
      auto const &[v, weight] = neighbor;
      parent[v] = u;
      stack.emplace_back(v, weight);
    }
  }
  return t;
}
} // namespace cyy::algorithm
