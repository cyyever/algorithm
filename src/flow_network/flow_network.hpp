/*!
 * \file graph.hpp
 *
 * \brief implements graph
 */

#pragma once

#include <algorithm>
#include <map>
#include <memory>
#include <numeric>
#include <unordered_set>
#include <utility>
#include <vector>

#include "graph/graph.hpp"
#include "hash.hpp"
namespace cyy::algorithm {
  template <typename vertex_type = size_t>
  class flow_network : public directed_graph<vertex_type> {
  public:
    flow_network(directed_graph<vertex_type> graph_, vertex_type source_,
                 vertex_type sink_,
                 const std::unordered_map<std::pair<vertex_type, vertex_type>,
                                          float> &capacity_)
        : graph(std::move(graph_)) {
      source = graph.get_vertex_index(source_);
      sink = graph.get_vertex_index(sink_);

      for (auto &[edge, capacity] : capacity_) {
        assert(capacity >= 0);
        capacities[{graph.get_vertex_index(edge.first),
                    graph.get_vertex_index(edge.second)}] = capacity;
      }

      // init flow to zero
      graph.set_all_weights(0);
    }
    float get_flow_value() const {
      float source_flow = 0;
      for (auto const &[_, edge_flow] : graph.get_adjacent_list(source)) {
        source_flow += edge_flow;
      }
      return source_flow;
    }

    void max_flow_by_ford_fulkerson() {
      auto residual_graph = get_residual_graph();
      while (true) {
        auto path = residual_graph.get_s_t_path();
        if (path.empty()) {
          break;
        }
        float bottleneck = std::numeric_limits<float>::max();
        for (size_t i = 0; i + 1 < path.size(); i++) {
          indexed_edge e{path[i], path[i + 1]};
          bottleneck = std::min(bottleneck, residual_graph.capacities[e]);
        }
        for (size_t i = 0; i + 1 < path.size(); i++) {
          indexed_edge e{path[i], path[i + 1]};
          if (residual_graph.is_backward_edge(e)) {
            auto new_weight = graph.get_weight(e.reverse()) - bottleneck;
            graph.set_weight(e.reverse(), new_weight);
            if (new_weight == 0) {
              residual_graph.remove_edge(e);
            }
          } else {
            auto new_weight = graph.get_weight(e) + bottleneck;
            graph.set_weight(e, new_weight);
            if (capacities[e] == new_weight) {
              residual_graph.remove_edge(e);
            }
          }
        }
      }
#ifndef NDEBUG
      check_flow();
#endif
    }

  private:
    flow_network() = default;
    bool check_flow() {
      // capacity condition
      for (const auto &[from_index, adjacent_vertices] :
           graph.get_adjacent_list()) {
        for (const auto &[to_index, weight] : adjacent_vertices) {
          if (weight > capacities.at({from_index, to_index}))
            return false;
        }
      }
      // conservation condition
      auto matrix = graph.get_adjacent_matrix();
      for (size_t i = 0; i < matrix.size(); i++) {
        float sum = std::accumulate(matrix[i].begin(), matrix[i].end(), 0);
        float sum2 = 0;
        for (size_t j = 0; j < matrix.size(); j++) {
          sum2 += matrix[j][i];
        }
        if (sum != sum2) {
          return false;
        }
      }
      return true;
    }

    flow_network<vertex_type> get_residual_graph() const {
      if (!backward_edges.empty()) {
        throw std::runtime_error(
            "can't get residual graph from a residual graph");
      }
      auto residual_graph = *this;
      residual_graph.graph.clear_edges();
      residual_graph.capacities.clear();
      graph.foreach_edge([this, &residual_graph](auto const &edge) {
        auto weight = graph.get_weight(edge);
        auto leftover_capacity = capacities.at(edge) - weight;
        if (leftover_capacity > 0 || weight > 0) {
          auto first_vertex = graph.get_vertex(edge.first);
          auto second_vertex = graph.get_vertex(edge.second);
          auto new_edge =
              cyy::algorithm::edge<vertex_type>{first_vertex, second_vertex, 0};
          if (leftover_capacity > 0) {
            residual_graph.graph.add_edge(new_edge);
            residual_graph.capacities[edge] = leftover_capacity;
          }
          if (weight > 0) {
            residual_graph.graph.add_edge(new_edge.reverse());
            residual_graph.capacities[edge.reverse()] = weight;
            residual_graph.backward_edges.insert(edge.reverse());
          }
        }
      });

      return residual_graph;
    }
    std::vector<size_t> get_s_t_path() const {
      std::vector<size_t> parent(graph.get_next_vertex_index(),
                                 graph.get_next_vertex_index());
      graph.recursive_depth_first_search(source,
                                         [&parent, this](auto u, auto v) {
                                           parent[v] = u;

                                           return v == sink;
                                         });
      std::vector<size_t> path;
      path.reserve(graph.get_next_vertex_index());
      auto vertex = sink;
      while (vertex != source) {
        if (vertex == graph.get_next_vertex_index()) {
          return {};
        }
        path.push_back(vertex);
        vertex = parent[vertex];
      }
      path.push_back(source);
      std::ranges::reverse(path);

      return path;
    }

    bool is_backward_edge(const indexed_edge &e) const {
      return backward_edges.contains(e);
    }

    void remove_edge(const indexed_edge &e) {
      graph.remove_edge(e);
      capacities.erase(e);
      backward_edges.erase(e);
    }

  private:
    directed_graph<vertex_type> graph;
    size_t source;
    size_t sink;
    std::unordered_map<indexed_edge, float> capacities;
    std::unordered_set<indexed_edge> backward_edges;
  };

} // namespace cyy::algorithm
