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
    bool check_flow() {
      // capacity condition
      for (const auto &[from_index, adjacent_vertices] :
           graph.get_weighted_adjacent_list()) {
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
    std::vector<size_t> get_s_t_path() const {
      std::vector<size_t> path(graph.get_next_vertex_index(), 0);
      graph.recursive_depth_first_search(source, [&path, this](auto u, auto v) {
        path[v] = u;
        return v == sink;
      });
      return path;
    }

    flow_network<vertex_type> get_residual_graph() const {
      if (!backward_capacities.empty()) {
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
            residual_graph.backward_capacities[edge.reverse()] = weight;
          }
        }
      });

      return residual_graph;
    }

  private:
    flow_network() = default;

  private:
    directed_graph<vertex_type> graph;
    size_t source;
    size_t sink;
    std::unordered_map<indexed_edge, float> capacities;
    std::unordered_map<indexed_edge, float> backward_capacities;
  };

} // namespace cyy::algorithm
