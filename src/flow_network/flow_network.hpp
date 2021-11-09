/*!
 * \file flow_network.hpp
 *
 * \brief implements flow networks
 */
#pragma once

#include <algorithm>
#include <memory>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

#include <range/v3/all.hpp>

#include "graph/graph.hpp"
#include "hash.hpp"
namespace cyy::algorithm {
  template <typename vertex_type = size_t, typename weight_type = double>
  class flow_network {
  public:
    using edge_type = edge<vertex_type, weight_type>;
    using capacity_fun_type =
        std::unordered_map<std::pair<vertex_type, vertex_type>, weight_type>;
    flow_network(directed_graph<vertex_type, weight_type> graph_,
                 vertex_type source_, vertex_type sink_,
                 const capacity_fun_type &capacities_)
        : graph(std::move(graph_)) {
      graph.rearrange_vertices();
      source = graph.get_vertex_index(source_);
      sink = graph.get_vertex_index(sink_);

      for (auto const &e : graph.foreach_edge()) {
        auto real_edge =
            std::pair{graph.get_vertex(e.first), graph.get_vertex(e.second)};
        auto capacity = capacities_.at(real_edge);

        if (capacity < 0) {
          throw std::runtime_error("capacity should be non-negative");
        }
        capacities[e] = capacity;
      }

      // init flow to zero
      graph.set_all_weights(0);
    }
    weight_type get_flow_value() const {
      weight_type source_flow = 0;
      for (auto const &[_, edge_flow] : graph.get_adjacent_list(source)) {
        source_flow += edge_flow;
      }
      return source_flow;
    }

    // may not terminate when there is a real capacity.
    void max_flow_by_ford_fulkerson(
        std::optional<std::function<std::vector<size_t>()>> get_s_t_path_fun =
            {}) {
      if (!get_s_t_path_fun.has_value()) {
        get_s_t_path_fun = std::bind(
            &flow_network<vertex_type, weight_type>::get_s_t_path, this);
      }
      auto residual_graph = get_residual_graph();
      while (true) {
        auto path = residual_graph.get_s_t_path();
        if (path.empty()) {
          break;
        }
        std::optional<weight_type> bottleneck_opt;
        for (size_t i = 0; i + 1 < path.size(); i++) {
          indexed_edge e{path[i], path[i + 1]};
          if (!bottleneck_opt.has_value()) {
            bottleneck_opt = residual_graph.capacities[e];
          } else {
            bottleneck_opt =
                std::min(*bottleneck_opt, residual_graph.capacities[e]);
          }
        }
        for (size_t i = 0; i + 1 < path.size(); i++) {
          indexed_edge e{path[i], path[i + 1]};
          if (residual_graph.is_backward_edge(e)) {
            auto new_weight =
                graph.get_weight(e.reverse()) - bottleneck_opt.value();
            graph.set_weight(e.reverse(), new_weight);
            if (new_weight == 0) {
              residual_graph.remove_edge(e);
            } else {
              residual_graph.capacities[e] = new_weight;
            }
          } else {
            auto new_weight = graph.get_weight(e) + bottleneck_opt.value();
            graph.set_weight(e, new_weight);
            if (capacities[e] == new_weight) {
              residual_graph.remove_edge(e);
            } else {
              residual_graph.capacities[e] -= bottleneck_opt.value();
            }
          }
        }
      }
#ifndef NDEBUG
      assert(check_flow());
#endif
    }
    void max_flow_by_edmonds_karp() {
      return max_flow_by_ford_fulkerson(
          std::bind(&flow_network::get_shortest_s_t_path, this));
    }

    std::pair<std::set<size_t>, std::set<size_t>>
    get_minimum_capacity_s_t_cut() {
      max_flow_by_ford_fulkerson();
      auto residual_graph = get_residual_graph();
      std::set<size_t> s_set;
      std::set<size_t> t_set;
      s_set.insert(source);
      residual_graph.graph.recursive_depth_first_search(source,
                                                        [&s_set](auto, auto v) {
                                                          s_set.insert(v);
                                                          return false;
                                                        });
      for (auto v : graph.get_vertex_indices()) {
        if (!s_set.contains(v)) {
          t_set.insert(v);
        }
      }
#ifndef NDEBUG
      assert(t_set.contains(sink));
#endif
      return {s_set, t_set};
    }

  private:
    flow_network() = default;
    bool check_flow() {
      graph.rearrange_vertices();
      // capacity condition
      for (auto const &[indexed_edge, weight] :
           graph.foreach_edge_with_weight()) {
        if (weight > capacities.at(indexed_edge)) {
          return false;
        }
      }
      // conservation condition
      auto matrix = graph.get_adjacent_matrix();
      for (size_t i = 0; i < matrix.size(); i++) {
        if (i == source || i == sink) {
          continue;
        }
        weight_type sum = ::ranges::accumulate(matrix[i], weight_type{});
        weight_type sum2 = 0;
        for (size_t j = 0; j < matrix.size(); j++) {
          sum2 += matrix[j][i];
        }
        if (sum != sum2) {
          return false;
        }
      }
      return true;
    }

    auto get_residual_graph() const {
      if (!backward_edges.empty()) {
        throw std::runtime_error(
            "can't get residual graph from a residual graph");
      }
      auto residual_graph = *this;
      residual_graph.graph.clear_edges();
      residual_graph.capacities.clear();
      for (auto const &edge : graph.foreach_edge()) {
        auto weight = graph.get_weight(edge);
        auto leftover_capacity = capacities.at(edge) - weight;
        if (leftover_capacity > 0 || weight > 0) {
          auto first_vertex = graph.get_vertex(edge.first);
          auto second_vertex = graph.get_vertex(edge.second);
          auto new_edge = edge_type{first_vertex, second_vertex, 0};
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
      }

      return residual_graph;
    }
    std::vector<size_t> get_shortest_s_t_path() const {
      std::vector<size_t> parent(graph.get_next_vertex_index(),
                                 graph.get_next_vertex_index());
      graph.breadth_first_search(source, [&parent, this](auto u, auto v, auto) {
        parent[v] = u;
        return v == sink;
      });
      return convert_parents_to_path(parent);
    }
    std::vector<size_t> get_s_t_path() const {
      std::vector<size_t> parent(graph.get_next_vertex_index(),
                                 graph.get_next_vertex_index());
      graph.recursive_depth_first_search(source,
                                         [&parent, this](auto u, auto v) {
                                           parent[v] = u;
                                           return v == sink;
                                         });
      return convert_parents_to_path(parent);
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
    std::vector<size_t>
    convert_parents_to_path(const std::vector<size_t> &parent) const {
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

  private:
    directed_graph<vertex_type, weight_type> graph;
    size_t source;
    size_t sink;
    std::unordered_map<indexed_edge, weight_type> capacities;
    std::unordered_set<indexed_edge> backward_edges;
  };

} // namespace cyy::algorithm
