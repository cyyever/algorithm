/*!
 * \file flow_network.hpp
 *
 * \brief implements flow networks
 */
#pragma once

#include "graph/graph.hpp"
#include "graph/graph_base.hpp"
#include "graph/path.hpp"
namespace cyy::algorithm {
  template <typename vertex_type = size_t, typename weight_type = double>
  class flow_network {
  private:
    enum class s_t_path_type : std::uint8_t { random = 0, shortest };

  public:
    using edge_type = weighted_edge<vertex_type, weight_type>;
    using capacity_fun_type = std::vector<edge_type>;
    flow_network(const capacity_fun_type &capacities_, const vertex_type& source_,
                 const vertex_type& sink_) {
      for (auto const &e : capacities_) {
        graph.add_edge(e);

        auto capacity = e.weight;
        if (capacity < 0) {
          throw std::runtime_error("capacity should be non-negative");
        }
        capacities[graph.get_edge(e)] = capacity;
      }
      if (!graph.has_vertex(source_)) {
        throw std::runtime_error("source is not in graph");
      }
      source = graph.get_vertex_index(source_);
      if (!graph.has_vertex(sink_)) {
        throw std::runtime_error("sink is not in graph");
      }
      sink = graph.get_vertex_index(sink_);
      if (source == sink) {
        throw std::runtime_error("sink is the same as source");
      }

      // init flow to zero
      graph.fill_weight(0);
    }
    weight_type get_flow_value() const {
      weight_type value{};
      for (auto const &e : graph.get_containing_edges(source)) {
        if (e.first == source) {
          value += e.weight;
        } else {
          value -= e.weight;
        }
      }
      return value;
    }

    void max_flow() { max_flow_by_edmonds_karp(); }
    void max_flow_by_edmonds_karp() {
      max_flow_by_ford_fulkerson<s_t_path_type::shortest>();
    }
    // may not terminate when there is a real capacity.
    template <s_t_path_type path_type = s_t_path_type::random>
    void max_flow_by_ford_fulkerson() {
      auto residual_graph = get_residual_graph();
      while (true) {
        std::vector<size_t> path;
        if constexpr (path_type == s_t_path_type::shortest) {
          path = convert_parent_list_to_path(
              shortest_path_by_edge_number(residual_graph, source), source,
              sink);
        } else {
          path = get_path(residual_graph, source, sink);
        }
        if (path.empty()) {
          break;
        }
        augment(residual_graph, path);
      }
#ifndef NDEBUG
      assert(check_flow());
#endif
    }

    std::pair<std::unordered_set<size_t>, std::unordered_set<size_t>>
    get_minimum_capacity_s_t_cut() {
      max_flow();
      auto residual_graph = get_residual_graph();
      std::unordered_set<size_t> s_set;
      std::unordered_set<size_t> t_set;
      s_set.insert(source);
      residual_graph.recursive_depth_first_search(source,
                                                  [&s_set](auto, auto v) {
                                                    s_set.insert(v);
                                                    return false;
                                                  });
      for (size_t v = 0; v < graph.get_vertex_number(); v++) {
        if (!s_set.contains(v)) {
          t_set.insert(v);
        }
      }
#ifndef NDEBUG
      assert(t_set.contains(sink));
#endif
      return {s_set, t_set};
    }

    auto const &get_graph() const { return graph; }

  private:
    flow_network() = default;
    bool check_flow() const {
      // capacity condition
      for (auto const &e : graph.foreach_edge_with_weight()) {
        if (e.weight > capacities.at(e)) {
          return false;
        }
      }
      // conservation condition
      auto matrix = graph.get_adjacent_matrix();
      for (size_t i = 0; i < matrix.size(); i++) {
        if (i == source || i == sink) {
          continue;
        }
        weight_type sum =
            std::accumulate(matrix[i].begin(), matrix[i].end(), weight_type{});
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
      auto residual_graph = this->graph;
      for (auto const &edge : graph.foreach_edge()) {
        modified_residual_edge(residual_graph, edge);
      }
      return residual_graph;
    }

    void remove_edge(const indexed_edge &e) {
      graph.remove_edge(e);
      capacities.erase(e);
    }

    void augment(auto &residual_graph, const path_type &path) {
      auto bottleneck = residual_graph.get_extreme_weight(path);
      assert(bottleneck > 0);
      for (size_t i = 0; i + 1 < path.size(); i++) {
        const indexed_edge indexed_e{path[i], path[i + 1]};
        // backward
        if (!graph.has_edge(indexed_e)) {
          const auto forward_edge = indexed_e.reverse();
          auto new_weight = graph.get_weight(forward_edge) - bottleneck;
          graph.set_weight(forward_edge, new_weight);
        } else {
          // forward
          auto const &forward_edge = indexed_e;
          auto new_weight = graph.get_weight(forward_edge) + bottleneck;
          graph.set_weight(forward_edge, new_weight);
        }
        modified_residual_edge(residual_graph, indexed_e);
      }
    }

    void modified_residual_edge(auto &residual_graph,
                                const indexed_edge &e) const {
      residual_graph.remove_edge(e);
      residual_graph.remove_edge(e.reverse());
      auto origin_edge = graph.get_edge(e);
      if (origin_edge.weight > 0) {
        residual_graph.add_edge(origin_edge.reverse());
      }
      auto leftover_capacity = capacities.at(e) - origin_edge.weight;
      if (leftover_capacity > 0) {
        origin_edge.weight = leftover_capacity;
        residual_graph.add_edge(origin_edge);
      }
    }

    directed_graph<vertex_type, weight_type> graph;
    size_t source{};
    size_t sink{};
    std::unordered_map<indexed_edge, weight_type> capacities;
  };

} // namespace cyy::algorithm
