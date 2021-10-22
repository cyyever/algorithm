/*!
 * \file graph.hpp
 *
 * \brief implements graph
 */

#pragma once

#include "graph.hpp"

namespace cyy::algorithm {

  template <typename vertex_type>
  class DAG : public directed_graph<vertex_type> {
  public:
    using directed_graph<vertex_type>::directed_graph;
    using edge_type = typename directed_graph<vertex_type>::edge_type;

    template <std::ranges::input_range U>
    requires std::same_as<edge_type, std::ranges::range_value_t<U>>
    explicit DAG(U edges, bool check = false)
        : directed_graph<vertex_type>(edges) {
      if (check && !get_topological_ordering()) {
        throw std::logic_error("not a DAG");
      }
    }

    explicit DAG(directed_graph<vertex_type> g, bool check = false)
        : directed_graph<vertex_type>(std::move(g)) {
      if (check && !get_topological_ordering()) {
        throw std::logic_error("not a DAG");
      }
    }

    std::optional<std::vector<size_t>> get_topological_ordering() const {
      // Time Complexity is O(m+n)
      std::vector<size_t> order;
      order.reserve(this->get_vertex_number());
      std::vector<size_t> to_delete_vertices;
      auto indegrees = this->get_indegrees();
      for (size_t i = 0; i < indegrees.size(); i++) {
        if (indegrees[i] == 0) {
          to_delete_vertices.push_back(i);
        }
      }
      while (!to_delete_vertices.empty()) {
        auto u = to_delete_vertices.back();
        order.push_back(u);
        to_delete_vertices.pop_back();

        for (auto const &[to_index, weight] : this->get_adjacent_list(u)) {
          indegrees[to_index]--;
          if (indegrees[to_index] == 0) {
            to_delete_vertices.push_back(to_index);
          }
        }
      }
      // Not a DAG
      if (order.size() != this->get_vertex_number()) {
        return {};
      }
      return {order};
    }
  };

} // namespace cyy::algorithm
