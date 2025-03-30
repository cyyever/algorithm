/*!
 * \file graph.hpp
 *
 * \brief implements graph
 */

#pragma once

#include "graph_base.hpp"
namespace cyy::algorithm {

  template <typename vertex_type, typename weight_type = double>
  class graph : public graph_base<vertex_type, false, weight_type> {
  public:
    using graph_base<vertex_type, false, weight_type>::graph_base;
    using typename graph_base<vertex_type, false, weight_type>::edge_type;
  };
  template <typename vertex_type, typename weight_type = double>
  class directed_graph : public graph_base<vertex_type, true, weight_type> {
  public:
    using graph_base<vertex_type, true, weight_type>::graph_base;
    using typename graph_base<vertex_type, true, weight_type>::edge_type;

    auto get_underlying_graph() const {
      graph<vertex_type, weight_type> g;
      g.set_vertex_pool(this->get_vertex_pool());
      for (auto const &e : this->foreach_edge()) {
        g.add_edge(this->get_edge(e));
      }
      return g;
    }

    directed_graph get_reverse() const {
      directed_graph reverse = *this;
      reverse.clear_edges();
      for (auto const &e : this->foreach_edge()) {
        auto edge = this->get_edge(e).reverse();
        reverse.add_edge(edge);
      }
      return reverse;
    }

    auto get_containing_edges(size_t vertex_index) const noexcept {
      return std::ranges::filter_view(
          this->foreach_edge_with_weight(),
          [vertex_index](auto const &e) { return e.contains(vertex_index); });
    }
    std::vector<size_t> get_indegrees() const {
      std::vector<size_t> indegrees(this->get_vertex_number(), 0);
      for (auto const &[_, adjacent_vertices] : this->weighted_adjacent_list) {
        for (auto const &[to_index, weight] : adjacent_vertices) {
          indegrees[to_index]++;
        }
      }
      return indegrees;
    }
  };

} // namespace cyy::algorithm
