/*!
 * \file graph.hpp
 *
 * \brief implements graph
 */

#pragma once

#include <algorithm>
#include <memory>
#include <vector>

#include "graph_base.hpp"
namespace cyy::algorithm {

  template <typename vertex_type>
  class graph : public graph_base<vertex_type, false> {
  public:
    using graph_base<vertex_type, false>::graph_base;
  };
  template <typename vertex_type>
  class directed_graph : public graph_base<vertex_type, true> {
  public:
    using graph_base<vertex_type, true>::graph_base;

    graph<vertex_type> get_underlying_graph() const {
      graph<vertex_type> g;
      for (auto &[from_vertex, to_vertices] : this->weighted_adjacent_list) {
        for (auto &to_vertex : to_vertices) {
          g.add_edge({this->get_vertex(to_vertex.first),
                              this->get_vertex(from_vertex), to_vertex.second});
        }
      }
      for(auto const &n:this->get_vertices()) {
        g.add_vertex(n);
      }
      return g;
    }

    directed_graph get_transpose() const {
      directed_graph transpose;
      for (auto &[from_vertex, to_vertices] : this->weighted_adjacent_list) {
        for (auto &to_vertex : to_vertices) {
          transpose.add_edge({this->get_vertex(to_vertex.first),
                              this->get_vertex(from_vertex), to_vertex.second});
        }
      }
      return transpose;
    }
    std::vector<size_t> get_indegrees() const {
      std::vector<size_t> indegrees(this->get_next_vertex_index(), 0);
      for (auto const &[_, adjacent_vertices] : this->weighted_adjacent_list) {
        for (auto const &[to_index, weight] : adjacent_vertices) {
          indegrees[to_index]++;
        }
      }
      return indegrees;
    }
  };

} // namespace cyy::algorithm
