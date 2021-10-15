/*!
 * \file graph.hpp
 *
 * \brief implements graph
 */

#pragma once

#include <algorithm>
#include <memory>
#include <vector>
#include <map>
#include <numeric>
#include <utility>


#include "graph/graph.hpp"
#include "hash.hpp"
namespace cyy::algorithm {
  template <typename vertex_type = size_t>
  class flow_network : public directed_graph<vertex_type> {
  public:
    flow_network(directed_graph<vertex_type> graph_, vertex_type source_,
                 vertex_type sink_,
                 const std::unordered_map<std::pair<vertex_type,vertex_type>, float>& capacity_)
        : graph(std::move(graph_)) {
      source = graph.get_vertex_index(source_);
      sink = graph.get_vertex_index(sink_);

      for (auto &[edge, capacity] :capacity_) {
        assert(capacity >= 0);
        capacities[{graph.get_vertex_index(edge.first),graph.get_vertex_index(edge.second)}] = capacity;
      }

      // init flow to zero
      graph.change_all_weights(0);
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
      //conservation condition
      auto matrix=graph.get_adjacent_matrix();
      for(size_t i=0;i<matrix.size();i++) {
        float sum=std::accumulate(matrix[i].begin(),matrix[i].end(),0);
      }
    }

  private:
    directed_graph<vertex_type> graph;
    size_t source;
    size_t sink;
    std::unordered_map<std::pair<size_t,size_t>, float> capacities;
  };

} // namespace cyy::algorithm
