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
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

#include "graph/graph.hpp"
#include "hash.hpp"
namespace cyy::algorithm {
  template <typename vertex_type = size_t>
  class minimum_cost_flow_network : public directed_graph<vertex_type> {
  public:
    using capacity_fun_type =
        std::unordered_map<std::pair<vertex_type, vertex_type>, double>;
    minimum_cost_flow_network(directed_graph<vertex_type> graph_,
                              const capacity_fun_type &lower_capacities_,
                              const capacity_fun_type &upper_capacities_,
                              const capacity_fun_type &costs_,
                                  std::unordered_map<vertex_type, double>
                                      demand_ )
        : graph(std::move(graph_)) {
      graph.foreach_edge([this, &upper_capacities_,
                          &lower_capacities_,&costs_](auto const &e) {
        auto real_edge =
            std::pair{graph.get_vertex(e.first), graph.get_vertex(e.second)};
        auto lower_capacity = lower_capacities_.at(real_edge);
        auto upper_capacity = upper_capacities_.at(real_edge);
        if (lower_capacity > upper_capacity) {
          throw std::runtime_error(
              "lower capacity is larger than upper capacity");
        }

        lower_capacities[e] = lower_capacity;
        upper_capacities[e] = upper_capacity;
        costs[e]=costs_.at(real_edge);
      });
      double total_demand=0;
      for(auto const &[_,d]:demand_) {
        total_demand+=d;
      }
      if(total_demand!=0) {
        throw std::logic_error("total demand must be 0");
      }

      for(auto const &[vertex,index]:graph.get_vertices_and_indices()) {
        demand[index]=demand_.at(vertex);
      }
    }
    bool check_feasible_flow(const std::unordered_map<indexed_edge, double> &flow) { 
      std::unordered_map<size_t, double> amount;
      bool flag=true;

      graph.foreach_edge([this, &flow,&flag,&amount](auto const &e) {
        auto lower_capacity = lower_capacities.at(e);
        auto upper_capacity = upper_capacities.at(e);
        auto edge_flow=flow.at(e);
        if(edge_flow>upper_capacity || edge_flow<lower_capacity) {
        flag=false;
        return;
        }
        amount[e.second]+=edge_flow;
        amount[e.first]-=edge_flow;
      });
      if(!flag) {
        return false;
      }
      return amount==demand;
    }

  private:
    minimum_cost_flow_network() = default;

  private:
    directed_graph<vertex_type> graph;
    std::unordered_map<indexed_edge, double> upper_capacities;
    std::unordered_map<indexed_edge, double> lower_capacities;
    std::unordered_map<indexed_edge, double> costs;
    std::unordered_map<size_t, double> demand;
  };

} // namespace cyy::algorithm
