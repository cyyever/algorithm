/*!
 * \file graph.hpp
 *
 * \brief implements graph
 */

#pragma once

#include <algorithm>
#include <memory>
#include <numeric>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

#include "graph/graph.hpp"
#include "graph/tree.hpp"
#include "hash.hpp"
namespace cyy::algorithm {
  template <typename vertex_type = size_t>
  class minimum_cost_flow_network : public directed_graph<vertex_type> {
  public:
    using capacity_fun_type =
        std::unordered_map<std::pair<vertex_type, vertex_type>, double>;
    using  capacity_and_cost_fun_type=
        std::unordered_map<std::pair<vertex_type, vertex_type>, 
     std::tuple< double,double,double >>;
    minimum_cost_flow_network( const capacity_and_cost_fun_type &capacity_and_cost_map,
                                  std::unordered_map<vertex_type, double>
                                      demand_ )
    {
      for(const auto &[e,capacity_and_cost]:capacity_and_cost_map) {
        graph.add_edge({e.first,e.second});
        auto indexed_e=
            indexed_edge{graph.get_vertex_index(e.first), graph.get_vertex_index(e.second)};
        auto [lower_capacity,upper_capacity,cost]=capacity_and_cost;
        if (lower_capacity > upper_capacity) {
          throw std::runtime_error(
              "lower capacity is larger than upper capacity");
        }
        lower_capacities[indexed_e] = lower_capacity;
        upper_capacities[indexed_e] = upper_capacity;
        costs[indexed_e] =cost;
      }
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

    void min_cost_flow_by_network_simplex() {
      get_strongly_feasible_tree_structure();
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

    struct tree_structure{
    std::vector<indexed_edge> T;
    std::vector<indexed_edge> L;
    std::vector<indexed_edge> U;
    };

  private:
    minimum_cost_flow_network() = default;

    tree_structure get_strongly_feasible_tree_structure() {
      if(!artificial_vertex_opt)  {
       double C= graph.get_max_weight() *graph.get_vertex_number()+1;
       auto B=demand;
       graph.foreach_edge([this,&B](auto const &e) {
         B[e.first]+=graph.get_weight(e);
         B[e.second]-=graph.get_weight(e);
       });
       auto artificial_vertex=graph.add_vertex("_____artificial_vertex");
       artificial_vertex_opt=artificial_vertex;
       demand[artificial_vertex]=0;

       for(auto [v,b]:B) {
         if(b>0) {
           graph.add_edge({graph.get_vertex(artificial_vertex),
              graph.get_vertex(v) });
           lower_capacities[{artificial_vertex,v}]=0;
           upper_capacities[{artificial_vertex,v}]=b+1;
           costs[{artificial_vertex,v}]=C;
         } else {
           graph.add_edge({graph.get_vertex(v),
              graph.get_vertex(artificial_vertex) });
           lower_capacities[{v,artificial_vertex}]=0;
           upper_capacities[{v,artificial_vertex}]=-b+1;
           costs[{v,artificial_vertex}]=C;
         }
       }
      }
    std::vector<indexed_edge> L;
    std::vector<indexed_edge> T;
       graph.foreach_edge([&L,&T,this](auto const &e) {
           if(e.first==artificial_vertex_opt || e.second==artificial_vertex_opt) {
           T.emplace_back(e);
           } else {
           L.emplace_back(e);
           }
    });
    return {std::move(T),std::move(L),{}};
    }

  private:
    directed_graph<vertex_type> graph;
    std::unordered_map<indexed_edge, double> upper_capacities;
    std::unordered_map<indexed_edge, double> lower_capacities;
    std::unordered_map<indexed_edge, double> costs;
    std::unordered_map<size_t, double> demand;
    std::optional<size_t> artificial_vertex_opt;
  };

} // namespace cyy::algorithm
