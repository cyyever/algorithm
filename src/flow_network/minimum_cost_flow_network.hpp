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
    using flow_fun_type = std::unordered_map<indexed_edge, double>;
    using capacity_and_cost_fun_type =
        std::unordered_map<std::pair<vertex_type, vertex_type>,
                           std::tuple<double, double, double>>;
    minimum_cost_flow_network(
        const capacity_and_cost_fun_type &capacity_and_cost_map,
        std::unordered_map<vertex_type, double> demand_) {
      for (const auto &[e, capacity_and_cost] : capacity_and_cost_map) {
        graph.add_edge({e.first, e.second});
        auto indexed_e = indexed_edge{graph.get_vertex_index(e.first),
                                      graph.get_vertex_index(e.second)};
        auto [lower_capacity, upper_capacity, cost] = capacity_and_cost;
        if (lower_capacity > upper_capacity) {
          throw std::runtime_error(
              "lower capacity is larger than upper capacity");
        }
        lower_capacities[indexed_e] = lower_capacity;
        upper_capacities[indexed_e] = upper_capacity;
        costs[indexed_e] = cost;
      }
      double total_demand = 0;
      for (auto const &[_, d] : demand_) {
        total_demand += d;
      }
      if (total_demand != 0) {
        throw std::logic_error("total demand must be 0");
      }

      for (auto const &[vertex, index] : graph.get_vertices_and_indices()) {
        demand[index] = demand_.at(vertex);
      }
    }

    flow_fun_type min_cost_flow_by_network_simplex() {
      flow_fun_type flow;
      auto ts = get_strongly_feasible_tree_structure();
      while(true) {
        flow = determin_flow(ts);
        determin_potential(ts);
        break;
        bool forward_direction=true;
        indexed_edge violating_edge;
        size_t u,v;
        auto it=std::ranges::find_if(ts.U,[this](auto const &e){
            return reduced_costs[e]>0;
            });
        if(it!=ts.U.end()) {
          violating_edge=*it;
          forward_direction=false;
        } else {
          it=std::ranges::find_if(ts.L,[this](auto const &e){
              return reduced_costs[e]<0;
              });
          if(it!=ts.L.end()) {
            violating_edge=*it;
          } else {
            break;
          } 
        } 
      }
      return flow;
    }

    bool check_flow(const flow_fun_type &flow) {
      decltype(demand) amount;

      graph.foreach_edge([this, &flow, &amount](auto const &e) {
        auto edge_flow = flow.at(e);
        amount[e.second] += edge_flow;
        amount[e.first] -= edge_flow;
      });
      return amount == demand;
    }

    bool check_feasible_flow(const flow_fun_type &flow) {
      if (!check_flow(flow)) {
        return false;
      }
      bool flag = true;

      graph.foreach_edge([this, &flag, &flow](auto const &e) {
        auto lower_capacity = lower_capacities.at(e);
        auto upper_capacity = upper_capacities.at(e);
        auto edge_flow = flow.at(e);
        if (edge_flow > upper_capacity || edge_flow < lower_capacity) {
          flag = false;
        }
      });
      return flag;
    }

    struct tree_structure {
      in_directed_tree<vertex_type> T;
      std::vector<indexed_edge> L;
      std::vector<indexed_edge> U;
    };

  private:
    minimum_cost_flow_network() = default;

    flow_fun_type determin_flow(const tree_structure &ts) {
      flow_fun_type flow;
      for (auto const &e : ts.U) {
        flow[e] = upper_capacities[e];
      }
      for (auto const &e : ts.L) {
        flow[e] = lower_capacities[e];
      }

      auto remain_demand = demand;
      for (auto &[_, v] : remain_demand) {
        v = 0;
      }
      for (auto const &[e, edge_flow] : flow) {
        remain_demand[e.first] -= edge_flow;
        remain_demand[e.second] += edge_flow;
      }

      auto leaves = ts.T.get_leaves();
      while (!leaves.empty()) {
        decltype(leaves) next_leaves;
        for (auto &leaf : leaves) {
          auto parent_opt = ts.T.parent(leaf);
          if (!parent_opt) {
            continue;
          }

          auto parent = *parent_opt;
          if (costs.count({leaf, parent})) {
            auto edge_flow = remain_demand[leaf] - demand[leaf];
            flow[{leaf, parent}] = edge_flow;
            remain_demand[parent] += edge_flow;
          } else {
            auto edge_flow = demand[leaf] - remain_demand[leaf];
            flow[{parent, leaf}] = edge_flow;
            remain_demand[parent] -= edge_flow;
          }
          next_leaves.push_back(parent);
        }
        leaves = std::move(next_leaves);
      }
      assert(flow.size() == costs.size());
      assert(check_flow(flow));
      assert(check_feasible_flow(flow));
      return flow;
    }
    void determin_potential(const tree_structure &ts) {
      auto T = ts.T.get_transpose();
      auto root = T.get_root();
      potential[root] = 0;

      T.breadth_first_search(root, [this](size_t u, size_t v, double) {
        auto it = costs.find({u, v});
        if (it != costs.end()) {
          potential[v] = potential[u] + it->second;
        } else {
          it = costs.find({v, u});
          potential[v] = potential[u] - it->second;
        }
        return false;
      });
      assert(potential.size() == demand.size());
      for (const auto &[e, cost] : costs) {
        reduced_costs[e] = cost + potential[e.first] - potential[e.second];
      }
    }

    tree_structure get_strongly_feasible_tree_structure() {
      static constexpr auto artificial_vertex_name = "_____artificial_vertex";
      if (!artificial_vertex_opt) {
        double max_abs_cost = std::numeric_limits<double>::min();
        for (auto &[_, cost] : costs) {
          max_abs_cost = std::max(std::fabs(cost), max_abs_cost);
        }
        double C = max_abs_cost * graph.get_vertex_number() + 1;

        auto B = demand;
        graph.foreach_edge([this, &B](auto const &e) {
          B[e.first] += lower_capacities[e];
          B[e.second] -= lower_capacities[e];
        });
        auto artificial_vertex = graph.add_vertex(artificial_vertex_name);
        artificial_vertex_opt = artificial_vertex;
        demand[artificial_vertex] = 0;

        for (auto [v, b] : B) {
          if (b > 0) {
            graph.add_edge(
                {graph.get_vertex(artificial_vertex), graph.get_vertex(v)});
            lower_capacities[{artificial_vertex, v}] = 0;
            upper_capacities[{artificial_vertex, v}] = b + 1;
            costs[{artificial_vertex, v}] = C;
          } else {
            graph.add_edge(
                {graph.get_vertex(v), graph.get_vertex(artificial_vertex)});
            lower_capacities[{v, artificial_vertex}] = 0;
            upper_capacities[{v, artificial_vertex}] = -b + 1;
            costs[{v, artificial_vertex}] = C;
          }
        }
      }
      std::vector<indexed_edge> L;
      auto T = graph;
      T.clear_edges();
      graph.foreach_edge([&L, &T, this](auto const &e) {
        if (e.first == artificial_vertex_opt) {
          T.add_edge(graph.get_edge(e).reverse());
        } else if (e.second == artificial_vertex_opt) {
          T.add_edge(graph.get_edge(e));
        } else {
          L.emplace_back(e);
        }
      });
      return {in_directed_tree<vertex_type>(std::move(T),
                                            artificial_vertex_name, false),
              std::move(L),
              {}};
    }

  private:
    directed_graph<vertex_type> graph;
    flow_fun_type upper_capacities;
    flow_fun_type lower_capacities;
    flow_fun_type costs;
    flow_fun_type reduced_costs;
    std::unordered_map<size_t, double> demand;
    std::unordered_map<size_t, double> potential;
    std::optional<size_t> artificial_vertex_opt;
  };

} // namespace cyy::algorithm
