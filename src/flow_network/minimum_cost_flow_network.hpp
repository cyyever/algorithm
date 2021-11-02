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
#include <utility>
#include <vector>

#include "graph/tree.hpp"
#include "hash.hpp"
namespace cyy::algorithm {
  template <typename vertex_type = size_t, typename weight_type = double>
  class minimum_cost_flow_network {
  public:
    using edge_type = edge<weight_type>;
    using flow_fun_type = std::unordered_map<indexed_edge, weight_type>;
    using capacity_and_cost_fun_type =
        std::unordered_map<std::pair<vertex_type, vertex_type>,
                           std::tuple<weight_type, weight_type, weight_type>>;
    minimum_cost_flow_network(
        const capacity_and_cost_fun_type &capacity_and_cost_map,
        std::unordered_map<vertex_type, weight_type> demand_) {
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
        // assert(costs.contains({4,5}) || costs.contains({5,4}));
      }
      weight_type total_demand = 0;
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

    weight_type get_cost(const flow_fun_type &flow) const {
      weight_type total_cost = 0;
      for (auto const &[e, edge_flow] : flow) {
        total_cost += edge_flow * costs.at(e);
      }
      return total_cost;
    }

    std::optional<flow_fun_type> min_cost_flow_by_network_simplex() {
      flow_fun_type flow;
      auto ts = get_strongly_feasible_tree_structure();
      flow = determin_flow(ts);
      while (true) {
        assert(ts.T.get_underlying_graph().is_tree());
        determin_potential(ts);
        bool forward_direction = true;
        indexed_edge violating_edge;
        {
          auto it = std::ranges::find_if(
              ts.U, [this](auto const &e) { return reduced_costs[e] > 0; });
          if (it != ts.U.end()) {
            violating_edge = *it;
            ts.U.erase(it);
            forward_direction = false;
          } else {
            it = std::ranges::find_if(
                ts.L, [this](auto const &e) { return reduced_costs[e] < 0; });
            if (it != ts.L.end()) {
              violating_edge = *it;
              ts.L.erase(it);
            } else {
              break;
            }
          }
        }
        assert(costs.contains(violating_edge));
        // create cycle
        auto ancestor =
            ts.T.nearest_ancestor(violating_edge.first, violating_edge.second);
        std::vector<size_t> cycle;
        if (forward_direction) {
          cycle = ts.T.get_path(violating_edge.first, ancestor);
          std::ranges::reverse(cycle);
          auto path = ts.T.get_path(violating_edge.second, ancestor);
          cycle.insert(cycle.end(), path.begin(), path.end());
        } else {
          cycle = ts.T.get_path(violating_edge.second, ancestor);
          std::ranges::reverse(cycle);
          auto path = ts.T.get_path(violating_edge.first, ancestor);
          cycle.insert(cycle.end(), path.begin(), path.end());
        }
        weight_type delta = std::numeric_limits<weight_type>::max();
        for (size_t i = 0; i + 1 < cycle.size(); i++) {
          auto it = flow.find({cycle[i], cycle[i + 1]});
          if (it != flow.end()) {
            delta =
                std::min(delta, upper_capacities.at(it->first) - it->second);
          } else {
            it = flow.find({cycle[i + 1], cycle[i]});
            assert(it != flow.end());
            delta =
                std::min(delta, it->second - lower_capacities.at(it->first));
          }
        }

        for (size_t i = 0; i + 1 < cycle.size(); i++) {
          auto it = flow.find({cycle[i], cycle[i + 1]});
          if (it != flow.end()) {
            it->second += delta;
          } else {
            it = flow.find({cycle[i + 1], cycle[i]});
            assert(it != flow.end());
            it->second -= delta;
          }
        }

        std::optional<indexed_edge> last_blocking_edge;

        // check from the end of cycle
        for (auto it = cycle.rbegin(); it + 1 < cycle.rend(); ++it) {
          auto u = *(it + 1);
          auto v = *it;
          auto it2 = flow.find({u, v});
          if (it2 != flow.end()) {
            if (it2->second == upper_capacities[it2->first]) {
              last_blocking_edge = it2->first;
              ts.U.push_back(last_blocking_edge.value());
              break;
            }
          } else {
            it2 = flow.find({v, u});
            assert(it2 != flow.end());
            if (it2->second == lower_capacities[it2->first]) {
              last_blocking_edge = it2->first;
              ts.L.push_back(last_blocking_edge.value());
              break;
            }
          }
        }
        assert(last_blocking_edge.has_value());
        assert(costs.contains(violating_edge));

        auto underlying_graph = ts.T.get_underlying_graph();
        underlying_graph.add_edge(graph.get_edge(violating_edge));
        underlying_graph.remove_edge(*last_blocking_edge);
        ts.T.clear_edges();
        underlying_graph.breadth_first_search(
            ts.T.get_root(),
            [&ts, this](size_t u, size_t v, weight_type weight) {
              ts.T.add_edge({graph.get_vertex(v), graph.get_vertex(u), weight});
              return false;
            });
        // ts.T.print_edges(std::cout);
      }
      if (std::ranges::any_of(flow, [&ts](const auto &p) {
            if (p.first.first == ts.T.get_root() ||
                p.first.second == ts.T.get_root()) {
              if (p.second != 0) {
                return true;
              }
            }
            return false;
          })) {
        return {};
      }
      std::erase_if(flow, [&ts](const auto &p) {
        return p.first.first == ts.T.get_root() ||
               p.first.second == ts.T.get_root();
      });
      return flow;
    }

    bool check_flow(const flow_fun_type &flow) const {
      decltype(demand) amount;

      graph.foreach_edge([this, &flow, &amount](auto const &e) {
        auto edge_flow = flow.at(e);
        amount[e.second] += edge_flow;
        amount[e.first] -= edge_flow;
      });
      return amount == demand;
    }

    bool check_feasible_flow(const flow_fun_type &flow) const {
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
      in_directed_tree<vertex_type, weight_type> T;
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

      T.breadth_first_search(root, [this](size_t u, size_t v, weight_type) {
        auto it = costs.find({u, v});
        if (it != costs.end()) {
          potential[v] = potential[u] + it->second;
        } else {
          it = costs.find({v, u});
          assert(it != costs.end());
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
        weight_type max_abs_cost = std::numeric_limits<weight_type>::min();
        for (auto &[_, cost] : costs) {
          auto abs_cost = cost;
          if (abs_cost < 0) {
            abs_cost = -abs_cost;
          }
          max_abs_cost = std::max(abs_cost, max_abs_cost);
        }
        weight_type C = static_cast<weight_type>(max_abs_cost * graph.get_vertex_number() + 1);

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
      // T.print_edges(std::cout);
      return {in_directed_tree<vertex_type, weight_type>(
                  std::move(T), artificial_vertex_name),
              std::move(L),
              {}};
    }

  private:
    directed_graph<vertex_type, weight_type> graph;
    flow_fun_type upper_capacities;
    flow_fun_type lower_capacities;
    flow_fun_type costs;
    flow_fun_type reduced_costs;
    std::unordered_map<size_t, weight_type> demand;
    std::unordered_map<size_t, weight_type> potential;
    std::optional<size_t> artificial_vertex_opt;
  };

} // namespace cyy::algorithm
