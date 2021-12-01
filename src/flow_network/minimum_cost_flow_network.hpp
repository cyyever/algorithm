/*!
 * \file minimum_cost_flow_network.hpp
 *
 * \brief
 */

#pragma once

#include <algorithm>
#include <memory>
#include <numeric>
#include <optional>
#include <ranges>
#include <utility>
#include <vector>

#include "graph/tree.hpp"
#include "hash.hpp"
namespace cyy::algorithm {
  template <typename vertex_type = size_t, typename weight_type = double>
  class minimum_cost_flow_network {
  public:
    using edge_type = edge<vertex_type,weight_type>;
    using flow_fun_type = std::unordered_map<indexed_edge, weight_type>;
    using capacity_and_cost_fun_type =
        /* std::unordered_map<std::pair<vertex_type, vertex_type>, */
        std::unordered_map<edge_type,
                           std::tuple<weight_type, weight_type, weight_type>>;
    using demand_fun_type = std::unordered_map<vertex_type, weight_type>;
    minimum_cost_flow_network(
        const capacity_and_cost_fun_type &capacity_and_cost_map,
        const demand_fun_type &demand_) {
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
        in_directed_tree in_T(ts.T);
        auto ancestor =
            in_T.nearest_ancestor(violating_edge.first, violating_edge.second);
        std::vector<size_t> cycle;
        if (forward_direction) {
          cycle = in_T.get_path(violating_edge.first, ancestor);
          std::ranges::reverse(cycle);
          auto path = in_T.get_path(violating_edge.second, ancestor);
          cycle.insert(cycle.end(), path.begin(), path.end());
        } else {
          cycle = in_T.get_path(violating_edge.second, ancestor);
          std::ranges::reverse(cycle);
          auto path = in_T.get_path(violating_edge.first, ancestor);
          cycle.insert(cycle.end(), path.begin(), path.end());
        }
        std::optional<weight_type> delta_opt;
        for (size_t i = 0; i + 1 < cycle.size(); i++) {
          auto it = flow.find({cycle[i], cycle[i + 1]});
          weight_type delta{};
          if (it != flow.end()) {
            delta = upper_capacities.at(it->first) - it->second;
          } else {
            it = flow.find({cycle[i + 1], cycle[i]});
            assert(it != flow.end());
            delta = it->second - lower_capacities.at(it->first);
          }
          if (!delta_opt.has_value()) {
            delta_opt = delta;
          } else {
            delta_opt = std::min(*delta_opt, delta);
          }
        }

        for (size_t i = 0; i + 1 < cycle.size(); i++) {
          auto it = flow.find({cycle[i], cycle[i + 1]});
          if (it != flow.end()) {
            it->second += *delta_opt;
          } else {
            it = flow.find({cycle[i + 1], cycle[i]});
            assert(it != flow.end());
            it->second -= *delta_opt;
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

        ts.T.remove_edge(last_blocking_edge.value());
        ts.T.add_edge(graph.get_edge(violating_edge));
      }

      // check if G has a feasible flow
      if (std::ranges::any_of(flow, [&ts](const auto &p) {
            if (p.first.contains(ts.T.get_root())) {
              if (p.second != 0) {
                return true;
              }
            }
            return false;
          })) {
        return {};
      }

      assert(check_feasible_flow(flow));
      std::erase_if(flow, [this](const auto &p) {
        return p.first.contains(artificial_vertex_opt.value());
      });
      std::erase_if(costs, [this](const auto &p) {
        return p.first.contains(artificial_vertex_opt.value());
      });
      std::erase_if(lower_capacities, [this](const auto &p) {
        return p.first.contains(artificial_vertex_opt.value());
      });
      std::erase_if(upper_capacities, [this](const auto &p) {
        return p.first.contains(artificial_vertex_opt.value());
      });
      graph.remove_vertex(artificial_vertex_opt.value());
      demand.erase(artificial_vertex_opt.value());
      assert(is_tree_solution(flow));
      return flow;
    }

    bool check_flow(const flow_fun_type &flow) const {
      decltype(demand) amount;
      assert(flow.size() == graph.get_edge_number());

      for (auto const &e : graph.foreach_edge()) {
        auto edge_flow = flow.at(e);
        amount[e.second] += edge_flow;
        amount[e.first] -= edge_flow;
      }
      return amount == demand;
    }

    bool check_feasible_flow(const flow_fun_type &flow) const {
      if (!check_flow(flow)) {
        return false;
      }

      for (auto const &e : graph.foreach_edge()) {
        auto lower_capacity = lower_capacities.at(e);
        auto upper_capacity = upper_capacities.at(e);
        auto edge_flow = flow.at(e);
        if (edge_flow > upper_capacity || edge_flow < lower_capacity) {
          return false;
        }
      }
      return true;
    }

    bool is_tree_solution(const flow_fun_type &flow) const {
      assert(check_feasible_flow(flow));
      ::cyy::algorithm::graph<size_t, weight_type> free_edges;
      for (auto const &e : graph.foreach_edge()) {
        auto lower_capacity = lower_capacities.at(e);
        auto upper_capacity = upper_capacities.at(e);
        auto edge_flow = flow.at(e);
        if (edge_flow > lower_capacity && edge_flow < upper_capacity) {
          free_edges.add_edge({e.first, e.second});
        }
      }
      return free_edges.is_tree();
    }

    struct tree_structure {
      tree<vertex_type, weight_type> T;
      std::vector<indexed_edge> L;
      std::vector<indexed_edge> U;
    };

  private:
    minimum_cost_flow_network() = default;

    flow_fun_type determin_flow(const tree_structure &ts) {
      assert(ts.T.get_edge_number() + ts.L.size() + ts.U.size() ==
             costs.size());
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
          auto adjacent_vertex = ts.T.get_adjacent_vertex(leaf);
          if (costs.contains({leaf, adjacent_vertex})) {
            auto edge_flow = remain_demand[leaf] - demand[leaf];
            flow[{leaf, adjacent_vertex}] = edge_flow;
            remain_demand[adjacent_vertex] += edge_flow;
          } else {
            assert(costs.contains({adjacent_vertex, leaf}));
            assert(!flow.contains({adjacent_vertex, leaf}));
            auto edge_flow = demand[leaf] - remain_demand[leaf];
            flow[{adjacent_vertex, leaf}] = edge_flow;
            remain_demand[adjacent_vertex] -= edge_flow;
          }
          if (adjacent_vertex != ts.T.get_root()) {
            next_leaves.insert(adjacent_vertex);
          }
        }
        leaves = std::move(next_leaves);
      }
      assert(flow.size() == costs.size());
      assert(check_flow(flow));
      assert(check_feasible_flow(flow));
      return flow;
    }
    void determin_potential(const tree_structure &ts) {
      auto root = ts.T.get_root();
      potential[root] = 0;

      ts.T.breadth_first_search(root, [this](size_t u, size_t v, weight_type) {
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
#ifdef NDEBUG
        if (ts.T.has_edge(e)) {
          assert(reduced_costs[e] == 0);
        }
#endif
      }
    }

    tree_structure get_strongly_feasible_tree_structure() {
      if (!artificial_vertex_opt) {
        weight_type max_abs_cost = std::numeric_limits<weight_type>::min();
        for (auto &[_, cost] : costs) {
          auto abs_cost = cost;
          if (abs_cost < 0) {
            abs_cost = -abs_cost;
          }
          max_abs_cost = std::max(abs_cost, max_abs_cost);
        }
        weight_type C = static_cast<weight_type>(
            max_abs_cost * graph.get_vertex_number() + 1);

        auto B = demand;
        for (auto const &e : graph.foreach_edge()) {
          B[e.first] += lower_capacities[e];
          B[e.second] -= lower_capacities[e];
        }
        artificial_vertex_opt=graph.add_dummy_vertex();
        demand[*artificial_vertex_opt] = 0;

        for (auto [v, b] : B) {
          if (b > 0) {
            graph.add_edge(
                {graph.get_vertex(*artificial_vertex_opt), graph.get_vertex(v)});
            lower_capacities[{*artificial_vertex_opt, v}] = 0;
            upper_capacities[{*artificial_vertex_opt, v}] = b + 1;
            costs[{*artificial_vertex_opt, v}] = C;
          } else {
            graph.add_edge(
                {graph.get_vertex(v), graph.get_vertex(*artificial_vertex_opt)});
            lower_capacities[{v, *artificial_vertex_opt}] = 0;
            upper_capacities[{v, *artificial_vertex_opt}] = -b + 1;
            costs[{v, *artificial_vertex_opt}] = C;
          }
        }
      }
      std::vector<indexed_edge> L;
      tree<vertex_type, weight_type> T(graph.get_underlying_graph(), false);
      for (auto const &e : graph.foreach_edge()) {
        if (e.first != artificial_vertex_opt &&
            e.second != artificial_vertex_opt) {
          T.remove_edge(e);
          L.emplace_back(e);
        }
      }
      T.set_root_by_index(*artificial_vertex_opt);
      assert(T.get_edge_number() + L.size() == graph.get_edge_number());
      /* T.print_edges(std::cout); */
      return {std::move(T), std::move(L), {}};
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
