/*!
 * \file helper.hpp
 *
 * \brief fuzzing helper functions
 * \author cyy
 * \date 2019-02-12
 */
#pragma once

#include <span>

#include "flow_network/flow_network.hpp"
#include "flow_network/minimum_cost_flow_network.hpp"
#include "graph/graph.hpp"

template <typename weight_type = double>
cyy::algorithm::directed_graph<uint8_t, weight_type>
generate_graph(std::span<const uint8_t> &data) {
  cyy::algorithm::directed_graph<uint8_t, weight_type> g;
  while (data.size() >= 3) {
    g.add_edge({data[0], data[1], static_cast<weight_type>(data[2])});
    data = data.subspan(3);
  }
  return g;
}

template <typename weight_type = double>
cyy::algorithm::flow_network<uint8_t, weight_type>
generate_flow_network(std::span<const uint8_t> &data) {
  auto g = generate_graph<weight_type>(data);
  uint8_t source = UINT8_MAX;
  uint8_t sink = UINT8_MAX;
  for (auto v : g.get_vertices()) {
    if (source == UINT8_MAX) {
      source = v;
    } else if (sink == UINT8_MAX) {
      sink = v;
    }
  }
  typename cyy::algorithm::flow_network<uint8_t, weight_type>::capacity_fun_type
      capacities;
  for (auto e : g.foreach_edge()) {
    capacities.emplace_back(g.get_edge(e));
  }
  return cyy::algorithm::flow_network<uint8_t, weight_type>(capacities, source,
                                                            sink);
}

template <typename weight_type = double>
cyy::algorithm::minimum_cost_flow_network<uint8_t, weight_type>
generate_minimum_cost_flow_network(std::span<const uint8_t> &data) {
  auto g = generate_graph<weight_type>(data);
  typename cyy::algorithm::minimum_cost_flow_network<
      uint8_t, weight_type>::capacity_and_cost_fun_type capacity_and_cost;
  typename cyy::algorithm::minimum_cost_flow_network<
      uint8_t, weight_type>::demand_fun_type demand;
  for (auto e : g.foreach_edge()) {
    if (data.size() >= 3) {
      capacity_and_cost.emplace(g.get_edge(e), {data[0], data[1], data[2]});
      data = data.subspan(3);
    } else {
      break;
    }
  }
  for (auto v : g.get_vertices()) {
    if (!data.empty()) {
      demand.emplace(v, data[0]);
      data = data.subspan(1);
    } else {
      break;
    }
  }

  return cyy::algorithm::minimum_cost_flow_network<uint8_t, weight_type>(
      capacity_and_cost, demand);
}
