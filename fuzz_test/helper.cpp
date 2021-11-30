/*!
 * \file helper.cpp
 *
 * \brief fuzzing helper functions
 * \author cyy
 * \date 2019-02-12
 */
#include "helper.hpp"

cyy::algorithm::directed_graph<uint8_t>
generate_graph(std::span<uint8_t> &data) {
  cyy::algorithm::directed_graph<uint8_t> g;
  while (data.size() >= 2) {
    g.add_edge({data[0], data[1]});
    data = data.subspan(2);
  }
  return g;
}

cyy::algorithm::flow_network<uint8_t>
generate_flow_network(std::span<uint8_t> &data) {
  auto g = generate_graph(data);
  uint8_t source = UINT8_MAX;
  uint8_t sink = UINT8_MAX;
  for (auto idx : g.get_vertex_indices()) {
    if (source == UINT8_MAX) {
      source = idx;
    } else if (sink == UINT8_MAX) {
      sink = idx;
    }
  }
  cyy::algorithm::flow_network<uint8_t>::capacity_fun_type capacities;
  for (auto e : g.foreach_edge()) {
    capacities.emplace_back(g.get_edge(e));
  }
  return cyy::algorithm::flow_network<uint8_t>(capacities, source, sink);
}
