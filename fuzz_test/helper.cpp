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
    g.add_edge({Data[0], Data[1]});
    data = data.subspan(2);
  }
  return g;
}

cyy::algorithm::flow_network<uint8_t>
generate_flow_network(std::span<uint8_t> &data) {
  auto g = generate_graph(data);
  size_t source = SIZE_MAX;
  size_t sink = SIZE_MAX;
  for (auto idx : g.get_vertex_indices()) {
    if (source == SIZE_MAX) {
      source = idx;
    } else if (sink == SIZE_MAX) {
      sink = idx;
    }
  }
  cyy::algorithm::flow_network<uint8_t>::capacity_fun_type capacities;
  for (auto e : g.foreach_edge()) {
    capacities.emplace_back(g.get_edge(e));
  }
  return cyy::algorithm::flow_network<uint8_t>(capacities, source, sink);
}
