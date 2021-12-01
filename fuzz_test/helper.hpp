/*!
 * \file helper.hpp
 *
 * \brief fuzzing helper functions
 * \author cyy
 * \date 2019-02-12
 */
#pragma once

#include <span>

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
