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
#include "graph/graph.hpp"

cyy::algorithm::directed_graph<uint8_t>
generate_graph(std::span<uint8_t> &data);

cyy::algorithm::flow_network<uint8_t>
generate_flow_network(std::span<uint8_t> &data);
