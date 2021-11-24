/*!
 * \file helper.hpp
 *
 * \brief fuzzing helper functions
 * \author cyy
 * \date 2019-02-12
 */
#pragma once

#include "graph/graph.hpp"

cyy::algorithm::graph fuzzing_graph(const uint8_t *Data, size_t Size);
