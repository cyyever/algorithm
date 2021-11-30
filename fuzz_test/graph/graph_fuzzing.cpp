/*!
 * \file graph_fuzzing.cpp
 *
 */
#include "../helper.hpp"
#include "graph/dag.hpp"
#include "graph/path.hpp"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size);
extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size) {
  try {
    std::span<const uint8_t> data_span(Data, Size);
    auto dg = generate_graph(data_span);
    auto g = dg.get_underlying_graph();
    g.is_tree();
    cyy::algorithm::DAG<uint8_t> dag(dg);
    dag.get_topological_ordering();
    if (g.has_vertex_index(0)) {
      shortest_path_Bellman_Ford(g, 0);
    }
  } catch (const std::invalid_argument &) {
  }
  return 0; // Non-zero return values are reserved for future use.
}
