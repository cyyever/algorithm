/*!
 * \file graph_fuzzing.cpp
 *
 */
#include "../helper.hpp"
#include "graph/dag.hpp"
#include "graph/path.hpp"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size) {
  try {
    auto dg = fuzzing_graph(Data, Size);
    auto g=dg.get_underlying_graph();
    g.is_tree();
    cyy::algorithm::DAG<uint8_t> dag(dg);
    dag.get_topological_ordering();
    shortest_path_Bellman_Ford(g, 0);
  } catch (const std::invalid_argument &) {
  }
  return 0; // Non-zero return values are reserved for future use.
}
