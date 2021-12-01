/*!
 * \file graph_fuzzing.cpp
 *
 */
#include "../helper.hpp"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size);
extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size) {
  try {
    std::span<const uint8_t> data_span(Data, Size);
    auto f = generate_minimum_cost_flow_network<int>(data_span);
    f.min_cost_flow_by_network_simplex();
  } catch (const std::exception &) {
  }
  return 0; // Non-zero return values are reserved for future use.
}
