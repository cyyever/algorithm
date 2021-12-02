/*!
 * \file minimum_cost_flow_network_fuzzing.cpp
 *
 */
#include "helper.hpp"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size);
extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size) {
  try {
    std::span<const uint8_t> data_span(Data, Size);
    auto f =generate_linear_program<double>(data_span);
    f.solve_by_primal_simplex();
  } catch (const std::exception &) {
  }
  return 0; // Non-zero return values are reserved for future use.
}
