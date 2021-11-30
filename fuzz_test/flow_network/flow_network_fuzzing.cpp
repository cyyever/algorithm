/*!
 * \file graph_fuzzing.cpp
 *
 */
#include "../helper.hpp"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size) {
  try {
    std::span<const uint8_t> data_span(Data,Size);
    auto f = generate_flow_network(data_span);
    f.max_flow();
  } catch (const std::exception &) {
  }
  return 0; // Non-zero return values are reserved for future use.
}
