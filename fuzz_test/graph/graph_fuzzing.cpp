/*!
 * \file regex_test.cpp
 *
 * \brief 测试正則
 */
#include "../helper.hpp"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size) {
  using namespace cyy::computation;
  try {
    auto g = fuzzing_graph(Data, Size);
    auto t = g.get_transpose();
  } catch (const std::invalid_argument &) {
  }
  return 0; // Non-zero return values are reserved for future use.
}
