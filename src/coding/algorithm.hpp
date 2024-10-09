/*!
 * \file algorithm.hpp
 *
 * \brief
 */

#include <string>
#include <unordered_map>

#include "alphabet/symbol.hpp"

namespace cyy::algorithm {
  std::unordered_map<symbol_type, std::string>
  huffman_code(const std::unordered_map<symbol_type, double> &symbol_frequency);
} // namespace cyy::algorithm
