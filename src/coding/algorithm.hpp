/*!
 * \file algorithm.hpp
 *
 * \brief
 */

#include <string>
#include <unordered_map>

#include "alphabet/alphabet.hpp"

namespace cyy::algorithm {
  std::unordered_map<symbol_type, std::string>
  huffman_code(const std::unordered_map<symbol_type, float> &symbol_frequency);
}
