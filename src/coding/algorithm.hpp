/*!
 * \file algorithm.hpp
 *
 * \brief
 */

#include "alphabet/alphabet.hpp"
#include <string>
#include <unordered_map>

namespace cyy::algorithm {
std::unordered_map<symbol_type, std::string>
huffman_code(const std::unordered_map<symbol_type, float> &symbol_frequency);
}
