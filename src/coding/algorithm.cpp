/*!
 * \file algorithm.cpp
 *
 * \brief
 */
#include "algorithm.hpp"

#include "heap.hpp"
#include "iostream"
namespace cyy::algorithm {
  std::unordered_map<symbol_type, std::string>
  huffman_code(const std::unordered_map<symbol_type, double> &symbol_frequency) {
    assert(!symbol_frequency.empty());
    heap<symbol_type, double> h;
    for (auto [symbol, frequency] : symbol_frequency) {
      h.insert(symbol, frequency);
    }

    auto huffman_code_fun = [&h](auto &&self) {
      std::unordered_map<symbol_type, std::string> code;
      if (h.size() == 1) {
        auto symbol = h.top();
        code[symbol] = "0";
        return code;
      }
      auto frequency1 = h.top_key();
      auto symbol1 = h.top();
      h.pop();
      auto frequency2 = h.top_key();
      auto symbol2 = h.top();
      if (h.size() == 1) {
        code[symbol1] = "0";
        code[symbol2] = "1";
        return code;
      }
      h.change_key(symbol2, frequency1 + frequency2);
      code = self(self);
      code[symbol1] = code[symbol2];
      code[symbol1].push_back('0');
      code[symbol2].push_back('1');
      return code;
    };
    return huffman_code_fun(huffman_code_fun);
  }
} // namespace cyy::algorithm
