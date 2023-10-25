/*!
 * \file algorithm.cpp
 *
 * \brief
 */
#include "algorithm.hpp"

#include "priority_queue.hpp"
namespace cyy::algorithm {
  std::unordered_map<symbol_type, std::string> huffman_code(
      const std::unordered_map<symbol_type, double> &symbol_frequency) {
    assert(!symbol_frequency.empty());
    priority_queue<symbol_type, double> h;
    for (auto const [symbol, frequency] : symbol_frequency) {
      h.insert(symbol, frequency);
    }

    std::unordered_map<symbol_type, std::string> code;
    auto huffman_code_fun = [&h, &code](auto &&self) {
      if (h.size() == 1) {
        auto symbol = h.top_data();
        code[symbol] = "0";
        return;
      }
      auto frequency1 = h.top_key();
      auto symbol1 = h.top_data();
      h.pop();
      auto frequency2 = h.top_key();
      auto symbol2 = h.top_data();
      if (h.size() == 1) {
        code[symbol1] = "0";
        code[symbol2] = "1";
        return;
      }
      h.change_key(symbol2, frequency1 + frequency2);
      self(self);
      code[symbol1] = code[symbol2];
      code[symbol1].push_back('0');
      code[symbol2].push_back('1');
      return;
    };
    huffman_code_fun(huffman_code_fun);
    return code;
  }
} // namespace cyy::algorithm
