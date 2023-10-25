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
    priority_queue<symbol_type, double> queue;
    for (auto const& [symbol, frequency] : symbol_frequency) {
      queue.insert(symbol, frequency);
    }

    std::unordered_map<symbol_type, std::string> code;
    auto huffman_code_fun = [&queue, &code](auto &&self) {
      if (queue.size() == 1) {
        auto symbol = queue.top_data();
        code[symbol] = "0";
        return;
      }
      auto frequency1 = queue.top_key();
      auto symbol1 = queue.top_data();
      queue.pop();
      auto frequency2 = queue.top_key();
      auto symbol2 = queue.top_data();
      if (queue.size() == 1) {
        code[symbol1] = "0";
        code[symbol2] = "1";
        return;
      }
      queue.change_key(symbol2, frequency1 + frequency2);
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
