/*!
 * \file string_algorithm.hpp
 *
 * \brief string algorithms
 * \author cyy
 * \date 2018-03-03
 */

#pragma once

import std;

namespace cyy::algorithm {

  template <typename T> class KMP {

  public:
    template <std::ranges::random_access_range U>
      requires std::same_as<std::ranges::range_value_t<U>, T>
    explicit KMP(const U &p, const std::optional<T> &any_char_opt_ = {})
        : pattern(p), any_char_opt(any_char_opt_) {
      // for each substring w1...ws,compute the longest proper prefix w1...wf(s)
      // that is a suffix of w1...ws
      auto pattern_size = std::ranges::size(pattern);
      failure_function.resize(pattern_size, 0);
      // f[1] is always empty string,so we begin with w2;
      for (std::size_t i = 2; i < pattern_size; i++) {
        auto const &w = pattern[i - 1];
        auto t = failure_function[i - 1];
        while (true) {
          if (pattern[t] == w || pattern[t] == any_char_opt ||
              w == any_char_opt) {
            failure_function[i] = t + 1;
            break;
          }
          if (t == 0) {
            break;
          }
          t = failure_function[t];
        }
      }
    }

    template <std::ranges::random_access_range U>
      requires std::same_as<std::ranges::range_value_t<U>, T>
    std::optional<std::size_t> search(const U &str) {
      if (failure_function.empty()) {
        return 0;
      }
      // for each substring w1...ws,compute the longest proper prefix w1...wf(s)
      // that is a suffix of w1...ws
      std::size_t s = 0;
      std::size_t i = 0;
      auto seq_size = std::ranges::size(str);
      while (i < seq_size) {
        auto const &c = str[i];
        if (pattern[s] == c || any_char_opt == pattern[s]) {
          s++;
          if (s == pattern.size()) {
            return i + 1 - s;
          }
        } else {
          if (s > 0) {
            s = failure_function[s];
            continue;
          }
        }
        i++;
      }
      return {};
    }

  private:
    std::span<const T> pattern;
    std::optional<T> any_char_opt;
    std::vector<std::size_t> failure_function;
  };

} // namespace cyy::algorithm
