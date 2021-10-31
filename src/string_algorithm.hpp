/*!
 * \file string_algorithm.hpp
 *
 * \brief string algorithms
 * \author cyy
 * \date 2018-03-03
 */

#pragma once

#include <map>
#include <optional>
#include <ranges>
#include <unordered_map>
#include <vector>

namespace cyy::algorithm {

  // find word from str
  template <std::ranges::input_range U, std::ranges::input_range V>
  requires std::same_as<std::ranges::range_value_t<U>,
                        std::ranges::range_value_t<V>>
      std::optional<size_t> KMP(U word, V str) {
    if (word.size() > str.size()) {
      return {};
    }
    if (word.empty()) {
      return 0;
    }

    // for each substring w1...ws,compute the longest proper prefix w1...wf(s)
    // that is a suffix of w1...ws
    std::vector<size_t> failure_function(word.size(), 0);
    // f[1] is always empty string,so we begin with w2;
    for (size_t i = 2; i < word.size(); i++) {
      auto next_char = word[i - 1];
      auto t = failure_function[i - 1];
      while (true) {
        if (word[t] == next_char) {
          failure_function[i] = t + 1;
          break;
        }
        if (t == 0) {
          break;
        }
        t = failure_function[t];
      }
    }

    size_t s = 0;
    for (size_t i = 0; i < str.size(); i++) {
      auto next_char = str[i];
      while (s > 0 && word[s] != next_char) {
        s = failure_function[s];
      }
      if (word[s] == next_char) {
        s++;
        if (s == word.size()) {
          return 0 + i - word.size() + 1;
        }
      }
    }
    return {};
  }

  // find one of words from str
  template <typename CharT>
  std::basic_string_view<CharT>
  Aho_Corasick(const std::vector<std::basic_string_view<CharT>> &words,
               std::basic_string_view<CharT> str) {
    // creat trie
    std::vector<std::map<CharT, size_t>> trie(1);
    std::unordered_map<size_t, size_t> final_states;

    {
      size_t next_state = 1;
      for (auto &word : words) {
        size_t cur_state = 0;
        for (auto const &c : word) {
          auto [it, has_emplaced] = trie[cur_state].try_emplace(c, next_state);
          cur_state = it->second;
          if (has_emplaced) {
            trie.emplace_back();
            next_state++;
          }
        }
        final_states.emplace(cur_state, word.size());
      }

      if (next_state == 1) {
        return {};
      }
    }

    // for each substring w1...ws,compute the longest proper suffix w1...wf(s)
    // that is a prefix of some word

    // so we compute this failure_function by broad first search of tree.
    // we find all immediate child of start_state first
    std::vector<size_t> failure_function(trie.size(), 0);

    std::multimap<size_t, size_t> search_frontier;
    for (size_t i = 1; i < trie.size(); i++) {
      search_frontier.emplace(0, i);
    }

    while (!search_frontier.empty()) {
      std::multimap<size_t, size_t> tmp;
      for (auto const &[prefix_state, suffix_state] : search_frontier) {
        auto const &prefix_frontier = trie[prefix_state];
        for (auto const &[next_char, next_state] : trie[suffix_state]) {
          auto it = prefix_frontier.find(next_char);
          if (it != prefix_frontier.end()) {
            tmp.emplace(it->second, next_state);
            failure_function[next_state] = it->second;
          }
        }
      }
      search_frontier = std::move(tmp);
    }

    size_t s = 0;
    size_t i = 0;

    while (s != 0 || i < str.size()) {
      if (i < str.size()) {
        auto next_char = str[i];

        auto it = trie[s].find(next_char);
        if (it != trie[s].end()) {
          s = it->second;
          i++;
        } else if (s == 0) {
          i++;
        } else {
          s = failure_function[s];
        }
      } else {
        s = failure_function[s];
      }

      // check state
      if (final_states.contains(s)) {
        auto word_size = final_states[s];
        if (i == str.size()) {
          i--;
        }
        return {str.data() + i - word_size + 1, word_size};
      }

      if (i > str.size() && s == 0) {
        break;
      }
    }
    return {};
  }

} // namespace cyy::algorithm
