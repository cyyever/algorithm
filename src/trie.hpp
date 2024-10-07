/*!
 * \file string_algorithm.hpp
 *
 * \brief string algorithms
 * \author cyy
 * \date 2018-03-03
 */

#pragma once

#include <concepts>
#include <ranges>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace cyy::algorithm {
  template <typename T> class trie {
  public:
    using element_id_type = size_t;

    struct trie_node {
      element_id_type element_id{0};
      element_id_type parent_id{0};
      bool is_end{false};
      bool operator==(const trie_node &rhs) const noexcept {
        return element_id == rhs.element_id;
      }
    };

    // Custom hash can be a standalone function object.
    struct trie_node_hash {
      auto operator()(const trie_node &s) const noexcept {
        return std::hash<element_id_type>{}(s.element_id);
      }
    };

    template <std::ranges::input_range Sequences>
      requires std::ranges::input_range<
                   std::ranges::range_value_t<Sequences>> &&
               std::same_as<std::ranges::range_value_t<
                                std::ranges::range_value_t<Sequences>>,
                            T>
    trie(Sequences sequences) {
      size_t level_num = 0;
      for (auto const &r : sequences) {
        level_num = std::max(level_num, std::ranges::size(r));
      }
      if (level_num == 0) {
        return;
      }
      levels.reserve(level_num);
      for (auto const &r : sequences) {
        insert(r);
      }
    }

    template <std::ranges::input_range Sequence>
      requires std::same_as<std::ranges::range_value_t<Sequence>, T>
    void insert(Sequence sequence) {
      element_id_type parent_id = 0;
      auto sequence_size = std::ranges::size(sequence);
      decltype(sequence_size) level_idx = 0;
      for (auto const &e : sequence) {
        if (levels.size() < level_idx + 1) {
          levels.emplace_back();
        }
        auto node_id = add_data(e);
        auto &level = levels[level_idx];
        auto it2 = level.emplace(trie_node{node_id, parent_id, false}).first;
        if (level_idx + 1 == sequence_size) {
          const_cast<trie_node &>(*it2).is_end = true;
        }
        level_idx++;
      }
    }
    element_id_type get_data_id(const T &elem) { return node_pool[elem]; }
    element_id_type add_data(const T &e) {
      auto [it, emplaced] = node_pool.emplace(e, next_node_id);
      if (emplaced) {
        next_node_id++;
      }
      return it->second;
    }
    auto get_level_view(size_t level_idx) const {
      return std::views::all(levels.at(level_idx));
    }

  private:
    std::vector<std::unordered_set<trie_node, trie_node_hash>> levels;
    std::unordered_map<T, element_id_type> node_pool;
    std::vector<T> node_pool_elements;
    element_id_type next_node_id{0};
  };

} // namespace cyy::algorithm
