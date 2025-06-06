/*!
 * \file trie.hpp
 *
 * \brief trie data structure
 * \author cyy
 * \date 2024-03-03
 */

#pragma once

#include "pool.hpp"

namespace cyy::algorithm {
  template <typename T> class trie {
  private:
    object_pool<T, false> pool;

  public:
    using element_id_type = decltype(pool)::element_id_type;

    struct trie_node {
      element_id_type element_id{0};
      element_id_type parent_id{0};
      bool is_end{false};
      bool operator==(const trie_node &rhs) const noexcept {
        return element_id == rhs.element_id && parent_id == rhs.parent_id;
      }
    };

    // Custom hash can be a standalone function object.
    struct trie_node_hash {
      auto operator()(const trie_node &s) const noexcept {
        return std::hash<element_id_type>{}(s.element_id);
      }
    };

    template <std::ranges::range Sequences>
      requires std::ranges::range<std::ranges::range_value_t<Sequences>> &&
               std::same_as<std::ranges::range_value_t<
                                std::ranges::range_value_t<Sequences>>,
                            T>
    explicit trie(const Sequences& sequences) {
      size_t level_num = 0;
      for (auto const &r : sequences) {
        level_num = std::max(level_num, std::ranges::size(r));
      }
      if (level_num == 0) {
        return;
      }
      levels.reserve(level_num);
      for (auto const &r : sequences) {
        add_sequence(r);
      }
    }

    template <std::ranges::range Sequence>
      requires std::same_as<std::ranges::range_value_t<Sequence>, T>
    void add_sequence(const Sequence& sequence) {
      element_id_type parent_id = 0;
      auto sequence_size = std::ranges::size(sequence);
      decltype(sequence_size) level_idx = 0;
      for (auto const &e : sequence) {
        if (levels.size() < level_idx + 1) {
          levels.emplace_back();
        }
        auto node_id = pool.add_data(e);
        auto &level = levels[level_idx];
        auto it2 = level.emplace(trie_node{node_id, parent_id, false}).first;
        if (level_idx + 1 == sequence_size) {
          const_cast<trie_node &>(*it2).is_end = true;
        }
        parent_id = node_id;
        level_idx++;
      }
    }
    element_id_type get_data_id(const T &elem) {
      return pool.get_data_id(elem);
    }
    auto get_level_view(size_t level_idx) const {
      return std::views::all(levels.at(level_idx));
    }

  private:
    std::vector<std::unordered_set<trie_node, trie_node_hash>> levels;
  };

} // namespace cyy::algorithm
