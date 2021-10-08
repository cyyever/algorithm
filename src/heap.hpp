/*!
 * \file heap.hpp
 *
 * \brief
 */

#pragma once
#include <unordered_map>
#include <vector>
#include <cassert>
namespace cyy::algorithm {
template <typename data_type, typename key_type,
          class compare = std::less<key_type>>
class heap {
public:
  heap() = default;
  void reserve(size_t n) {
    items.reserve(n);
    items.reserve(n);
  }
  const data_type &top() const { assert(!empty()); return items[0].data; }
  void pop() {
    auto data = std::move(items[0].data);
    return data;
  }
  bool empty() const { return items.empty(); }

  void insert(data_type data, key_type key) {
    auto [it, has_insertion] = position.try_emplace(data, SIZE_MAX);
    if (!has_insertion) {
      return;
    }
    items.emplace_back(std::move(data), std::move(key));
    it->second = heapify_up(items.size() - 1);
  }

private:
  size_t heapify_up(size_t i) {
    if (i > 0) {
      auto parent_idx = i / 2;
      if (compare{}(items[i].key, items[parent_idx].key)) {
        std::swap(items[i], items[parent_idx]);
        return heapify_up(parent_idx);
      }
    }
    return i;
  }

private:
  struct item {
    data_type data;
    key_type key;
  };
  std::vector<item> items;
  std::unordered_map<data_type, size_t> position;
};
} // namespace cyy::algorithm
