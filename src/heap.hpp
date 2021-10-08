/*!
 * \file heap.hpp
 *
 * \brief
 */

#pragma once
#include <cassert>
#include <unordered_map>
#include <vector>
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
  const data_type &top() const {
    assert(!empty());
    return items[0].data;
  }
  void pop() {
    if (items.empty()) {
      return;
    }
    position.erase(items[0].data);
    std::swap(items[0], items.back());
    items.pop_back();
    auto it = position.find(items[0].data);
    it->second = heapify_down(0);
    return;
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
      auto parent_idx = (i+1) / 2-1;
      if (compare{}(items[i].key, items[parent_idx].key)) {
        std::swap(items[i], items[parent_idx]);
        return heapify_up(parent_idx);
      }
    }
    return i;
  }
  size_t heapify_down(size_t i) {
    auto left_child_index = 2 * (i+1)-1;
    auto n = items.size();
    if (left_child_index >= n) {
      return i;
    }
    auto min_child_index = left_child_index;
    auto right_child_index = left_child_index + 1;
    if (right_child_index < n) {
      if (compare{}(items[right_child_index].key,
                    items[left_child_index].key)) {
        min_child_index = right_child_index;
      }
    }
    if (!compare{}(items[i].key, items[min_child_index].key)) {
      std::swap(items[i], items[min_child_index]);
      return heapify_down(min_child_index);
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
