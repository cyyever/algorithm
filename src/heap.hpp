/*!
 * \file heap.hpp
 *
 * \brief
 */

#pragma once
#include <cassert>
#include <iostream>
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
  const key_type &top_key() const {
    assert(!empty());
    return items[0].key;
  }
  const data_type &top() const {
    assert(!empty());
    return items[0].data;
  }
  size_t size() const { return items.size(); }
  void pop() {
    if (items.empty()) {
      return;
    }
    if (items.size() == 1) {
      items.clear();
      position.clear();
      return;
    }
    assert(position[items[0].data] == 0);
    position.erase(items[0].data);
    position[items.back().data] = 0;
    std::swap(items[0], items.back());

    items.pop_back();
    heapify_down(0);
    return;
  }
  void change_key(const data_type &data, key_type key) {
    auto it = position.find(data);
    if (it == position.end()) {
      return;
    }
    auto idx = it->second;
    assert(data == items[idx].data);
    items[idx].key = std::move(key);
    auto new_idx = heapify_up(idx);
    if (idx != new_idx) {
      it->second = new_idx;
      return;
    }
    heapify_down(idx);
  }
  bool empty() const { return items.empty(); }
  bool contains(const data_type &data) const { return position.contains(data); }

  void insert(data_type data, key_type key) {
    auto [it, has_insertion] = position.try_emplace(data, SIZE_MAX);
    if (!has_insertion) {
      return;
    }
    items.emplace_back(std::move(data), std::move(key));
    it->second = items.size() - 1;
    heapify_up(items.size() - 1);
  }

private:
  size_t heapify_up(size_t i) {
    if (i > 0) {
      auto parent_idx = (i + 1) / 2 - 1;
      if (compare{}(items[i].key, items[parent_idx].key)) {
        std::swap(position[items[i].data], position[items[parent_idx].data]);
        std::swap(items[i], items[parent_idx]);
        return heapify_up(parent_idx);
      }
    }
    return i;
  }
  size_t heapify_down(size_t i) {
    auto left_child_index = 2 * (i + 1) - 1;
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
      std::swap(position[items[i].data], position[items[min_child_index].data]);
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
