/*!
 * \file heap.hpp
 *
 * \brief
 */

#pragma once
#include <cassert>
#include <functional>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <vector>
namespace cyy::algorithm {

  template <typename data_type, class compare = std::less<data_type>>
  class heap {
  public:
    heap() = default;
    ~heap() = default;
    void reserve(size_t n) { items.reserve(n); }
    const data_type &top() const { return items.at(0); }
    size_t size() const { return items.size(); }
    void pop() {
      if (items.empty()) {
        return;
      }
      items[0] = std::move(items.back());
      items.pop_back();
      heapify_down(0);
      return;
    }
    bool empty() const { return items.empty(); }

    size_t insert(data_type data) {
      items.emplace_back(std::move(data));
      return heapify_up(items.size() - 1);
    }

  protected:
    size_t heapify_up(size_t i) {
      if (i > 0) {
        auto parent_idx = (i + 1) / 2 - 1;
        if (compare{}(items[i], items[parent_idx])) {
          swap_items(i, parent_idx);
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
        if (compare{}(items[right_child_index], items[left_child_index])) {
          min_child_index = right_child_index;
        }
      }
      if (compare{}(items[min_child_index], items[i])) {
        swap_items(i, min_child_index);
        return heapify_down(min_child_index);
      }
      return i;
    }

  protected:
    void swap_items(size_t i, size_t j) {
      std::swap(items[i], items[j]);
    }

  protected:
    std::vector<data_type> items;
  };
  template <typename key_type>
  using max_heap = heap<key_type, std::greater<key_type>>;

  template <typename key_type> using min_heap = heap<key_type>;

} // namespace cyy::algorithm
