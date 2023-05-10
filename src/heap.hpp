/*!
 * \file heap.hpp
 *
 * \brief
 */

#pragma once
#include <cassert>
#include <functional>
#include <vector>
namespace cyy::algorithm {

  template <typename data_type, template <typename T> class compare = std::less>
  class heap {
  public:
    heap() = default;
    ~heap() = default;
    void reserve(size_t n) { items.reserve(n); }
    const data_type &top() const { return get_item(0); }
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
    void change_item(size_t index, std::function<void(data_type &)> cb) {
      assert(index < this->size());
      cb(items[index]);
      heapify(index);
    }
    const data_type &get_item(size_t index) const { return items.at(index); }

  private:
    void heapify(size_t i) {
      if (heapify_up(i) != i) {
        return;
      }
      heapify_down(i);
    }

    size_t heapify_up(size_t i) {
      if (i == 0) {
        return i;
      }
      auto tmp = std::move(items[i]);
      while (i > 0) {
        auto parent_idx = (i + 1) / 2 - 1;
        if (comparator(tmp, items[parent_idx])) {
          items[i] = std::move(items[parent_idx]);
          i = parent_idx;
        } else {
          break;
        }
      }
      items[i] = std::move(tmp);
      return i;
    }
    void heapify_down(size_t i) {
      auto left_child_index = 2 * (i + 1) - 1;
      if (left_child_index >= size()) {
        return;
      }
      auto tmp = std::move(items[i]);
      do {
        auto min_child_index = left_child_index;
        auto right_child_index = left_child_index + 1;
        if (right_child_index < size()) {
          if (comparator(items[right_child_index], items[left_child_index])) {
            min_child_index = right_child_index;
          }
        }
        if (comparator(items[min_child_index], tmp)) {
          items[i] = std::move(items[min_child_index]);
          i = min_child_index;
        } else {
          break;
        }
        left_child_index = 2 * (i + 1) - 1;
      } while (left_child_index < size());
      items[i] = std::move(tmp);
      return;
    }

    compare<data_type> comparator;
    std::vector<data_type> items;
  };
  template <typename key_type> using max_heap = heap<key_type, std::greater>;

  template <typename key_type> using min_heap = heap<key_type>;

} // namespace cyy::algorithm
