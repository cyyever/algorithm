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
    void reserve(size_t n) { items.reserve(n); }
    const data_type &top() const { return get_item(0); }
    [[nodiscard]] size_t size() const noexcept { return items.size(); }
    void pop() noexcept {
      if (items.empty()) {
        return;
      }
      items[0] = std::move(items.back());
      items.pop_back();
      heapify_down(0);
    }
    [[nodiscard]] bool empty() const noexcept { return items.empty(); }

    size_t insert(data_type data) {
      items.emplace_back(std::move(data));
      return heapify_up(items.size() - 1);
    }
    void change_item(size_t index, std::function<void(data_type &)> callback) {
      assert(index < this->size());
      callback(items[index]);
      heapify(index);
    }
    const data_type &get_item(size_t index) const { return items.at(index); }

  private:
    void heapify(size_t index) noexcept {
      if (heapify_up(index) != index) {
        return;
      }
      heapify_down(index);
    }

    size_t heapify_up(size_t index) noexcept {
      if (index == 0) {
        return index;
      }
      auto tmp = std::move(items[index]);
      while (index > 0) {
        const auto parent_idx = (index + 1) / 2 - 1;
        if (comparator(tmp, items[parent_idx])) {
          items[index] = std::move(items[parent_idx]);
          index = parent_idx;
        } else {
          break;
        }
      }
      items[index] = std::move(tmp);
      return index;
    }
    void heapify_down(size_t index) noexcept {
      auto left_child_index = 2 * (index + 1) - 1;
      if (left_child_index >= size()) {
        return;
      }
      auto tmp = std::move(items[index]);
      while (left_child_index < size()) {
        auto min_child_index = left_child_index;
        auto const right_child_index = left_child_index + 1;
        if (right_child_index < size()) {
          if (comparator(items[right_child_index], items[left_child_index])) {
            min_child_index = right_child_index;
          }
        }
        if (comparator(items[min_child_index], tmp)) {
          items[index] = std::move(items[min_child_index]);
          index = min_child_index;
        } else {
          break;
        }
        left_child_index = 2 * (index + 1) - 1;
      }
      items[index] = std::move(tmp);
    }

    compare<data_type> comparator;
    std::vector<data_type> items;
  };
  template <typename key_type> using max_heap = heap<key_type, std::greater>;

  template <typename key_type> using min_heap = heap<key_type>;

} // namespace cyy::algorithm
