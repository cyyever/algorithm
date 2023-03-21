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
  class simple_heap {
  public:
    simple_heap() = default;
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
    size_t change_data(size_t idx, data_type data) {
      if (idx >= items.size()) {
        throw std::range_error("out of range");
      }

      items[idx] = std::move(data);
      auto new_idx = heapify_up(idx);
      if (idx != new_idx) {
        return new_idx;
      }
      return heapify_down(idx);
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
      if (!compare{}(items[i], items[min_child_index])) {
        swap_items(i, min_child_index);
        return heapify_down(min_child_index);
      }
      return i;
    }

  private:
    void swap_items(size_t i, size_t j) { std::swap(items[i], items[j]); }

  protected:
    std::vector<data_type> items;
  };
  template <typename key_type>
  using simple_max_heap = simple_heap<key_type, std::greater<key_type>>;

  template <typename key_type, typename data_type> struct key_and_data {
    key_type key;
    data_type data;

    auto operator<=>(const key_and_data &rhs) const { return key <=> rhs.key; }
  };

  template <typename data_type, typename key_type,
            class compare = std::less<key_and_data<key_type, data_type>>>
  class heap : public simple_heap<key_and_data<key_type, data_type>, compare> {
  public:
    heap() = default;
    using parent_heap_type =
        simple_heap<key_and_data<key_type, data_type>, compare>;
    void reserve(size_t n) {
      simple_heap<data_type, compare>::reserve(n);
      position.reserve(n);
    }
    const key_type &top_key() const { return parent_heap_type::top().key; }
    const data_type &top_data() const { return parent_heap_type::top().data; }
    void pop() {
      if (parent_heap_type::empty()) {
        return;
      }
      position.erase(top_data());
      parent_heap_type::pop();
    }
    void change_key(const data_type &data, key_type key) {
      auto idx = position.at(data);
      this->items[idx].key = std::move(key);
      auto new_idx = this->heapify_up(idx);
      if (idx != new_idx) {
        return;
      }
      this->heapify_down(idx);
    }
    bool contains(const data_type &data) const {
      return position.contains(data);
    }
    template <typename = std::is_same<data_type, key_type>>
    void insert(data_type data) {
      insert(data, std::move(data));
    }

    void insert(data_type data, key_type key) {
      auto [it, has_insertion] = position.try_emplace(data, SIZE_MAX);
      if (!has_insertion) {
        return;
      }
      auto idx = parent_heap_type::insert(
          key_and_data{std::move(key), std::move(data)});
      it->second = idx;
    }

  private:
    std::unordered_map<data_type, size_t> position;
  };
  template <typename data_type, typename key_type = data_type>
  using max_heap = heap<data_type, key_type,
                        std::greater<key_and_data<key_type, data_type>>>;
} // namespace cyy::algorithm
