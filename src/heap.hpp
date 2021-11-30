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

  template <typename data_type, class compare = std::less<data_type>>
  class simple_heap {
  public:
    simple_heap() = default;
    void reserve(size_t n) { items.reserve(n); }
    const data_type &top() const {
      assert(!empty());
      return items[0];
    }
    size_t size() const { return items.size(); }
    void pop() {
      if (items.empty()) {
        return;
      }
      if (items.size() == 1) {
        items.clear();
        return;
      }
      std::swap(items[0], items.back());
      items.pop_back();
      heapify_down(0);
    }
    void change_data(size_t idx, data_type data) {
      if (idx >= items.size()) {
        throw std::invalid_argument("not data");
      }

      items[idx] = std::move(data);
      auto new_idx = heapify_up(idx);
      if (idx != new_idx) {
        return;
      }
      heapify_down(idx);
    }
    bool empty() const { return items.empty(); }

    void insert(data_type data) {
      items.emplace_back(std::move(data));
      heapify_up(items.size() - 1);
    }

  private:
    size_t heapify_up(size_t i) {
      if (i > 0) {
        auto parent_idx = (i + 1) / 2 - 1;
        if (compare{}(items[i], items[parent_idx])) {
          std::swap(items[i], items[parent_idx]);
          return heapify_up(parent_idx);
        }
      }
      return i;
    }
    void heapify_down(size_t i) {
      auto left_child_index = 2 * (i + 1) - 1;
      auto n = items.size();
      if (left_child_index >= n) {
        return;
      }
      auto min_child_index = left_child_index;
      auto right_child_index = left_child_index + 1;
      if (right_child_index < n) {
        if (compare{}(items[right_child_index], items[left_child_index])) {
          min_child_index = right_child_index;
        }
      }
      if (!compare{}(items[i], items[min_child_index])) {
        std::swap(items[i], items[min_child_index]);
        heapify_down(min_child_index);
      }
    }

  private:
    std::vector<data_type> items;
  };
  template <typename data_type>
  using simple_max_heap = simple_heap<data_type, std::greater<data_type>>;

  template <typename data_type, typename key_type = data_type,
            class compare = std::less<key_type>>
  class heap {
  public:
    heap() = default;
    void reserve(size_t n) {
      items.reserve(n);
      position.reserve(n);
    }
    const key_type &top_key() const {
      assert(!empty());
      return items[0].key;
    }
    const data_type &top_data() const {
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
    }
    void change_key(const data_type &data, key_type key) {
      auto idx = position.at(data);
      assert(data == items[idx].data);
      items[idx].key = std::move(key);
      auto new_idx = heapify_up(idx);
      if (idx != new_idx) {
        return;
      }
      heapify_down(idx);
    }
    void change_data(const data_type &old_data, data_type data, key_type key) {
      auto it = position.find(old_data);
      if (it == position.end()) {
        throw std::invalid_argument("not data");
      }
      auto idx = it->second;
      position.erase(it);

      items[idx].data = data;
      items[idx].key = std::move(key);
      position.emplace(std::move(data), idx);
      auto new_idx = heapify_up(idx);
      if (idx != new_idx) {
        return;
      }
      heapify_down(idx);
    }
    template <typename = std::is_same<data_type, key_type>>
    void change_data(const data_type &old_data, data_type data) {
      change_data(old_data, data, data);
    }
    bool empty() const { return items.empty(); }
    bool contains(const data_type &data) const {
      return position.contains(data);
    }
    template <typename = std::is_same<data_type, key_type>>
    void insert(data_type data) {
      insert(data, data);
    }

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
    void heapify_down(size_t i) {
      auto left_child_index = 2 * (i + 1) - 1;
      auto n = items.size();
      if (left_child_index >= n) {
        return;
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
        std::swap(position[items[i].data],
                  position[items[min_child_index].data]);
        std::swap(items[i], items[min_child_index]);
        heapify_down(min_child_index);
      }
    }

  private:
    struct item {
      data_type data;
      key_type key;
    };
    std::vector<item> items;
    std::unordered_map<data_type, size_t> position;
  };
  template <typename data_type, typename key_type = data_type>
  using max_heap = heap<data_type, key_type, std::greater<key_type>>;
} // namespace cyy::algorithm
