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

  private:
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
  template <typename data_type>
  using simple_max_heap = simple_heap<data_type, std::greater<data_type>>;

  template <typename data_type, typename key_type = data_type,
            class compare = std::less<key_type>>
  class heap : public simple_heap<key_type, compare> {
  public:
    heap() = default;
    void reserve(size_t n) {
      simple_heap<data_type, compare>::reserve(n);
      data_vec.reserve(n);
      position.reserve(n);
    }
    const key_type &top_key() const {
      return simple_heap<key_type, compare>::top();
    }
    const data_type &top_data() const { return data_vec.at(0).data; }
    void pop() {
      if (data_vec.empty()) {
        return;
      }
      position.erase(data_vec[0]);
      data_vec[0] = std::move(data_vec.back());
      data_vec.pop_back();
      simple_heap<key_type, compare>::pop();
    }
    void change_key(const data_type &data, key_type key) {
      auto idx = position.at(data);
      assert(data == data_vec[idx]);
      this->items[idx].key = std::move(key);
      auto new_idx = heapify_up(idx);
      if (idx != new_idx) {
        return;
      }
      heapify_down(idx);
    }
    /* void change_data(const data_type &old_data, data_type data, key_type key)
     * { */
    /*   auto it = position.at(old_data); */
    /*   if (it == position.end()) { */
    /*     throw std::invalid_argument("not data"); */
    /*   } */
    /*   auto idx = it->second; */
    /*   position.erase(it); */

    /*   items[idx].data = data; */
    /*   items[idx].key = std::move(key); */
    /*   position.emplace(std::move(data), idx); */
    /*   auto new_idx = heapify_up(idx); */
    /*   if (idx != new_idx) { */
    /*     return; */
    /*   } */
    /*   heapify_down(idx); */
    /* } */
    /* template <typename = std::is_same<data_type, key_type>> */
    /* void change_data(const data_type &old_data, data_type data) { */
    /*   change_data(old_data, data, data); */
    /* } */
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
      data_vec.emplace_back(std::move(data));
      it->second = data_vec.size() - 1;
      simple_heap<key_type>::insert(std::move(key));
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
    std::vector<data_type> data_vec;
    std::unordered_map<data_type, size_t> position;
  };
  template <typename data_type, typename key_type = data_type>
  using max_heap = heap<data_type, key_type, std::greater<key_type>>;
} // namespace cyy::algorithm
