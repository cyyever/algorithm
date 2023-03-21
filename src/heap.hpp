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
    virtual ~simple_heap() = default;
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
    virtual void swap_items(size_t i, size_t j) {
      std::swap(items[i], items[j]);
    }

  protected:
    std::vector<data_type> items;
  };
  template <typename key_type>
  using simple_max_heap = simple_heap<key_type, std::greater<key_type>>;

  template <typename key_type, typename data_type> struct key_and_data {
    key_type key;
    data_type data;

    auto operator<(const key_and_data &rhs) const { return key < rhs.key; }
    auto operator>(const key_and_data &rhs) const { return key > rhs.key; }
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
      parent_heap_type::reserve(n);
      position.reserve(n);
    }
    const key_type &top_key() const { return parent_heap_type::top().key; }
    const data_type &top_data() const { return parent_heap_type::top().data; }
    void pop() {
      check_consistency();
      if (parent_heap_type::empty()) {
        return;
      }
      position.erase(top_data());
      if (parent_heap_type::size() > 1) {
        position.at(this->items.back().data) = 0;
      }
      parent_heap_type::pop();
      check_consistency();
    }
    bool contains(const data_type &data) const {
      return position.contains(data);
    }
    void change_key(const data_type &data, key_type key) {
      check_consistency();
      auto it = position.find(data);
      auto idx = it->second;
      assert(this->items[idx].data == data);
      this->items[idx].key = std::move(key);
      auto new_idx = this->heapify_up(idx);
      if (idx != new_idx) {
        it->second = new_idx;
        check_consistency();
        return;
      }
      new_idx = this->heapify_down(idx);
      it->second = new_idx;
      check_consistency();
    }
    size_t change_data(const data_type &old_data, data_type data) {
      auto idx = position.at(old_data);
      position.erase(old_data);
      position[data] = idx;
      this->items[idx].data = std::move(data);
      check_consistency();
      return idx;
    }
    void swap_items(size_t i, size_t j) override {
      printf("swap items %d %d\n", (int)i, (int)j);
      assert(position.at(this->items[i].data) == i);
      assert(position.at(this->items[j].data) == j);
      position[this->items[j].data] = i;
      position[this->items[i].data] = j;
      parent_heap_type::swap_items(i, j);
    }

    void insert(data_type data, key_type key) {
      check_consistency();
      auto [it, has_insertion] = position.try_emplace(data, SIZE_MAX);
      if (!has_insertion) {
        return;
      }
      it->second = this->items.size();

      auto idx = parent_heap_type::insert(
          key_and_data{std::move(key), std::move(data)});
      printf("new idx is %d\n", (int)idx);
      check_consistency();
    }

  private:
    void check_consistency() {
      assert(position.size() == this->items.size());
      for (size_t i = 0; i < this->items.size(); i++) {
        assert(position.at(this->items[i].data) == i);
      }
    }

  private:
    std::unordered_map<data_type, size_t> position;
  };
  template <typename data_type, typename key_type = data_type>
  using max_heap = heap<data_type, key_type,
                        std::greater<key_and_data<key_type, data_type>>>;
} // namespace cyy::algorithm
