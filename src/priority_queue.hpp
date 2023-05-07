/*!
 * \file heap.hpp
 *
 * \brief
 */

#pragma once
#include <cassert>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <unordered_map>

#include "heap.hpp"
namespace cyy::algorithm {

  template <typename key_type> using min_heap = heap<key_type>;

  template <typename key_type, typename iterator_type>
  struct priority_queue_item {
    key_type key;
    iterator_type iterator;
    size_t heap_index;
    priority_queue_item(const priority_queue_item &rhs) = delete;
    priority_queue_item &operator=(const priority_queue_item &rhs) = delete;

    auto operator<=>(const priority_queue_item &rhs) const {
      return key <=> rhs.key;
    }
    priority_queue_item(key_type key_, iterator_type iterator_)
        : key(std::move(key_)),
          iterator(iterator_), heap_index{iterator_->second} {}
    priority_queue_item(priority_queue_item &&rhs) {
      key = std::move(rhs.key);
      iterator = std::move(rhs.iterator);
      heap_index = iterator->second;
    }
    priority_queue_item &operator=(priority_queue_item &&rhs) {
      key = std::move(rhs.key);
      auto old_heap_index = heap_index;
      iterator = std::move(rhs.iterator);
      iterator->second = old_heap_index;
      return *this;
    }

    const auto &get_data() const { return iterator->first; }
  };

  template <
      typename data_type, typename key_type,
      class compare = std::less<priority_queue_item<
          key_type, typename std::unordered_map<data_type, size_t>::iterator>>>
  class priority_queue
      : public heap<
            priority_queue_item<key_type, typename std::unordered_map<
                                              data_type, size_t>::iterator>,
            compare> {
  public:
    priority_queue() = default;
    using heap_type =
        heap<priority_queue_item<key_type, typename std::unordered_map<
                                               data_type, size_t>::iterator>,
             compare>;
    void reserve(size_t n) {
      heap_type::reserve(n);
      position.reserve(n);
    }
    const key_type &top_key() const { return this->top().key; }
    const data_type &top_data() const { return this->top().get_data(); }
    void pop() {
      if (this->empty()) {
        return;
      }
      auto it = this->top().iterator;
      heap_type::pop();
      position.erase(it);
      check_consistency();
    }
    bool contains(const data_type &data) const {
      return position.contains(data);
    }
    void change_key(const data_type &data, key_type key) {
      auto idx = position.find(data)->second;
      this->items[idx].key = std::move(key);
      this->heapify(idx);
      check_consistency();
    }

    void insert(data_type data, key_type key) {
      check_consistency();
      auto [it, has_insertion] = position.try_emplace(data, SIZE_MAX);
      if (!has_insertion) {
        return;
      }
      it->second = this->size();
      heap_type::insert(priority_queue_item{std::move(key), it});
      check_consistency();
    }

  private:
    void check_consistency() {
      assert(position.size() == this->size());
      for (size_t i = 0; i < this->size(); i++) {
        assert(this->items[i].heap_index == i);
        assert(position.at(this->items[i].get_data()) == i);
      }
    }

  private:
    std::unordered_map<data_type, size_t> position;
  };
  template <typename data_type, typename key_type>
  using max_priority_queue = priority_queue<
      data_type, key_type,
      std::greater<priority_queue_item<
          key_type, typename std::unordered_map<data_type, size_t>::iterator>>>;
} // namespace cyy::algorithm
