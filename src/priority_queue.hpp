/*!
 * \file heap.hpp
 *
 * \brief
 */

#pragma once
#include <cassert>
#include <unordered_map>

#include "heap.hpp"
namespace cyy::algorithm {

  template <typename data_type, typename key_type,
            template <typename T> class compare = std::less>
  class priority_queue {
  public:
    priority_queue() = default;
    size_t size() const noexcept { return item_heap.size(); }
    void reserve(size_t n) {
      position.reserve(n);
      item_heap.reserve(n);
    }
    const key_type &top_key() const { return item_heap.top().key; }
    const data_type &top_data() const { return item_heap.top().get_data(); }
    bool empty() const noexcept { return item_heap.empty(); }
    void pop() {
      if (this->empty()) {
        return;
      }
      auto it = item_heap.top().iterator;
      item_heap.pop();
      position.erase(it);
      check_consistency();
    }
    bool contains(const data_type &data) const {
      return position.contains(data);
    }
    void change_key(const data_type &data, key_type key) {
      auto idx = position.at(data);
      item_heap.change_item(idx,
                            [&key](auto &item) { item.key = std::move(key); });
      check_consistency();
    }

    bool insert(data_type data, key_type key) {
      check_consistency();
      auto const [it, has_insertion] = position.try_emplace(data, this->size());
      if (!has_insertion) {
        return false;
      }
      item_heap.insert(priority_queue_item{std::move(key), it});
      check_consistency();
      return true;
    }

  private:
    void check_consistency() {
#ifndef NDEBUG
      assert(position.size() == this->size());
      for (size_t i = 0; i < this->size(); i++) {
        auto const &item = item_heap.get_item(i);
        assert(item.heap_index == i);
        assert(position.at(item.get_data()) == i);
      }
#endif
    }

  private:
    std::unordered_map<data_type, size_t> position;
    using iterator_type = typename decltype(position)::iterator;
    struct priority_queue_item {
      key_type key;
      iterator_type iterator;
      size_t heap_index;
      priority_queue_item() = delete;
      priority_queue_item(const priority_queue_item &rhs) = delete;
      priority_queue_item &operator=(const priority_queue_item &rhs) = delete;
      auto operator<=>(const priority_queue_item &rhs) const noexcept {
        return key <=> rhs.key;
      }
      priority_queue_item(key_type key_, iterator_type iterator_) noexcept
          : key(std::move(key_)), iterator(iterator_),
            heap_index{iterator_->second} {}

      priority_queue_item(priority_queue_item &&rhs) noexcept {
        key = std::move(rhs.key);
        iterator = std::move(rhs.iterator);
        heap_index = iterator->second;
      }
      priority_queue_item& operator=(priority_queue_item &&rhs) noexcept {
        if (this ==&rhs) {
          return *this;
        }
        key = std::move(rhs.key);
        auto const old_heap_index = heap_index;
        iterator = std::move(rhs.iterator);
        iterator->second = old_heap_index;
        return *this;
      }
      const auto &get_data() const noexcept { return iterator->first; }
    };
    using heap_type = heap<priority_queue_item, compare>;
    heap_type item_heap;
  };
  template <typename data_type, typename key_type>
  using max_priority_queue = priority_queue<data_type, key_type, std::greater>;
} // namespace cyy::algorithm
