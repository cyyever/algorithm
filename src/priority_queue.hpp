/*!
 * \file heap.hpp
 *
 * \brief
 */

#pragma once
#include <cassert>
#include <cstddef>
#include <functional>
#include <unordered_map>

#include "heap.hpp"
namespace cyy::algorithm {

  template <typename data_type, typename key_type,
            template <typename T> class compare = std::less>
  class priority_queue {
  public:
    priority_queue() = default;
    [[nodiscard]] size_t size() const noexcept { return item_heap.size(); }
    void reserve(size_t n) {
      position.reserve(n);
      item_heap.reserve(n);
    }
    const key_type &top_key() const { return item_heap.top().data; }
    const data_type &top_data() const {
      return item_heap.top().iterator->first;
    }
    [[nodiscard]] bool empty() const noexcept { return item_heap.empty(); }
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
    void remove(const data_type &data) {
      auto it = position.find(data);
      if (it != position.end()) {
        item_heap.remove_item(it->second);
        position.erase(it);
      }
      check_consistency();
    }
    void change_key(const data_type &data, key_type key) {
      auto idx = position.at(data);
      item_heap.change_item(idx,
                            [&key](auto &item) { item.data = std::move(key); });
      check_consistency();
    }

    bool insert(data_type data, key_type key) {
      check_consistency();
      auto const [it, has_insertion] =
          position.try_emplace(std::move(data), this->size());
      if (!has_insertion) {
        return false;
      }
      item_heap.insert(item_type{std::move(key), it, it->second});
      check_consistency();
      return true;
    }

  private:
    void check_consistency() {
#ifndef NDEBUG
      assert(position.size() == this->size());
      for (size_t i = 0; i < this->size(); i++) {
        auto const &item = item_heap.get_item(i);
        assert(item.iterator->second == i);
      }
#endif
    }

    std::unordered_map<data_type, size_t> position;
    using iterator_type = typename decltype(position)::iterator;
    using item_type = pair_referred_item<key_type, iterator_type>;
    using heap_type = heap<item_type, compare>;
    heap_type item_heap;
  };
  template <typename data_type, typename key_type>
  using max_priority_queue = priority_queue<data_type, key_type, std::greater>;
} // namespace cyy::algorithm
