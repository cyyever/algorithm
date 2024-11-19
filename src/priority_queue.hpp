/*!
 * \file priority_queue.hpp
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
  class priority_queue
      : public mapped_heap_base<std::unordered_map<data_type, size_t>, key_type,
                                compare> {
  public:
    const key_type &top_key() const { return this->item_heap.top().data; }
    const data_type &top_data() const {
      return this->item_heap.top().iterator->first;
    }
    bool contains(const data_type &data) const {
      return this->container.contains(data);
    }
    void remove(const data_type &data) {
      auto it = this->container.find(data);
      if (it != this->container.end()) {
        this->item_heap.remove_item(it->second);
        this->container.erase(it);
      }
      this->check_consistency();
    }
    void change_key(const data_type &data, key_type key) {
      auto idx = this->container.at(data);
      this->item_heap.change_item(
          idx, [&key](auto &item) { item.data = std::move(key); });
      this->check_consistency();
    }

    bool insert(data_type data, key_type key) {
      this->check_consistency();
      auto const [it, has_insertion] =
          this->container.try_emplace(std::move(data), this->size());
      if (!has_insertion) {
        return false;
      }
      this->item_heap.insert({std::move(key), it, it->second});
      this->check_consistency();
      return true;
    }
  };
  template <typename data_type, typename key_type>
  using max_priority_queue = priority_queue<data_type, key_type, std::greater>;
} // namespace cyy::algorithm
