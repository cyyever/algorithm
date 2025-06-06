/*!
 * \file heap.hpp
 *
 * \brief
 */

#pragma once
#include <cassert>
import std;
namespace cyy::algorithm {

  template <typename data_type, template <typename T> class compare = std::less>
  class heap {
  public:
    using heap_data_type = data_type;
    void reserve(std::size_t n) { items.reserve(n); }
    const data_type &top() const { return get_item(0); }
    [[nodiscard]] std::size_t size() const noexcept { return items.size(); }
    void pop() noexcept {
      if (items.empty()) {
        return;
      }
      items[0] = std::move(items.back());
      items.pop_back();
      heapify_down(0);
    }
    [[nodiscard]] bool empty() const noexcept { return items.empty(); }

    std::size_t insert(data_type data) {
      items.emplace_back(std::move(data));
      return heapify_up(items.size() - 1);
    }
    std::size_t change_item(std::size_t index,
                            const std::function<void(data_type &)> &callback) {
      assert(index < this->size());
      callback(items[index]);
      return heapify(index);
    }
    void remove_item(std::size_t index) {
      assert(index < this->size());
      if (this->empty()) {
        return;
      }
      std::swap(items[index], items.back());
      items.pop_back();
      heapify(index);
    }
    const data_type &get_item(std::size_t index) const {
      return items.at(index);
    }

  private:
    std::size_t heapify(std::size_t index) noexcept {
      auto new_idx = heapify_up(index);
      if (new_idx != index) {
        return new_idx;
      }
      return heapify_down(index);
    }

    std::size_t heapify_up(std::size_t index) noexcept {
      if (index == 0) {
        return index;
      }
      auto tmp = std::move(items[index]);
      while (index > 0) {
        const auto parent_idx = ((index + 1) / 2) - 1;
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
    std::size_t heapify_down(std::size_t index) noexcept {
      auto left_child_index = (2 * (index + 1)) - 1;
      if (left_child_index >= size()) {
        return index;
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
      return index;
    }

    compare<data_type> comparator;
    std::vector<data_type> items;
  };
  template <typename data_type> using max_heap = heap<data_type, std::greater>;
  template <typename data_type> using min_heap = heap<data_type>;

  template <typename data_type> struct referred_item_base {
    data_type data{};
    std::size_t heap_index{};
    referred_item_base() = default;
    referred_item_base(const referred_item_base &rhs) = delete;
    referred_item_base &operator=(const referred_item_base &rhs) = delete;
    ~referred_item_base() = default;
    auto operator<=>(const referred_item_base &rhs) const noexcept {
      return data <=> rhs.data;
    }
    referred_item_base(referred_item_base &&rhs) noexcept = default;
    referred_item_base &operator=(referred_item_base &&rhs) = default;
  };
  template <typename data_type, typename iterator_type>
  struct referred_item : public referred_item_base<data_type> {
    iterator_type iterator;
    using referred_item_base<data_type>::referred_item_base;
    using referred_item_base<data_type>::operator<=>;
    referred_item(data_type data_, iterator_type iterator_,
                  std::size_t heap_index_) noexcept
        : iterator(iterator_) {
      this->data = std::move(data_);
      this->heap_index = heap_index_;
      *iterator = heap_index_;
    }
    referred_item(referred_item &&rhs) noexcept = default;
    referred_item(const referred_item &rhs) noexcept = delete;
    referred_item &operator=(const referred_item &rhs) = delete;
    ~referred_item() = default;

    referred_item &operator=(referred_item &&rhs) noexcept {
      if (this == &rhs) {
        return *this;
      }
      this->data = std::move(rhs.data);
      *(this->iterator) = this->heap_index;
      return *this;
    }
  };
  template <typename data_type, typename iterator_type>
  struct pair_referred_item : public referred_item_base<data_type> {
    iterator_type iterator;
    using referred_item_base<data_type>::referred_item_base;
    using referred_item_base<data_type>::operator<=>;
    pair_referred_item(data_type data_, iterator_type iterator_,
                       std::size_t heap_index_) noexcept
        : iterator(iterator_) {
      this->data = std::move(data_);
      this->heap_index = heap_index_;
      iterator->second = heap_index_;
    }
    pair_referred_item(const pair_referred_item &) noexcept = delete;

    pair_referred_item &operator=(const pair_referred_item &) = delete;
    pair_referred_item(pair_referred_item &&rhs) noexcept = default;
    ~pair_referred_item() = default;

    pair_referred_item &operator=(pair_referred_item &&rhs) noexcept {
      if (this == &rhs) {
        return *this;
      }
      this->data = std::move(rhs.data);
      this->iterator = std::move(rhs.iterator);
      this->iterator->second = this->heap_index;
      return *this;
    }
  };

  template <typename container_type, typename key_type,
            template <typename T> class compare = std::less>
  class mapped_heap_base {
  public:
    [[nodiscard]] std::size_t size() const noexcept { return item_heap.size(); }
    void reserve(std::size_t n) {
      container.reserve(n);
      item_heap.reserve(n);
    }
    const auto &top_key() const { return item_heap.top().data; }
    const auto &top_data() const { return item_heap.top().iterator->first; }
    [[nodiscard]] bool empty() const noexcept { return item_heap.empty(); }
    void pop() {
      if (this->empty()) {
        return;
      }
      auto it = item_heap.top().iterator;
      item_heap.pop();
      container.erase(it);
      check_consistency();
    }

  protected:
    void check_consistency() {
#ifndef NDEBUG
      assert(container.size() == this->size());
      for (std::size_t i = 0; i < this->size(); i++) {
        auto const &item = item_heap.get_item(i);
        assert(item.iterator->second == i);
      }
#endif
    }

    container_type container;
    using data_type = typename container_type::value_type::first_type;
    using iterator_type = typename container_type::iterator;
    using item_type = pair_referred_item<key_type, iterator_type>;
    using heap_type = heap<item_type, compare>;
    heap_type item_heap;
  };

  template <typename container_type, typename data_type,
            template <typename T> class compare = std::less>
  class referred_heap_base {
  public:
    [[nodiscard]] std::size_t size() const noexcept { return item_heap.size(); }
    [[nodiscard]] bool empty() const noexcept { return item_heap.empty(); }

  protected:
    void check_consistency() {
#ifndef NDEBUG
      assert(container.size() == this->size());
      for (std::size_t i = 0; i < this->size(); i++) {
        auto const &item = item_heap.get_item(i);
        assert(*item.iterator == i);
      }
#endif
    }

    container_type container;
    using iterator_type = typename container_type::iterator;
    using item_type = referred_item<data_type, iterator_type>;
    using heap_type = heap<item_type, compare>;
    heap_type item_heap;
  };

  template <typename key_type, template <typename T> class compare = std::less>
  class window_heap
      // : public mapped_heap_base<std::deque<std::pair<key_type, std::size_t>>,
      : public referred_heap_base<std::deque<std::size_t>, key_type, compare> {
  public:
    const auto &top() const { return this->item_heap.top().data; }
    void pop() {
      if (this->empty()) {
        return;
      }
      auto const &e = this->container.back();
      this->item_heap.remove_item(e);
      this->container.pop_back();
    }

    void push(key_type data) {
      this->check_consistency();

      this->container.push_front({0});
      auto it = this->container.begin();
      this->item_heap.insert({std::move(data), it, this->item_heap.size()});
      this->check_consistency();
    }
  };
} // namespace cyy::algorithm
